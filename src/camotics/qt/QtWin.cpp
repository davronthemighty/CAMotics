/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#include "QtWin.h"
#include "Settings.h"
#include "GLView.h"

#include "ui_camotics.h"

#include <camotics/project/Project.h>
#include <camotics/sim/SimulationRun.h>
#include <camotics/sim/CutWorkpiece.h>
#include <camotics/sim/DexelHeightMap.h>
#include <camotics/sim/ToolPathTask.h>
#include <camotics/sim/SurfaceTask.h>
#include <camotics/sim/ReduceTask.h>
#include <camotics/machine/MachineModel.h>
#include <camotics/opt/Opt.h>

#include <stl/Writer.h>

#include <cbang/Application.h>
#include <cbang/os/SystemUtilities.h>
#include <cbang/os/DirectoryWalker.h>
#include <cbang/log/Logger.h>
#include <cbang/util/SmartInc.h>
#include <cbang/Catch.h>
#include <cbang/time/Timer.h>
#include <cbang/time/TimeInterval.h>

#include <QFileDialog>
#include <QKeySequence>
#include <QKeyEvent>
#include <QMessageBox>
#include <QImageWriter>
#include <QDesktopWidget>
#include <QDialog>
#include <QDir>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QShortcut>
#include <QStringListModel>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <vector>
#include <cmath>
#include <limits>

using namespace std;
using namespace cb;
using namespace CAMotics;

#define LOCK_UI_UPDATES SmartInc<unsigned> inc(inUIUpdate)
#define PROTECT_UI_UPDATE if (inUIUpdate) return; LOCK_UI_UPDATES


QtWin::QtWin(Application &app, QApplication &qtApp) :
  ui(new Ui::CAMoticsWindow), newDialog(this), newProjectDialog(this),
  exportDialog(this), aboutDialog(this), settingsDialog(this),
  donateDialog(this), findDialog(this, false), findAndReplaceDialog(this, true),
  toolDialog(this), camDialog(this), connectDialog(this), uploadDialog(this),
  fileDialog(*this), app(app), options(app.getOptions()), qtApp(qtApp),
  view(new View(valueSet)) {

  // Translation
  qtTran.load(QLocale::system(), QStringLiteral("qtbase_"));
  qtApp.installTranslator(&qtTran);
  tran.load(QLocale(), QLatin1String("camotics"), QLatin1String("_"),
            QLatin1String(":/i18n"));
  qtApp.installTranslator(&tran);

  // UI
  ui->setupUi(this);
  QActionGroup *frameGroup = new QActionGroup(this);
  frameGroup->setExclusive(true);
  frameGroup->addAction(ui->actionStockFrame);
  frameGroup->addAction(ui->actionToolFrame);
  ui->actionStockFrame->setChecked(true);
  toolTableModel = new QStandardItemModel(this);
  ui->toolTableListView->setModel(toolTableModel);
  connect(toolTableModel, &QStandardItemModel::itemChanged,
          this, &QtWin::onToolTableItemChanged);
  playPauseShortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  playPauseShortcut->setContext(Qt::WindowShortcut);
  connect(playPauseShortcut, &QShortcut::activated,
          this, &QtWin::togglePlayback);
  ui->actionPlay->setToolTip
    (tr("Play or pause (Space while Simulation or Tool View is active)"));

  // QApplication
  connect(&qtApp, SIGNAL(openProject(QString)), this,
          SLOT(on_openProject(QString)));

  // Settings dialog
  connect(&settingsDialog, SIGNAL(machineChanged(QString, QString)),
          this, SLOT(on_machineChanged(QString, QString)));

  // FileTabManager
  connect(ui->actionUndo, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionUndo_triggered()));
  connect(ui->actionRedo, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionRedo_triggered()));
  connect(ui->actionCut, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionCut_triggered()));
  connect(ui->actionCopy, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionCopy_triggered()));
  connect(ui->actionPaste, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionPaste_triggered()));
  connect(ui->actionSelectAll, SIGNAL(triggered()),
          ui->fileTabManager, SLOT(on_actionSelectAll_triggered()));
  connect(&recentProjectsMapper, SIGNAL(mapped(QString)),
          this, SLOT(openRecentProjectsSlot(QString)));
  connect(ui->fileTabManager, SIGNAL(editorClicked(QString, int)),
          this, SLOT(on_editorClicked(QString, int)));

  // Find
  connect(ui->console, SIGNAL(find()), &findDialog, SLOT(show()));
  connect(ui->console, SIGNAL(findNext()), &findDialog, SLOT(find()));
  connect(ui->console, SIGNAL(findResult(bool)), &findDialog,
          SLOT(findResult(bool)));
  connect(&findDialog, SIGNAL(find(QString, bool, int)), ui->console,
          SLOT(on_find(QString, bool, int)));

  // Find & Replace
  connect(ui->fileTabManager, SIGNAL(find()), &findAndReplaceDialog,
          SLOT(show()));
  connect(ui->fileTabManager, SIGNAL(findNext()), &findAndReplaceDialog,
          SLOT(find()));
  connect(ui->fileTabManager, SIGNAL(findResult(bool)), &findAndReplaceDialog,
          SLOT(findResult(bool)));
  connect(&findAndReplaceDialog, SIGNAL(find(QString, bool, int)),
          ui->fileTabManager, SLOT(on_find(QString, bool, int)));
  connect(&findAndReplaceDialog,
          SIGNAL(replace(QString, QString, bool, int, bool)),
          ui->fileTabManager,
          SLOT(on_replace(QString, QString, bool, int, bool)));

  // BBCtrl
  connect(&connectDialog, SIGNAL(connect()), this, SLOT(on_bbctrlConnect()));
  connect(&connectDialog, SIGNAL(disconnect()), this,
          SLOT(on_bbctrlDisconnect()));

  // ConcurrentTaskManager
  taskMan.addObserver(this);
  taskCompleteEvent = QEvent::registerEventType();

  // Hide unimplemented console buttons
  ui->errorsCheckBox->setVisible(false);
  ui->warningsCheckBox->setVisible(false);

  // Hide unfinished optimize
  ui->actionOptimize->setVisible(false);

  // Set action shortcuts
  ui->actionZoomIn->setShortcut(QString("Alt+-"));
  ui->actionZoomOut->setShortcuts({QString("Alt+="), QString("Alt++")});
  ui->actionZoomAll->setShortcut(QString("Alt+A"));
  ui->actionToggleConsole->setShortcut(QString("Alt+C"));
  ui->actionTopView->setShortcut(QString("Alt+1"));
  ui->actionFrontView->setShortcut(QString("Alt+2"));
  ui->actionBackView->setShortcut(QString("Alt+3"));
  ui->actionLeftView->setShortcut(QString("Alt+4"));
  ui->actionRightView->setShortcut(QString("Alt+5"));
  ui->actionBottomView->setShortcut(QString("Alt+6"));
  ui->actionIsoView->setShortcut(QString("Alt+7"));
  ui->actionRun->setShortcuts({QString("Ctrl+R"), QString("F5")});

  // Load icons
  playIcon.addFile(QString::fromUtf8(":/icons/play.png"), QSize(),
                   QIcon::Normal, QIcon::Off);
  pauseIcon.addFile(QString::fromUtf8(":/icons/pause.png"), QSize(),
                    QIcon::Normal, QIcon::Off);
  forwardIcon.addFile(QString::fromUtf8(":/icons/forward.png"), QSize(),
                      QIcon::Normal, QIcon::Off);
  backwardIcon.addFile(QString::fromUtf8(":/icons/backward.png"), QSize(),
                       QIcon::Normal, QIcon::Off);

  // Load data
  loadMachines();
  loadExamples();
  loadRecentProjects();
  loadLanguageMenu();

  // Add docks to View menu
  QMenu *menu = new QMenu;
  QList<QDockWidget *> docks = findChildren<QDockWidget *>();
  for (int i = 0; i < docks.size(); i++)
    menu->addAction(docks.at(i)->toggleViewAction());
  ui->actionDocks->setMenu(menu);

  // Add toolbars to View menu
  menu = new QMenu;
  QList<QToolBar *> toolBars = findChildren<QToolBar *>();
  for (int i = 0; i < toolBars.size(); i++)
    menu->addAction(toolBars.at(i)->toggleViewAction());
  ui->actionToolbars->setMenu(menu);

  // Select workpiece radio
  ui->automaticCuboidRadioButton->setChecked(true);

  // Keep simulation and playback state in one fixed-width status-bar slot.
  statusLabel = new QLabel;
  statusLabel->setMinimumWidth(680);
  statusLabel->setFixedHeight(24);
  statusLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  statusLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  statusBar()->addPermanentWidget(statusLabel, 1);
  updateStatusDisplay(false);

  // Setup console stream
  consoleStream = new LineBufferStream<ConsoleWriter>(*ui->console);
  Logger::instance().setScreenStream(*consoleStream);
}


QtWin::~QtWin() {
  try {
    if (!bbCtrlAPI.isNull()) bbCtrlAPI->disconnectCNC(); // Avoid crash
    saveAllState();
    Logger::instance().setScreenStream(cout);
  } CATCH_ERROR;
}


void QtWin::init() {
  // Start animation timer
  animationTimer.setSingleShot(false);
  connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animate()));
  animationTimer.start(50);

  // Simulation and Tool View tabs are not closeable
  ui->fileTabManager->setTabsClosable(true);
  QTabBar *tabBar = ui->fileTabManager->findChild<QTabBar *>();
  for (int i = 0; i < tabBar->count(); i++) {
    tabBar->setTabButton(i, QTabBar::RightSide, 0);
    tabBar->setTabButton(i, QTabBar::LeftSide, 0);
  }

  // Hide console by default
  hideConsole();

  // Init GUI
  ui->fileTabManager->setCurrentIndex(0);
  updateActions();

  // Init Layout
  saveFullLayout();
  defaultLayout();
  setDefaultGeometry();
  restoreAllState();

  // Update fullscreen menu item
  if (windowState() & Qt::WindowFullScreen)
    ui->actionFullscreen->setChecked(true);

  // Observe values
  valueSet["play_speed"]->add(this, &QtWin::updatePlaySpeed);
  valueSet["view_flags"]->add(this, &QtWin::updateViewFlags);
  valueSet["play_direction"]->add(this, &QtWin::updatePlayDirection);
  valueSet["time_ratio"]->add(this, &QtWin::updateTimeRatio);

  valueSet["x"]->add(this, &QtWin::updateX);
  valueSet["y"]->add(this, &QtWin::updateY);
  valueSet["z"]->add(this, &QtWin::updateZ);

  valueSet["current_time"]->add(this, &QtWin::updateCurrentTime);
  valueSet["current_distance"]->add(this, &QtWin::updateCurrentDistance);
  valueSet["remaining_time"]->add(this, &QtWin::updateRemainingTime);
  valueSet["remaining_distance"]->add(this, &QtWin::updateRemainingDistance);
  valueSet["total_time"]->add(this, &QtWin::updateTotalTime);
  valueSet["total_distance"]->add(this, &QtWin::updateTotalDistance);
  valueSet["percent_time"]->add(this, &QtWin::updatePercentTime);
  valueSet["percent_distance"]->add(this, &QtWin::updatePercentDistance);

  valueSet["tool"]->add(this, &QtWin::updateTool);
  valueSet["feed"]->add(this, &QtWin::updateFeed);
  valueSet["speed"]->add(this, &QtWin::updateSpeed);
  valueSet["direction"]->add(this, &QtWin::updateDirection);
  valueSet["program_file"]->add(this, &QtWin::updateProgramFile);
  valueSet["program_line"]->add(this, &QtWin::updateProgramLine);

  valueSet["move_start_x"]->add(this, &QtWin::updateMoveStartX);
  valueSet["move_start_y"]->add(this, &QtWin::updateMoveStartY);
  valueSet["move_start_z"]->add(this, &QtWin::updateMoveStartZ);
  valueSet["move_end_x"]->add(this, &QtWin::updateMoveEndX);
  valueSet["move_end_y"]->add(this, &QtWin::updateMoveEndY);
  valueSet["move_end_z"]->add(this, &QtWin::updateMoveEndZ);
  valueSet["move_distance_x"]->add(this, &QtWin::updateMoveDistanceX);
  valueSet["move_distance_y"]->add(this, &QtWin::updateMoveDistanceY);
  valueSet["move_distance_z"]->add(this, &QtWin::updateMoveDistanceZ);
  valueSet["move_distance_total"]->add(this, &QtWin::updateMoveDistanceTotal);

  valueSet.updated();
}


void QtWin::show() {
  QMainWindow::show();
  if (!ui->fileTabManager->currentIndex())
    ui->simulationView->setFocus(Qt::OtherFocusReason);
  donateDialog.onStartup();
}


void QtWin::setUnitLabel(QLabel *label, double value, int precision,
                         bool withUnit) {
  if (numeric_limits<double>::max() == abs(value) || isinf(value) ||
      isnan(value)) {
    label->setText("nan");
    return;
  }

  double scale = isMetric() ? 1.0 : 1.0 / 25.4;
  label->setText(QString().sprintf("%.*f%s", precision, value * scale,
                                   withUnit ? (isMetric() ? "mm" : "in") : ""));
}


void QtWin::loadLanguageMenu() {
  QActionGroup *langGroup = new QActionGroup(ui->menuLanguage);
  langGroup->setExclusive(true);

  connect(langGroup, SIGNAL(triggered(QAction *)), this,
          SLOT(on_languageChanged(QAction *)));

  // Format systems language
  QString defaultLocale = QLocale::system().name();
  defaultLocale.truncate(defaultLocale.lastIndexOf('_'));

  // English
  QAction *action = ui->menuLanguage->addAction("English");
  action->setCheckable(true);
  action->setData("en");
  action->setIcon(QIcon(QString(":/flags/en.png")));
  action->setChecked(true);
  langGroup->addAction(action);

  QDir dir(":/i18n");
  QStringList fileNames = dir.entryList(QStringList("camotics_*.qm"));

  for (int i = 0; i < fileNames.size(); i++) {
    // Get locale from filename
    QString locale;
    locale = fileNames[i];
    locale.truncate(locale.lastIndexOf('.'));
    locale.remove(0, locale.lastIndexOf('_') + 1);

    QString lang = QLocale::languageToString(QLocale(locale).language());
    QString flag = QString(":/flags/%1.png").arg(locale);
    QAction *action = ui->menuLanguage->addAction(lang);
    action->setCheckable(true);
    action->setData(locale);
    action->setIcon(QIcon(flag));
    langGroup->addAction(action);

    if (defaultLocale == locale) action->setChecked(true);
  }
}


void QtWin::switchTranslator(QTranslator &translator, const QString &filename) {
  // remove the old translator
  qtApp.removeTranslator(&translator);

  // load the new translator
  if (translator.load(QLatin1String(":/i18n/") + filename))
    qtApp.installTranslator(&translator);
}


void QtWin::loadLanguage(const QString &lang) {
  if (currentLang == lang) return;

  LOG_INFO(1, "Switching language from " << currentLang.toStdString()
           << " to " << lang.toStdString());

  currentLang = lang;
  QLocale locale = QLocale(lang);
  QLocale::setDefault(locale);

  qtApp.removeTranslator(&tran);
  if (tran.load(QString(":/i18n/camotics_%1.qm").arg(lang)))
    qtApp.installTranslator(&tran);

  qtApp.removeTranslator(&qtTran);
  if (qtTran.load(QString("qt_%1.qm").arg(lang)))
    qtApp.installTranslator(&qtTran);

  QString languageName = QLocale::languageToString(locale.language());
  ui->statusbar->showMessage(tr("Language changed to %1").arg(languageName));
}


void QtWin::loadMachine(const string &name) {
  if (view->machine.isNull() || name != view->machine->getName()) {
    view->setMachine(machines[name]);
    if (toolPath.isSet()) validateMachinePath(*toolPath);
    redraw();
  }
}


void QtWin::setReferenceFrame(View::ReferenceFrame frame) {
  view->setReferenceFrame(frame);
  ui->actionStockFrame->setChecked(frame == View::STOCK_FRAME);
  ui->actionToolFrame->setChecked(frame == View::TOOL_FRAME);
  LOG_INFO(1, "GUI reference frame: "
           << (frame == View::TOOL_FRAME ? "tool" : "stock"));
  updateStatusDisplay(lastStatusActive ||
                      view->isFlagSet(View::PLAY_FLAG));
  redraw(true);
}


void QtWin::setSimulationToolEnabled(unsigned tool, bool enabled) {
  const bool wasDisabled = disabledSimulationTools.count(tool);
  if (enabled) disabledSimulationTools.erase(tool);
  else disabledSimulationTools.insert(tool);
  if (wasDisabled == !enabled) return;

  {
    QSignalBlocker blocker(toolTableModel);
    for (int row = 0; row < toolTableModel->rowCount(); row++) {
      QStandardItem *item = toolTableModel->item(row);
      if (!item || item->data(Qt::UserRole).toUInt() != tool) continue;
      item->setCheckState(enabled ? Qt::Checked : Qt::Unchecked);
      item->setForeground(enabled ? QBrush() : QBrush(QColor("#888888")));
      break;
    }
  }

  if (view->path.isSet())
    view->path->setDisabledTools(disabledSimulationTools);
  LOG_INFO(1, "GUI simulation tool " << tool << " "
           << (enabled ? "enabled" : "disabled"));
  updateStatusDisplay(lastStatusActive ||
                      view->isFlagSet(View::PLAY_FLAG));

  // The displayed path and cutter motion remain complete.  Restart only the
  // stock-removal simulation using a private filtered copy of the path.
  if (toolPath.isSet()) {
    taskMan.interrupt();
    loadToolPath(toolPath, true);
  }
}


void QtWin::loadMachines() {
  try {
    vector<string> paths;

    paths.push_back("machines");

    QStringList list = QStandardPaths::locateAll
      (QStandardPaths::GenericDataLocation, "camotics/machines",
       QStandardPaths::LocateDirectory);

    for (int i = 0; i < list.size(); i++)
      paths.push_back(list.at(i).toUtf8().constData());

#ifdef __APPLE__
    paths.push_back("../SharedSupport/machines");
#endif

    string root = ".";
    string appPath = QCoreApplication::applicationFilePath().toUtf8().data();
    if (appPath.empty()) LOG_WARNING("Couldn't get application path");
    else root = SystemUtilities::dirname(appPath);

    for (unsigned i = 0; i < paths.size(); i++) {
      string path = paths[i];
      if (path.empty()) continue;
      if (path[0] != '/') {
        path.insert(0, "/");
        path.insert(0, root);
      }

      if (SystemUtilities::isDirectory(path)) {
        DirectoryWalker walker(path, ".*\\.json(.bz2)?", 1);

        while (walker.hasNext()) {
          string filename = walker.next();

          try {
            SmartPointer<MachineModel> machine = new MachineModel;
            machine->read(*SystemUtilities::iopen(filename, true));

            if (machines.find(machine->getName()) != machines.end()) continue;

            machines[machine->getName()] = machine;
            LOG_INFO(1, "Loaded machine " << machine->getName());
          } CATCH_ERROR;
        }
      }
    }

    // Add machine names to settings dialog
    for (auto &it: machines)
      settingsDialog.addMachine(it.first);

    // Load machine
    string machine =
      Settings().get("Settings/Machine", "").toString().toUtf8().data();
    if (machine.empty()) machine = "Taig Mini Mill";
    loadMachine(machine);
   } CATCH_ERROR;
}


void QtWin::loadDefaultExample() {
  if (defaultExample.empty()) newProject();
  else {
    openProject(defaultExample);
    snapView('t');
  }
}


void QtWin::loadExamples() {
  try {
    vector<string> paths;
    paths.push_back(SystemUtilities::getPathPrefix() +
                    "/share/doc/camotics/examples");
    paths.push_back("../SharedSupport/examples");
    paths.push_back("examples");

    string root = ".";
    string appPath =
      QCoreApplication::applicationFilePath().toUtf8().data();
    if (appPath.empty()) LOG_WARNING("Couldn't get application path");
    else root = SystemUtilities::dirname(appPath);

    for (unsigned i = 0; i < paths.size(); i++) {
      string path = paths[i];
      if (path.empty()) continue;
      if (path[0] != '/') {
        path.insert(0, "/");
        path.insert(0, root);
      }

      if (SystemUtilities::isDirectory(path)) {
        typedef map<string, string> examples_t;
        examples_t examples;
        DirectoryWalker walker(path, ".*\\.camotics", 2);

        while (walker.hasNext()) {
          string filename = walker.next();
          string dirname = SystemUtilities::dirname(filename);
          string basename = SystemUtilities::basename(filename);
          string name = SystemUtilities::splitExt(basename)[0];

          if (String::endsWith(dirname, "/" + name)) {
            name = String::trim(name);
            name = String::transcode(name, "_-", "  ");
            name = String::capitalize(name);

            examples[name] = filename;
            if (name == "Camotics") defaultExample = filename;
          }
        }

        if (examples.empty()) continue;

        QMenu *menu = new QMenu;

        ui->actionExamples->setMenu(menu);
        ui->actionExamples->setEnabled(true);

        examples_t::iterator it;
        for (it = examples.begin(); it != examples.end(); it++) {
          string name = it->first;
          string path = it->second;

          QAction *action = menu->addAction(QString::fromUtf8(name.c_str()));
          action->setToolTip(QString::fromUtf8(path.c_str()));
          connect(action, SIGNAL(triggered()), this,
                  SLOT(on_actionExamples_triggered()));
        }

        break;
      }
    }

  } CATCH_ERROR;
}


void QtWin::loadRecentProjects() {
  QSettings settings;
  int size = settings.beginReadArray("recentProjects");
  QStringList recents;
  QSet<QString> unique;

  for (int i = 0; i < size; i++) {
    settings.setArrayIndex(i);
    QString recent = settings.value("fileName").toString();
    QFileInfo fi(recent);

    if (fi.exists() && fi.isReadable()) {
      QString canonical = fi.canonicalFilePath();
      if (unique.contains(canonical)) continue;
      unique.insert(canonical);

      QAction *action = ui->menuRecent_projects->
        addAction(recent, &recentProjectsMapper, SLOT(map()));
      recentProjectsMapper.setMapping(action, recent);
      recents.append(recent);
    }
  }

  settings.endArray();

  // If any of the recent projects are inaccessible, rewrite the whole list.
  if (recents.size() != size) {
    settings.beginWriteArray("recentProjects", recents.size());
    int i = 0;

    foreach (QString recent, recents) {
      settings.setArrayIndex(i++);
      settings.setValue("fileName", recent);
    }

    settings.endArray();
  }

  settings.sync();
}


void QtWin::updateRecentProjects(const string &_filename) {
  QSettings settings;
  QString filename = QString::fromUtf8(_filename.c_str());
  int size = settings.beginReadArray("recentProjects");
  QStringList recents;

  for (int i = 0; i < size; i++) {
    settings.setArrayIndex(i);
    QString recent = settings.value("fileName").toString();

    // Skip the currently opened project from the recents
    // if it was already present in the recents projects list.
    if (recent != filename) recents.append(recent);
  }
  settings.endArray();

  // Reload the actions in the recent menu too to move the opened file to the
  // first position.
  foreach (QAction *action, ui->menuRecent_projects->actions()) {
    ui->menuRecent_projects->removeAction(action);
    delete action;
  }

  // There is no way to remove an array item from QSettings
  // rewrite the whole array if the order changed.
  recents.prepend(filename);

  settings.beginWriteArray("recentProjects");
  const int maxRecentsSize = 20;
  int i = 0;

  foreach (QString recent, recents) {
    QAction *action = ui->menuRecent_projects->
      addAction(recent, &recentProjectsMapper, SLOT(map()));
    recentProjectsMapper.setMapping(action, recent);

    settings.setArrayIndex(i);
    settings.setValue("fileName", recent);

    if (++i == maxRecentsSize) break;
  }

  settings.endArray();
}


void QtWin::saveAllState() {
  Settings settings;
  settings.set("MainWindow/State", saveState());
  settings.set("MainWindow/Geometry", saveGeometry());
  settings.set("Console/Splitter", ui->splitter->saveState());
  settings.set("File/Splitter", ui->splitterFile->saveState());
}


void QtWin::restoreAllState() {
  Settings settings;
  restoreState(settings.get("MainWindow/State").toByteArray());
  restoreGeometry(settings.get("MainWindow/Geometry").toByteArray());
  ui->splitter->restoreState(settings.get("Console/Splitter").toByteArray());
  ui->splitterFile->restoreState(settings.get("File/Splitter").toByteArray());
}


void QtWin::setDefaultGeometry() {
  int width = 1200;
  int height = 800;

  QDesktopWidget dw;
  QRect screen = dw.availableGeometry(this);
  QRect geom = geometry();
  QRect frame = frameGeometry();

  if (screen.width() < width) width = screen.width();
  if (screen.height() < height) height = screen.height();

  width -= frame.width() - geom.width();
  height -= frame.height() - geom.height();

  int x = (screen.width() - width) / 2;
  int y = (screen.height() - height) / 2;

  setGeometry(QRect(x, y, width, height));
}


void QtWin::saveFullLayout() {
  fullLayoutState = saveState();
}


void QtWin::fullLayout() {
  restoreState(fullLayoutState);
  showConsole();
  showText();
}


void QtWin::defaultLayout() {
  restoreState(fullLayoutState);

  // Hide all closable docks
  QList<QDockWidget *> docks = findChildren<QDockWidget *>();
  for (int i = 0; i < docks.size(); i++)
    if (docks.at(i)->features() & QDockWidget::DockWidgetClosable)
      docks.at(i)->setVisible(false);

  hideConsole();
  hideText();
}


void QtWin::minimalLayout() {
  defaultLayout();

  // Hide all toolbars
  QList<QToolBar *> toolBars = findChildren<QToolBar *>();
  for (int i = 0; i < toolBars.size(); i++)
    toolBars.at(i)->setVisible(false);

  hideConsole();
  hideText();
}


void QtWin::snapView(char v) {
  view->resetView(v);
  redraw();
}


void QtWin::showMessage(const QString &msg, bool log) {
  if (log) LOG_INFO(1, msg.toStdString());
  ui->statusbar->showMessage(msg, 30000);
}


void QtWin::showMessage(const char *msg, bool log) {
  showMessage(QString::fromUtf8(msg), log);
}


void QtWin::message(const QString &msg) {
  QMessageBox::information(this, "CAMotics", msg, QMessageBox::Ok);
}


void QtWin::warning(const QString &msg) {
  QMessageBox::warning(this, "CAMotics", msg, QMessageBox::Ok);
}


SmartPointer<GCode::ToolPath> QtWin::makeSimulationToolPath() const {
  if (toolPath.isNull() || disabledSimulationTools.empty()) return toolPath;

  SmartPointer<GCode::ToolPath> filtered =
    new GCode::ToolPath(toolPath->getTools());
  uint64_t skipped = 0;
  for (const GCode::Move &move: *toolPath) {
    GCode::Move copy = move;
    if (disabledSimulationTools.count(move.getTool())) {
      copy = GCode::Move
        (move.getType(), move.getStart(), move.getEnd(), move.getStartTime(),
         -1, move.getFeed(), move.getSpeed(), move.getLine(),
         move.getFilename(), move.getTime());
      skipped++;
    }
    filtered->move(copy);
  }
  LOG_INFO(1, "GUI simulation tool filter: disabled_tools="
           << disabledSimulationTools.size()
           << " skipped_moves=" << skipped
           << " retained_moves=" << filtered->size());
  return filtered;
}


void QtWin::validateMachinePath(const GCode::ToolPath &toolPath) {
  machineConstraintWarnings = 0;
  machineConstraintSummary.clear();
  if (view->machine.isNull() || !view->machine->hasSpecs()) return;

  const MachineModel &machine = *view->machine;
  const Vector3D pathDims = toolPath.getBounds().getDimensions();
  const Vector3D workArea = machine.getWorkArea();
  QStringList warnings;
  static const char *axisNames[] = {"X", "Y", "Z"};
  for (unsigned axis = 0; axis < 3; axis++)
    if (0 < workArea[axis] && workArea[axis] + 1e-9 < pathDims[axis])
      warnings << tr("%1 travel span %2 mm exceeds %3 mm")
        .arg(axisNames[axis]).arg(pathDims[axis], 0, 'f', 3)
        .arg(workArea[axis], 0, 'f', 3);

  double clearanceUsed = project.isNull() ? 0 :
    project->getWorkpiece().getBounds().getDimensions().z();
  if (!project.isNull() && !toolPath.empty())
    clearanceUsed = max
      (clearanceUsed, toolPath.getBounds().getMax().z() -
       project->getWorkpiece().getBounds().getMin().z());
  if (0 < machine.getGantryClearance() &&
      machine.getGantryClearance() + 1e-9 < clearanceUsed)
    warnings << tr("required Z clearance %1 mm exceeds %2 mm")
      .arg(clearanceUsed, 0, 'f', 3)
      .arg(machine.getGantryClearance(), 0, 'f', 3);

  double maxCutFeed = 0;
  double maxSpindle = 0;
  for (const GCode::Move &move: toolPath) {
    if (move.getType() != GCode::MoveType::MOVE_RAPID)
      maxCutFeed = max(maxCutFeed, move.getFeed());
    maxSpindle = max(maxSpindle, fabs(move.getSpeed()));
  }
  if (0 < machine.getMaxTravelSpeed() &&
      machine.getMaxTravelSpeed() + 1e-9 < maxCutFeed)
    warnings << tr("cut feed %1 mm/min exceeds %2 mm/min")
      .arg(maxCutFeed, 0, 'f', 1)
      .arg(machine.getMaxTravelSpeed(), 0, 'f', 1);
  if (0 < machine.getMaxSpindleSpeed() &&
      machine.getMaxSpindleSpeed() + 1e-9 < maxSpindle)
    warnings << tr("spindle speed %1 RPM exceeds %2 RPM")
      .arg(maxSpindle, 0, 'f', 0)
      .arg(machine.getMaxSpindleSpeed(), 0, 'f', 0);

  machineConstraintWarnings = warnings.size();
  machineConstraintSummary =
    tr("%1: %2 visualization; envelope/feed/spindle checks only. "
       "No runout, backlash, deflection, or collision physics.")
      .arg(QString::fromUtf8(machine.getName().c_str()))
      .arg(machine.getModelKind().empty() ? tr("legacy") :
           QString::fromUtf8(machine.getModelKind().c_str()));
  if (!warnings.empty())
    machineConstraintSummary += tr("\nWarnings: %1").arg(warnings.join("; "));

  LOG_INFO(1, "Machine profile diagnostics: name=" << machine.getName()
           << " path_dims=" << pathDims
           << " work_area=" << workArea
           << " clearance_used=" << clearanceUsed
           << " gantry_clearance=" << machine.getGantryClearance()
           << " max_cut_feed=" << maxCutFeed
           << " max_travel_speed=" << machine.getMaxTravelSpeed()
           << " max_spindle=" << maxSpindle
           << " max_spindle_speed=" << machine.getMaxSpindleSpeed()
           << " nominal_resolution=" << machine.getNominalResolution()
           << " spindle_runout=" << machine.getSpindleRunout()
           << " warnings=" << machineConstraintWarnings);
  for (const QString &warning: warnings)
    LOG_WARNING(machine.getName() << ": " << warning.toStdString());
}


void QtWin::loadToolPath(const SmartPointer<GCode::ToolPath> &toolPath,
                         bool simulate) {
  this->toolPath = toolPath;

  // The parsed path can reference tools that were not declared in the project.
  // Refresh after parsing so the table describes what is actually loaded.
  updateToolTables();

  // Update changed Project settings
  project->getWorkpiece().update(*toolPath);

  // Setup view
  view->setToolPath(toolPath);
  view->path->setDisabledTools(disabledSimulationTools);
  view->setWorkpiece(project->getWorkpiece().getBounds());
  validateMachinePath(*toolPath);

  // Update UI
  loadWorkpiece();
  updateBounds();

  redraw();

  // Clear old surface
  surface.release();
  simRun.release();
  ui->actionDexelHeightMap->setEnabled(false);
  clearDexelHeightMapWindow(tr("Updating the simulation result..."));
  view->setSurface(0);
  view->setMoveLookup(0);

  if (!simulate) return setStatusActive(false);

  // Auto play
  if (autoPlay) {
    autoPlay = false;
    view->path->setByRatio(0);
    view->setFlag(View::PLAY_FLAG);
    view->setReverse(false);
  }

  // Simulation
  SmartPointer<GCode::PlannerConfig> planConf =
    settingsDialog.getPlannerEnabled() ?
    new GCode::PlannerConfig(settingsDialog.getPlannerConfig()) : 0;
  RenderMode mode =
    (RenderMode::enum_t)Settings().get("Settings/RenderMode", 0).toInt();
  SmartPointer<GCode::ToolPath> simulationPath = makeSimulationToolPath();
  Simulation sim(simulationPath, planConf, 0,
                 project->getWorkpiece().getBounds(),
                 project->getResolution(), view->getTime(),
                 mode, options["threads"].toInteger());

  // Exact accelerators for the full-MC reference/fallback path.
  sim.toolSweepXYBins = 64;
  sim.toolSweepStockBounds = true;
  sim.backendPolicy = settingsDialog.getSimulationBackendPolicy();
  // The deterministic builder remains covered by retained topology tests.
  // Avoid rescanning every production GUI mesh unless explicitly requested.
  sim.validateDexelTopology =
    options["validate-dexel-topology"].toBoolean();

  if (options["simulation-backend"].hasValue()) {
    string backend = options["simulation-backend"].toString();
    if (backend == "auto-dexel")
      sim.backendPolicy = SimulationBackendPolicy::AUTO_DEXEL;
    else if (backend == "full-mc")
      sim.backendPolicy = SimulationBackendPolicy::FULL_MC;
    else LOG_WARNING("Unknown GUI simulation backend '" << backend
                     << "'; using "
                     << simulationBackendPolicyName(sim.backendPolicy));
  }

  simulationBackendPolicy = sim.backendPolicy;
  simulationBackendResolved = false;
  simulationBackendFallback = false;
  updateStatusDisplay(lastStatusActive ||
                      view->isFlagSet(View::PLAY_FLAG));

  // Load new surface
  // Tool-path installation can emit a position change.  This initial render
  // already uses the current view time, so do not immediately queue it again.
  positionChanged = false;
  // A playing view immediately replaces the complete surface with time zero.
  // Avoid constructing a final exact boundary frame that cannot be displayed.
  taskMan.addTask
    (new SurfaceTask(sim, true,
                     !view->isFlagSet(View::PLAY_FLAG)));
}


void QtWin::uploadGCode() {
  if (gcode.empty() || bbCtrlAPI.isNull() || !bbCtrlAPI->isConnected())
    return;

  QString current = uploadDialog.getFilename();
  QString filename = QString().fromUtf8(project->getUploadFilename().c_str());

  if (filename != current || !uploadDialog.isAutomatic()) {
    uploadDialog.setFilename(filename);

    if (uploadDialog.exec() != QDialog::Accepted) {
      uploadDialog.setFilename(current);
      return;
    }

    filename = uploadDialog.getFilename();
  }

  bbCtrlAPI->setFilename(filename.toUtf8().data());
  bbCtrlAPI->uploadGCode(gcode);
  showMessage(tr("Uploading %1 to %2")
              .arg(filename).arg(connectDialog.getAddress()));
}


void QtWin::toolPathComplete(ToolPathTask &task) {
  if (task.getErrorCount()) {
    auto msg = tr("Errors were encountered during tool path generation.  "
      "See the console output for more details");

    QMessageBox::critical(this, tr("Tool path errors"), msg, QMessageBox::Ok);

    showConsole();
  }

  gcode = task.getGCode();
  exportDialog.enableGCode(!gcode.empty());
  uploadGCode();
  loadToolPath(task.getPath(), !task.getErrorCount());
}


void QtWin::surfaceComplete(SurfaceTask &task) {
  simRun = task.getSimRun();
  lastSimulationBackend = task.getBackend();
  simulationBackendResolved = true;
  simulationBackendFallback = task.hasFallbackReason();
  const double targetTolerance = max(1.0, fabs(task.getTargetTime())) * 1e-12;
  const double playbackLag = fabs
    (pendingSimulationTime - task.getTargetTime()) /
    max(1U, view->getSpeed());
  const bool publishLaggedPlayback =
    view->isFlagSet(View::PLAY_FLAG) && playbackLag <= 0.25;
  const bool obsolete = task.getGeneration() && positionChanged &&
    targetTolerance < fabs(pendingSimulationTime - task.getTargetTime()) &&
    !publishLaggedPlayback;
  if (obsolete) {
    setStatusActive(false);
    simulationDiscarded++;
    updateStatusDisplay(view->isFlagSet(View::PLAY_FLAG));
    LOG_INFO(1, "GUI simulation discarded: generation="
             << task.getGeneration() << " target=" << task.getTargetTime()
             << " pending_target=" << pendingSimulationTime
             << " completed=" << simulationCompleted
             << " discarded=" << simulationDiscarded);
    return;
  }

  surface = task.getSurface();
  if (surface.isNull()) simRun.release();
  const bool hasDexelGrid =
    dynamic_cast<Dexel::GridSurface *>(surface.get()) != 0;
  exportDialog.enableSurface(!surface.isNull());
  exportDialog.enableSimData(true);

  const bool initializePlayback = !task.hasTargetTime() &&
    view->isFlagSet(View::PLAY_FLAG);
  if (initializePlayback) {
    // Playback starts from uncut stock while the exact time-zero surface is
    // restored from the retained Dexel state.
    view->setSurface(0);
    ui->actionDexelHeightMap->setEnabled(false);
    clearDexelHeightMapWindow(tr("Waiting for the current playback state..."));
    double ratio = view->path->getRequestedTimeRatio();
    pendingSimulationTime = isnan(ratio) ? 0 :
      ratio * view->path->getTotalTime();
    simulationRequests++;
    if (positionChanged) simulationCoalesced++;
    positionChanged = true;
    LOG_INFO(1, "GUI playback placeholder: uncut workpiece target="
             << pendingSimulationTime);
  } else {
    view->setSurface(surface);
    ui->actionDexelHeightMap->setEnabled(hasDexelGrid);
    updateDexelHeightMapWindow();
  }
  if (simRun.isSet()) view->setMoveLookup(simRun->getMoveLookup());

  redraw();

  setStatusActive(false);

  if (task.getGeneration()) {
    simulationCompleted++;
    updateStatusDisplay(view->isFlagSet(View::PLAY_FLAG));
    LOG_INFO(1, "GUI simulation completed: generation="
             << task.getGeneration() << " target=" << task.getTargetTime()
             << " pending=" << (positionChanged ? 1 : 0)
             << " playback_lag=" << playbackLag
             << " completed=" << simulationCompleted
             << " discarded=" << simulationDiscarded);
  }

  if (task.shouldQuit()) {
    showMessage(tr("Simulation cancelled"));
    return;
  }

  QString backend = task.getBackend() == SimulationBackend::DEXEL ?
    tr("Dexel") : tr("Full marching cubes");
  QString status = tr("Simulation complete: %1").arg(backend);
  if (task.hasFallbackReason())
    status += tr(" (Auto fallback: %1)").arg(QString::fromUtf8
      (Dexel::reasonName(task.getFallbackReason())));
  showMessage(status);

  if (!surface.isNull() && options["simulation-output"].hasValue()) {
    string filename = options["simulation-output"].toString();
    SmartPointer<iostream> stream =
      SystemUtilities::open(filename, ios::out);
    surface->writeSTL(*stream, true, "CAMotics Surface",
                      simRun->getSimulation().computeHash());
    LOG_INFO(1, "GUI simulation surface written: " << filename);
  }

  if (!dexelGridWindowTested &&
      options["test-dexel-grid-window"].toBoolean()) {
    if (!runDexelGridWindowTest()) return;
    dexelGridWindowTested = true;
  }

  if (!viewControlsTested && options["test-view-controls"].toBoolean()) {
    viewControlsTested = true;
    runViewControlsTest();
  }

  if (injectSimulationTestSeekBurst()) return;
  if (seekNextSimulationTestPosition()) return;
  if (!simulationEndInjected &&
      options["simulation-go-to-end"].toBoolean()) {
    simulationEndInjected = true;
    on_actionSimulationEnd_triggered();
    return;
  }

  if (autoCloseAfterSimulation) app.requestExit();
}


void QtWin::runViewControlsTest() {
  GLView *glView = ui->simulationView;
  const Vector2D originalTranslation = view->getTranslation();
  const QuaternionD originalRotation = view->getRotation();
  const double originalZoom = view->getZoom();
  const bool originalPlaying = view->isFlagSet(View::PLAY_FLAG);
  const double originalRatio = view->path->getRequestedTimeRatio();
  const QPointF local(glView->width() / 2.0, glView->height() / 2.0);
  const QPointF global(glView->mapToGlobal(local.toPoint()));

  QWheelEvent panEvent
    (local, global, QPoint(18, 24), QPoint(), Qt::NoButton,
     Qt::NoModifier, Qt::ScrollUpdate, false,
     Qt::MouseEventSynthesizedBySystem);
  QCoreApplication::sendEvent(glView, &panEvent);
  const Vector2D panned = view->getTranslation();
  const bool panPassed =
    1e-9 < fabs(panned.x() - originalTranslation.x()) ||
    1e-9 < fabs(panned.y() - originalTranslation.y());
  view->setTranslation(originalTranslation);

  QWheelEvent zoomEvent
    (local, global, QPoint(0, 30), QPoint(), Qt::NoButton,
     Qt::ControlModifier, Qt::ScrollUpdate, false,
     Qt::MouseEventSynthesizedBySystem);
  QCoreApplication::sendEvent(glView, &zoomEvent);
  const bool zoomPassed = 1e-9 < fabs(view->getZoom() - originalZoom);
  view->setZoom(originalZoom);

  QWheelEvent orbitEvent
    (local, global, QPoint(20, 12), QPoint(), Qt::NoButton,
     Qt::AltModifier, Qt::ScrollUpdate, false,
     Qt::MouseEventSynthesizedBySystem);
  QCoreApplication::sendEvent(glView, &orbitEvent);
  const QuaternionD orbited = view->getRotation();
  bool orbitPassed = false;
  for (unsigned i = 0; i < 4; i++)
    orbitPassed |= 1e-9 < fabs(orbited[i] - originalRotation[i]);
  view->setRotation(originalRotation);

  // Exercise the actual widget-scoped shortcut without starting from the end,
  // where normal Play behavior intentionally rewinds to the uncut stock.
  if (!view->path->isEmpty()) view->path->setByRatio(0.5);
  glView->setFocus(Qt::OtherFocusReason);
  QKeyEvent press
    (QEvent::KeyPress, Qt::Key_Space, Qt::NoModifier, QString(" "));
  QKeyEvent release
    (QEvent::KeyRelease, Qt::Key_Space, Qt::NoModifier, QString(" "));
  QCoreApplication::sendEvent(glView, &press);
  QCoreApplication::sendEvent(glView, &release);
  const bool firstSpacePassed =
    view->isFlagSet(View::PLAY_FLAG) != originalPlaying;
  QCoreApplication::sendEvent(glView, &press);
  QCoreApplication::sendEvent(glView, &release);
  const bool secondSpacePassed =
    view->isFlagSet(View::PLAY_FLAG) == originalPlaying;
  if (view->isFlagSet(View::PLAY_FLAG) != originalPlaying)
    view->toggleFlag(View::PLAY_FLAG);
  if (!view->path->isEmpty())
    view->path->setByRatio(isnan(originalRatio) ? 1 : originalRatio);

  // Verify the view-facing model notifications, not merely model storage.
  // Blocking these signals leaves rowCount() nonzero while QListView remains
  // blank, which is exactly the failure this check is intended to catch.
  int insertedToolRows = 0;
  int resetNotifications = 0;
  QMetaObject::Connection rowsConnection = connect
    (toolTableModel, &QAbstractItemModel::rowsInserted,
     [&insertedToolRows]
     (const QModelIndex &, int first, int last) {
       insertedToolRows += last - first + 1;
     });
  QMetaObject::Connection resetConnection = connect
    (toolTableModel, &QAbstractItemModel::modelReset,
     [&resetNotifications]() {resetNotifications++;});
  updateToolTables();
  disconnect(rowsConnection);
  disconnect(resetConnection);

  const QModelIndex firstTool = toolTableModel->index(0, 0);
  const QRect firstToolRect =
    ui->toolTableListView->visualRect(firstTool);
  const bool tablePassed = 0 < toolTableModel->rowCount() &&
    insertedToolRows == toolTableModel->rowCount() &&
    0 < resetNotifications && firstTool.isValid() &&
    !firstTool.data(Qt::DisplayRole).toString().isEmpty() &&
    firstToolRect.isValid() && !firstToolRect.isEmpty();
  const bool dockPassed = ui->actionDocks->menu() &&
    ui->actionDocks->menu()->actions().contains
      (ui->toolTableDockWidget->toggleViewAction());
  const bool spacePassed = firstSpacePassed && secondSpacePassed;

  LOG_INFO(1, "GUI view controls test: table_rows="
           << toolTableModel->rowCount()
           << " table_inserted=" << insertedToolRows
           << " table_resets=" << resetNotifications
           << " table_rect=" << firstToolRect.width()
           << "x" << firstToolRect.height()
           << " dock=" << (dockPassed ? "pass" : "fail")
           << " space=" << (spacePassed ? "pass" : "fail")
           << " pan=" << (panPassed ? "pass" : "fail")
           << " zoom=" << (zoomPassed ? "pass" : "fail")
           << " orbit=" << (orbitPassed ? "pass" : "fail"));

  if (!tablePassed || !dockPassed || !spacePassed || !panPassed ||
      !zoomPassed || !orbitPassed)
    THROW("GUI view controls test failed");
}


bool QtWin::runDexelGridWindowTest() {
  if (!dexelGridWindowTestStage) ui->actionDexelHeightMap->trigger();
  QDialog *dialog =
    findChild<QDialog *>("dexelHeightMapWindow");
  QLabel *imageLabel = dialog ?
    dialog->findChild<QLabel *>("dexelHeightMapImage") : 0;
  QScrollArea *scroll = dialog ?
    dialog->findChild<QScrollArea *>("dexelHeightMapScrollArea") : 0;
  QAction *zoomIn = dialog ?
    dialog->findChild<QAction *>("dexelHeightMapZoomIn") : 0;
  QAction *zoomOut = dialog ?
    dialog->findChild<QAction *>("dexelHeightMapZoomOut") : 0;
  QAction *actualSize = dialog ?
    dialog->findChild<QAction *>("dexelHeightMapActualSize") : 0;
  QAction *fitWindow = dialog ?
    dialog->findChild<QAction *>("dexelHeightMapFitWindow") : 0;
  QLabel *zoomLabel = dialog ?
    dialog->findChild<QLabel *>("dexelHeightMapZoomPercent") : 0;
  int width = dialog ? dialog->property("gridWidth").toInt() : 0;
  int height = dialog ? dialog->property("gridHeight").toInt() : 0;

  if (dexelGridWindowTestStage == 1) {
    int tool = dialog ? dialog->property("testDisabledTool").toInt() : -1;
    QByteArray initialHash = dialog ?
      dialog->property("testInitialHash").toByteArray() : QByteArray();
    QByteArray currentHash = dialog ?
      dialog->property("contentHash").toByteArray() : QByteArray();
    const bool requirePixelChange = dialog &&
      dialog->property("testRequirePixelChange").toBool();
    const bool pixelsChanged = !initialHash.isEmpty() &&
      !currentHash.isEmpty() && initialHash != currentHash;
    const bool pixelChangePassed = pixelsChanged || !requirePixelChange;
    uint64_t initialRevision = dialog ?
      dialog->property("testInitialRevision").toULongLong() : 0;
    uint64_t currentRevision = dialog ?
      dialog->property("liveRevision").toULongLong() : 0;
    const bool staleClearPassed = dialog &&
      dialog->property("testStaleClearPassed").toBool();
    const bool refreshPassed = dialog && dialog->isVisible() &&
      dialog->property("imageAvailable").toBool() &&
      initialRevision + 1 < currentRevision &&
      0 < width && 0 < height;
    const bool toolFilterPassed = 0 <= tool &&
      disabledSimulationTools.count(tool) &&
      !initialHash.isEmpty() && !currentHash.isEmpty() && pixelChangePassed;
    const bool zoomStatePassed = dialog &&
      fabs(dialog->property("zoomScale").toDouble() -
           dialog->property("testZoomScale").toDouble()) < 1e-9;
    const bool scrollStatePassed = scroll &&
      scroll->horizontalScrollBar()->value() ==
        dialog->property("testHorizontalScroll").toInt() &&
      scroll->verticalScrollBar()->value() ==
        dialog->property("testVerticalScroll").toInt();
    const bool passed = staleClearPassed && refreshPassed &&
      toolFilterPassed && zoomStatePassed && scrollStatePassed;

    LOG_INFO(1, "GUI Dexel height-map live test: stale_clear="
             << (staleClearPassed ? "pass" : "fail")
             << " refresh=" << (refreshPassed ? "pass" : "fail")
             << " tool_filter=" << (toolFilterPassed ? "pass" : "fail")
             << " pixels_changed="
             << (pixelsChanged ? "pass" :
                 (requirePixelChange ? "fail" : "not-required"))
             << " zoom_state=" << (zoomStatePassed ? "pass" : "fail")
             << " scroll_state=" << (scrollStatePassed ? "pass" : "fail")
             << " result=" << (passed ? "pass" : "fail"));
    if (dialog) dialog->close();
    dexelGridWindowTestStage = 2;
    if (!passed) THROW("GUI Dexel height-map live test failed");
    return true;
  }

  if (dexelGridWindowTestStage)
    THROW("Invalid GUI Dexel height-map test stage");

  const QPixmap *initialPixmap = imageLabel ? imageLabel->pixmap() : 0;
  const bool initialPassed = ui->actionDexelHeightMap->isEnabled() &&
    dialog && dialog->isVisible() && imageLabel && initialPixmap && scroll &&
    zoomIn && zoomOut && actualSize && fitWindow && zoomLabel &&
    0 < width && 0 < height &&
    initialPixmap->width() == width && initialPixmap->height() == height &&
    fabs(dialog->property("zoomScale").toDouble() - 1) < 1e-9 &&
    zoomLabel->text() == "100%";

  if (zoomIn) zoomIn->trigger();
  const bool zoomInPassed = dialog && imageLabel &&
    1 < dialog->property("zoomScale").toDouble() &&
    width < imageLabel->width() && height < imageLabel->height();

  if (zoomOut) zoomOut->trigger();
  const bool zoomOutPassed = dialog && imageLabel &&
    fabs(dialog->property("zoomScale").toDouble() - 1) < 1e-9 &&
    imageLabel->width() == width && imageLabel->height() == height;

  if (zoomIn) zoomIn->trigger();
  if (actualSize) actualSize->trigger();
  const bool actualPassed = dialog && imageLabel &&
    fabs(dialog->property("zoomScale").toDouble() - 1) < 1e-9 &&
    imageLabel->width() == width && imageLabel->height() == height;

  if (fitWindow) fitWindow->trigger();
  double fitScale = dialog ? dialog->property("zoomScale").toDouble() : 0;
  const bool fitPassed = dialog && scroll && imageLabel && 0 < fitScale &&
    imageLabel->width() == max(1, qRound(width * fitScale)) &&
    imageLabel->height() == max(1, qRound(height * fitScale)) &&
    imageLabel->width() <= scroll->viewport()->width() &&
    imageLabel->height() <= scroll->viewport()->height();

  if (actualSize) actualSize->trigger();
  if (scroll) {
    QPointF local(scroll->viewport()->rect().center());
    QPointF global(scroll->viewport()->mapToGlobal(local.toPoint()));
    QWheelEvent wheel
      (local, global, QPoint(), QPoint(0, 120), Qt::NoButton,
       Qt::ControlModifier, Qt::ScrollUpdate, false,
       Qt::MouseEventSynthesizedBySystem);
    QCoreApplication::sendEvent(scroll->viewport(), &wheel);
  }
  const bool wheelPassed = dialog &&
    1 < dialog->property("zoomScale").toDouble() && imageLabel &&
    width < imageLabel->width() && height < imageLabel->height();
  const bool controlsPassed = initialPassed && zoomInPassed && zoomOutPassed &&
    actualPassed && fitPassed && wheelPassed;

  LOG_INFO(1, "GUI Dexel height-map window test: width=" << width
           << " height=" << height
           << " action="
           << (ui->actionDexelHeightMap->isEnabled() ? "enabled" :
               "disabled")
           << " window=" << (dialog && dialog->isVisible() ?
                               "visible" : "hidden")
           << " controls=" << (controlsPassed ? "pass" : "fail")
           << " zoom_in=" << (zoomInPassed ? "pass" : "fail")
           << " zoom_out=" << (zoomOutPassed ? "pass" : "fail")
           << " actual=" << (actualPassed ? "pass" : "fail")
           << " fit=" << (fitPassed ? "pass" : "fail")
           << " ctrl_wheel=" << (wheelPassed ? "pass" : "fail")
           << " result=" << (controlsPassed ? "pass" : "fail"));
  if (!controlsPassed) THROW("GUI Dexel height-map window test failed");

  QStandardItem *toolItem = toolTableModel->item(0);
  if (!dialog || !scroll || !toolItem)
    THROW("GUI Dexel height-map live test requires a tool");
  int tool = toolItem->data(Qt::UserRole).toInt();
  bool requirePixelChange = true;
  for (const GCode::Move &move: *toolPath)
    if ((int)move.getTool() != tool) {
      requirePixelChange = false;
      break;
    }
  dialog->setProperty("testDisabledTool", tool);
  dialog->setProperty("testRequirePixelChange", requirePixelChange);
  dialog->setProperty("testInitialHash", dialog->property("contentHash"));
  dialog->setProperty("testInitialRevision",
                      dialog->property("liveRevision"));
  dialog->setProperty("testZoomScale", dialog->property("zoomScale"));
  dialog->setProperty("testHorizontalScroll",
                      scroll->horizontalScrollBar()->value());
  dialog->setProperty("testVerticalScroll",
                      scroll->verticalScrollBar()->value());
  uint64_t staleClearCount =
    dialog->property("staleClearCount").toULongLong();
  dexelGridWindowTestStage = 1;
  setSimulationToolEnabled(tool, false);
  dialog->setProperty
    ("testStaleClearPassed",
     !dialog->property("imageAvailable").toBool() &&
     staleClearCount < dialog->property("staleClearCount").toULongLong());
  return false;
}


bool QtWin::seekNextSimulationTestPosition() {
  if (!options["simulation-seek-ratios"].hasValue()) return false;

  string values = options["simulation-seek-ratios"].toString();
  size_t begin = 0;
  for (unsigned i = 0; i < autoSeekIndex; i++) {
    begin = values.find(',', begin);
    if (begin == string::npos) return false;
    begin++;
  }

  if (values.size() <= begin) return false;
  size_t end = values.find(',', begin);
  string value = values.substr(begin, end - begin);
  double ratio = String::parseDouble(value);
  if (ratio <= 0 || 1 <= ratio)
    THROW("GUI test seek ratio must be between zero and one");

  autoSeekIndex++;
  LOG_INFO(1, "GUI simulation timeline seek: ratio=" << ratio);
  on_positionSlider_valueChanged((int)lround(ratio * 10000));
  return true;
}


bool QtWin::injectSimulationTestSeekBurst() {
  if (simulationSeekBurstInjected ||
      !options["simulation-seek-burst-ratios"].hasValue()) return false;

  simulationSeekBurstInjected = true;
  string values = options["simulation-seek-burst-ratios"].toString();
  size_t begin = 0;
  unsigned count = 0;
  while (begin < values.size()) {
    size_t end = values.find(',', begin);
    string value = values.substr(begin, end - begin);
    double ratio = String::parseDouble(value);
    if (ratio <= 0 || 1 <= ratio)
      THROW("GUI test burst ratio must be between zero and one");
    LOG_INFO(1, "GUI simulation burst seek: ratio=" << ratio);
    on_positionSlider_valueChanged((int)lround(ratio * 10000));
    count++;
    if (end == string::npos) break;
    begin = end + 1;
  }
  if (!count) THROW("GUI test burst requires at least one ratio");
  return true;
}


void QtWin::reduceComplete(ReduceTask &task) {
  surface = task.getSurface();
  view->setSurface(surface);
  ui->actionDexelHeightMap->setEnabled
    (dynamic_cast<Dexel::GridSurface *>(surface.get()) != 0);
  updateDexelHeightMapWindow();
  redraw();

  exportDialog.enableSurface(!surface.isNull());

  setStatusActive(false);
}


void QtWin::optimizeComplete(Opt &opt) {
  loadToolPath(opt.getPath(), true);
}


void QtWin::quit() {
  stop();
  app.requestExit();
  taskMan.join();
  QCoreApplication::exit();
}


void QtWin::stop() {
  taskMan.interrupt();
  setStatusActive(false);
}


void QtWin::reload(bool now) {
  if (!now) {
    simDirty = true;
    return;
  }

  simDirty = false;

  // Reset console
  ui->console->document()->clear();
  ui->console->setTextColor(QColor("#d9d9d9"));

  try {
    // Queue tool path task
    const GCode::PlannerConfig *config = 0;
    if (settingsDialog.getPlannerEnabled())
      config = &settingsDialog.getPlannerConfig();

    taskMan.addTask(new ToolPathTask(*project, config));
    setStatusActive(true);
  } CATCH_ERROR;
}


void QtWin::reduce() {
  if (surface.isNull()) return;

  try {
    // Queue reduce task
    taskMan.addTask(new ReduceTask(surface->copy()));
    setStatusActive(true);
  } CATCH_ERROR;
}


void QtWin::optimize() {
  if (toolPath.isNull()) return;

  try {
    // Queue optimize task
    taskMan.addTask(new Opt(*toolPath));
    setStatusActive(true);
  } CATCH_ERROR;
}


void QtWin::redraw(bool now) {
  if (now && 0.05 < Timer::now() - lastRedraw) {
    lastRedraw = Timer::now();

    updateWorkpieceBounds();
    ui->simulationView->update();

    dirty = false;
    return;
  }

  dirty = true;
}


void QtWin::snapshot() {
  QString filename = project->getFilename().c_str();
  QPixmap pixmap =
    QPixmap::fromImage(ui->simulationView->grabFramebuffer());

  QList<QByteArray> formats = QImageWriter::supportedImageFormats();
  QString fileTypes = tr("Image files") + QString(" (");
  for (int i = 0; i < formats.size(); i++) {
    if (i) fileTypes.append(",");
    fileTypes.append("*.");
    fileTypes.append(formats.at(i).data());
  }
  fileTypes.append(")");

  filename =
    SystemUtilities::swapExtension(filename.toStdString(), "png").c_str();
  filename = openFile(tr("Save snapshot"), fileTypes, filename, true);
  if (filename.isEmpty()) return;

  if (!pixmap.save(filename)) warning(tr("Failed to save snapshot."));
  else showMessage(tr("Snapshot saved."));
}








void QtWin::exportData() {
  // Check what we have to export
  if (surface.isNull() && gcode.empty() && simRun.isNull()) {
    warning(tr("Nothing to export.\nRun a simulation first."));
    return;
  }

  exportDialog.enableSurface(surface.isSet());
  exportDialog.enableGCode(!gcode.empty());
  exportDialog.enableSimData(simRun.isSet());

  // Run dialog
  if (exportDialog.exec() != QDialog::Accepted) return;

  // Select output file
  QString title = tr("Export ");
  QString fileTypes;
  string ext;

  if (exportDialog.surfaceSelected()) {
    title.append(tr("Surface"));
    fileTypes = tr("STL Files") + QString(" (*.stl)");
    ext = "stl";

  } else if (exportDialog.gcodeSelected()) {
    title.append(tr("GCode"));
    fileTypes = tr("GCode Files") + QString(" (*.gcode *.nc *.ngc *.tap)");
    ext = "gcode";

  } else {
    title.append(tr("Simulation Data"));
    fileTypes = tr("JSON Files") + QString(" (*.json)");
    ext = "json";
  }

  fileTypes += ";;All Files (*.*)";

  // Open output file
  QString filename =
    SystemUtilities::swapExtension(project->getFilename(), ext).c_str();
  filename = openFile(title, fileTypes, filename, true);

  if (filename.isEmpty()) return;
  SmartPointer<iostream> stream =
    SystemUtilities::open(filename.toStdString(), ios::out);

  // Export
  if (exportDialog.surfaceSelected()) {
    string hash = simRun->getSimulation().computeHash();
    bool binary = exportDialog.binarySTLSelected();
    surface->writeSTL(*stream, binary, "CAMotics Surface", hash);

  } else if (exportDialog.gcodeSelected()) {
    if (exportDialog.crlfSelected()) {
      for (unsigned i = 0; i < gcode.length(); i++) {
        if (gcode[i] == '\n') stream->put('\r');
        stream->put(gcode[i]);
      }

    } else *stream << gcode << flush;

  } else {
    JSON::Writer writer(*stream, 0, exportDialog.compactJSONSelected());
    Simulation sim = simRun->getSimulation();
    sim.surface = exportDialog.withCutSurfaceSelected() ? surface : 0;
    sim.write(writer);
    writer.close();
  }
}


bool QtWin::runNewProjectDialog() {
  // Initialize dialog
  newProjectDialog.setUnits(getDefaultUnits());

  // Run dialog
  return newProjectDialog.exec() == QDialog::Accepted;
}


GCode::ToolTable QtWin::getNewToolTable() {
  if (newProjectDialog.defaultToolTableSelected())
    return loadDefaultToolTable();

  if (newProjectDialog.currentToolTableSelected())
    return project.isNull() ? GCode::ToolTable() : project->getTools();

  return GCode::ToolTable();
}


GCode::Units QtWin::getNewUnits() {return newProjectDialog.getUnits();}


bool QtWin::runCAMDialog(const string &filename) {
  camDialog.loadDXFLayers(filename);
  camDialog.setUnits(getNewUnits());

  // Run dialog
  return camDialog.exec() == QDialog::Accepted;
}


QString QtWin::openFile(const QString &title, const QString &filters,
                       const QString &_filename, bool save, bool anyFile) {
  QString filename = _filename;
  if (filename.isEmpty() && !project.isNull())
    filename =
      QString(SystemUtilities::dirname(project->getFilename()).c_str());

  return fileDialog.open(title, filters, filename, save, anyFile);
}


void QtWin::loadProject() {
  // Close editor tabs
  ui->fileTabManager->closeAll(false, true);
  ui->fileTabManager->setCurrentIndex(0);

  // Reset view
  view->clear();
  redraw();

  // Free old sim
  surface.release();
  simRun.release();
  toolPath.release();
  ui->actionDexelHeightMap->setEnabled(false);
  clearDexelHeightMapWindow(tr("No simulation result is available."));
  simulationEndInjected = false;
  disabledSimulationTools.clear();

  reload();
  updateToolTables();
  updateFiles();
  updateUnits();

  // Open first file
  if (project.isSet() && project->getFileCount())
    ui->fileTabManager->open(project->getFile(0));
}


void QtWin::newProject() {
  if (!checkSave()) return;

  LOG_INFO(1, "New project");

  if (!runNewProjectDialog()) return;

  // Save tool table before resetting project
  GCode::ToolTable toolTable = getNewToolTable();
  GCode::Units units = getNewUnits();

  // Create new project
  project = new Project::Project;
  project->setUnits(units);
  project->getTools() = toolTable;

  loadProject();
  project->markClean();
}


void QtWin::openProject(const string &_filename) {
  if (!checkSave()) return;

  string filename = _filename;

  if (filename.empty()) {
    QSettings settings;
    QString lastDir =
      settings.value("Projects/lastDir", QDir::homePath()).toString();

    filename = QFileDialog::getOpenFileName
      (this, tr("Open File"), lastDir,
       (tr("Supported Files") +
        QString("(*.camotics *.xml *.nc *.ngc *.gcode *.tap *.tpl *.dxf);;") +
        tr("All Files") + QString(" (*.*)"))).toUtf8().data();

    if (filename.empty()) return;
    settings.setValue("Projects/lastDir", QString::fromUtf8(filename.c_str()));
  }

  updateRecentProjects(filename);
  showMessage(tr("Opening %1").arg(filename.c_str()));

  try {
    // Check for project file
    string ext = SystemUtilities::extension(filename);
    bool is_project = ext == "camotics" || ext == "xml";

    if (!is_project) {
      string projectPath;

      // Check if .camotics files exists
      string path = SystemUtilities::swapExtension(filename, "camotics");
      if (SystemUtilities::exists(path)) projectPath = path;
      else {
        string path = SystemUtilities::swapExtension(filename, "xml");
        if (SystemUtilities::exists(path)) projectPath = path;
      }

      if (!projectPath.empty()) {
        int response =
          QMessageBox::question
          (this, "Project File Exists", "A CAMotics project file for the "
           "selected file exists.  It may contain important project settings.  "
           "Would you like to open it instead?",
           QMessageBox::Cancel  | QMessageBox::No | QMessageBox::Yes,
           QMessageBox::Yes);

        if (response == QMessageBox::Cancel) return;
        if (response == QMessageBox::Yes) {
          is_project = true;
          filename = projectPath;
        }
      }
    }

    if (is_project) project = new Project::Project(filename);
    else {
      // Otherwise, create a new project with the file
      if (!runNewProjectDialog()) return;

      if (String::toLower(SystemUtilities::extension(filename)) == "dxf") {
        THROW("DXF support not yet implemented");
        if (!runCAMDialog(filename)) return;
        // TODO handle CAM JSON
      }

      // Save tool table before resetting project
      GCode::ToolTable toolTable = getNewToolTable();
      GCode::Units units = getNewUnits();

      project = new Project::Project;
      project->addFile(filename);
      project->setUnits(units);
      project->getTools() = toolTable;
    }

    return loadProject();
  } CATCH_ERROR;

  warning("Failed to open project.  See console for errors.");
}


bool QtWin::saveProject(bool saveAs) {
  QString filename = project->getFilename().c_str();
  string ext = SystemUtilities::extension(filename.toStdString());

  if (saveAs || filename.isEmpty() || ext != "camotics" ||
      !project->isOnDisk()) {
    if (filename.isEmpty()) filename = "project.camotics";
    else filename = SystemUtilities::swapExtension(filename.toStdString(),
                                                   "camotics").c_str();

    filename = openFile(tr("Save Project"),
                        tr("Projects") + QString(" (*.camotics)"),
                        filename, true);
    if (filename.isEmpty()) return false;

    string ext = SystemUtilities::extension(filename.toStdString());
    if (ext.empty()) filename.append(".camotics");
    else if (ext != "camotics") {
      warning(tr("Project file must have .camotics extension, not saved!"));
      return false;
    }
  }

  try {
    project->save(filename.toStdString());
    ui->fileTabManager->saveAll();
    showMessage(tr("Saved %1").arg(filename));
    return true;

  } catch (const Exception &e) {
    warning(tr("Could not save project: ").append(e.getMessage().c_str()));
  }

  return false;
}


void QtWin::revertProject() {
  string filename = project->getFilename();

  if (filename.empty()) {
    warning(tr("Cannot revert project."));
    return;
  }

  project->markClean();
  openProject(filename);
  ui->fileTabManager->revertAll();
}


bool QtWin::isMetric() const {return project.isNull() || project->isMetric();}


GCode::Units QtWin::getDefaultUnits() const {
  return (GCode::Units::enum_t)QSettings().value("Settings/Units").toInt();
}


void QtWin::updateFiles() {
  QStringList list;

  for (unsigned i = 0; i < project->getFileCount(); i++)
    list.append(QString::fromUtf8(project->getFileRelativePath(i).c_str()));

  ui->filesListView->setModel(new QStringListModel(list));
}


void QtWin::newFile(bool tpl) {
  QString filename = project->getFilename().c_str();
  if (filename.isEmpty()) filename = "newfile";
  filename = SystemUtilities::swapExtension
    (filename.toStdString(), tpl ? "tpl" : "nc").c_str();

  filename = openFile(tpl ? tr("New TPL file") : tr("New GCode file"),
                      tpl ? QString("TPL (*.tpl);;") + tr("All files") +
                      QString(" (*.*)") :
                      (QString("GCode (*.nc *.ngc *.gcode *.tap);;") +
                       tr("All files") + QString(" (*.*)")), filename, false,
                      true);
  if (filename.isEmpty()) return;

  string ext = SystemUtilities::extension(filename.toStdString());
  if (ext.empty()) filename.append(tpl ? ".tpl" : ".gcode");

  else if (tpl && ext != "tpl") {
    warning(tr("TPL file must have .tpl extension"));
    return;

  } else if (!tpl && (ext == "camotics" || ext == "xml" || ext == "tpl")) {
    warning(tr("GCode file cannot have .tpl, .camotics or .xml extension"));
    return;
  }

  project->addFile(filename.toStdString());
  updateFiles();

  // Reset view
  view->clear();
  redraw();

  // Free old sim
  surface.release();
  simRun.release();
  ui->actionDexelHeightMap->setEnabled(false);
  clearDexelHeightMapWindow(tr("No simulation result is available."));

  reload();
}


void QtWin::addFile() {
  QString filename =
    openFile(tr("Add file"), tr("Supported Files") +
             QString(" (*.dxf, *.nc *.ngc *.gcode *.tap *.tpl);;") +
             tr("All Files") + QString(" (*.*)"), "", false);
  if (filename.isEmpty()) return;

  if (SystemUtilities::extension(filename.toLower().toStdString()) == "dxf") {
    THROW("DXF supported not yet implemented");
     if (!runCAMDialog(filename.toStdString())) return;
    // TODO handle CAM JSON
  }

  project->addFile(filename.toStdString());
  updateFiles();
}


void QtWin::editFile(unsigned index) {
  SmartPointer<Project::File> file = project->getFile(index);

  if (file.isSet()) {
    ui->fileTabManager->open(file);
    if (!ui->splitterFile->sizes()[1]) showText();
  }
}


void QtWin::removeFile(unsigned index) {
  // Remove text from editor
  ui->fileTabManager->close(project->getFile(index), false, false);

  // Remove file from project
  project->removeFile(index);
  updateFiles();

  // Reset view
  view->clear();
  redraw();

  // Free old sim
  surface.release();
  simRun.release();
  ui->actionDexelHeightMap->setEnabled(false);
  clearDexelHeightMapWindow(tr("No simulation result is available."));

  reload();
}


bool QtWin::checkSave(bool canCancel) {
  if (!ui->fileTabManager->checkSaveAll()) return false;
  if (project.isNull() || !project->isDirty()) return true;

  int response =
    QMessageBox::question(this, "Project Modified", "The current project has "
                          "been modified.  Would you like to save it?",
                          (canCancel ?
                           QMessageBox::Cancel : QMessageBox::NoButton) |
                          QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);

  if (response == QMessageBox::Yes) return saveProject();
  return response == QMessageBox::No;
}


void QtWin::activateFile(const string &filename, int line, int col) {
  SmartPointer<Project::File> file = project->findFile(filename);
  if (!file.isNull()) ui->fileTabManager->open(file, line, col);
}


void QtWin::updateActions() {
  int tab = ui->fileTabManager->currentIndex();
  bool fileTab = tab != -1;

  if (!fileTab) {
    ui->actionSaveFile->setEnabled(false);
    ui->actionSaveFileAs->setEnabled(false);
    ui->actionRevertFile->setEnabled(false);

    ui->actionSaveFile->setText("Save File");
    ui->actionSaveFileAs->setText("Save File As");
    ui->actionRevertFile->setText("Revert File");

  } else {
    SmartPointer<Project::File> file = ui->fileTabManager->getFile(tab);
    QString title = QString::fromUtf8(file->getBasename().c_str());

    bool modified = ui->fileTabManager->isModified(tab);
    bool exists = file->exists();

    ui->actionSaveFile->setEnabled(modified);
    ui->actionSaveFileAs->setEnabled(true);
    ui->actionRevertFile->setEnabled(modified && exists);

    ui->actionSaveFile->setText(QString("Save \"%1\"").arg(title));
    ui->actionSaveFileAs->setText(QString("Save \"%1\" As").arg(title));
    ui->actionRevertFile->setText(QString("Revert \"%1\"").arg(title));
  }

  ui->actionUndo->setEnabled(fileTab);
  ui->actionRedo->setEnabled(fileTab);
  ui->actionCut->setEnabled(fileTab);
  ui->actionCopy->setEnabled(fileTab);
  ui->actionPaste->setEnabled(fileTab);
  ui->actionSelectAll->setEnabled(fileTab);
}


void QtWin::updateUnits() {
  loadWorkpiece();
  updateBounds();
  valueSet.updated();
}


void QtWin::updateToolTables() {
  GCode::ToolTable tools;
  if (!project.isNull()) tools = project->getTools();

  // Prefer current project definitions, but retain any definitions captured in
  // the parsed path and include every tool number that the moves actually use.
  // A missing definition must remain visible so it can be fixed in the editor.
  if (!toolPath.isNull())
    for (const auto &entry: toolPath->getTools())
      if (!tools.has(entry.first)) tools.set(entry.second);

  std::set<unsigned> toolNumbers;
  for (const auto &entry: tools) toolNumbers.insert(entry.first);
  if (!toolPath.isNull())
    for (const GCode::Move &move: *toolPath)
      if (0 <= move.getTool()) toolNumbers.insert((unsigned)move.getTool());

  toolTableModel->clear();
  for (unsigned number: toolNumbers) {
      const bool defined = tools.has(number);
      QString label = defined ?
        QString().sprintf("%d: %s", number,
                          tools.get(number).getText().c_str()) :
        tr("%1: used by toolpath; definition missing").arg(number);
      QStandardItem *item = new QStandardItem(label);
      const bool enabled = !disabledSimulationTools.count(number);
      item->setCheckable(true);
      item->setCheckState(enabled ? Qt::Checked : Qt::Unchecked);
      item->setData((uint)number, Qt::UserRole);
      item->setData(defined, Qt::UserRole + 1);
      item->setToolTip
        (tr("Uncheck to exclude this tool's moves from stock removal. "
            "The original toolpath and cutter motion remain visible.%1")
          .arg(defined ? QString() :
               tr(" Double-click to define this tool.")));
      if (!enabled) item->setForeground(QBrush(QColor("#888888")));
      else if (!defined) item->setForeground(QBrush(QColor("#b36b00")));
      toolTableModel->appendRow(item);
  }

  LOG_INFO(1, "GUI tool table: project_tools="
           << (project.isNull() ? 0 : project->getTools().size())
           << " path_tools="
           << (toolPath.isNull() ? 0 : toolPath->getTools().size())
           << " rows=" << toolTableModel->rowCount());
}


void QtWin::toolsChanged() {
  project->markDirty();
  updateToolTables();
}


void QtWin::editTool(unsigned number) {
  if (project.isNull()) return;

  GCode::ToolTable &tools = project->getTools();

  if (tools.has(number)) toolDialog.setTool(tools.get(number));
  else toolDialog.getTool().setNumber(number);

  if (toolDialog.edit() != QDialog::Accepted) return;

  GCode::Tool &tool = toolDialog.getTool();

  if (tool.getNumber() != number) {
    if (tools.has(tool.getNumber())) {
      int response = QMessageBox::question
        (this, "Overwrite Tool?",
         QString().sprintf("Tool %d already exists.  Do you want to overwrite "
                           "it?", tool.getNumber()),
         QMessageBox::No | QMessageBox::Yes, QMessageBox::No);

      if (response == QMessageBox::No) return;
    }

    tools.erase(number);
  }

  tools.set(tool);
  toolsChanged();
}


void QtWin::addTool() {
  if (project.isNull()) return;
  GCode::ToolTable &tools = project->getTools();

  for (unsigned i = 1; i < 1000; i++)
    if (!tools.has(i)) {
      editTool(i);
      return;
    }

  THROW("Too many tools");
}


void QtWin::removeTool(unsigned number) {
  project->getTools().erase(number);
  toolsChanged();
}


void QtWin::exportToolTable() {
  if (project.isNull()) return;
  const GCode::ToolTable &tools = project->getTools();

  QString filename =
    (SystemUtilities::dirname(project->getFilename()) + "/tools.json").c_str();

  filename = openFile(tr("Export tool table"), tr("Tool table files") +
                      QString(" (*.json)"), filename, true);

  if (filename.isEmpty()) return;

  *SystemUtilities::oopen(filename.toStdString()) << tools;
}


void QtWin::importToolTable() {
  if (project.isNull()) return;

  QString filename = openFile
    (tr("Import tool table"), tr("Tool table files") + QString(" (*.json)"),
     "", false);

  if (filename.isEmpty()) return;

  GCode::ToolTable tools;
  *SystemUtilities::iopen(filename.toStdString()) >> tools;

  if (tools.empty()) {
    warning(tr("'%1' empty or not a tool table").arg(filename));
    return;
  }

  project->getTools() = tools;
  toolsChanged();
}


void QtWin::saveDefaultToolTable(const GCode::ToolTable &tools) {
  ostringstream str;
  str << tools << flush;

  QSettings settings;
  settings.setValue("ToolTable/Default", QString::fromUtf8(str.str().c_str()));

  showMessage(tr("Default tool table saved"));
}


GCode::ToolTable QtWin::loadDefaultToolTable() {
  QSettings settings;
  QByteArray data = settings.value("ToolTable/Default").toString().toUtf8();

  GCode::ToolTable tools;

  if (!data.isEmpty()) {
    istringstream str(data.data());
    str >> tools;
  }

  return tools;
}


void QtWin::updateWorkpiece() {
  view->setWorkpiece(project->getWorkpiece().getBounds());
  redraw();
}


void QtWin::loadWorkpiece() {
  const Project::Workpiece &workpiece = project->getWorkpiece();

  if (workpiece.isAutomatic()) on_automaticCuboidRadioButton_clicked();
  else on_manualCuboidRadioButton_clicked();

  LOCK_UI_UPDATES;
  ui->marginDoubleSpinBox->setValue(workpiece.getMargin());

  // Bounds
  double scale = isMetric() ? 1 : 1 / 25.4;
  Rectangle3D bounds = workpiece.getBounds();
  ui->xDimDoubleSpinBox->setValue(bounds.getDimensions().x() * scale);
  ui->yDimDoubleSpinBox->setValue(bounds.getDimensions().y() * scale);
  ui->zDimDoubleSpinBox->setValue(bounds.getDimensions().z() * scale);
  ui->xOffsetDoubleSpinBox->setValue(bounds.getMin().x() * scale);
  ui->yOffsetDoubleSpinBox->setValue(bounds.getMin().y() * scale);
  ui->zOffsetDoubleSpinBox->setValue(bounds.getMin().z() * scale);

  // GCode::Units
  const char *suffix = isMetric() ? "mm" : "in";
  ui->xDimDoubleSpinBox->setSuffix(suffix);
  ui->yDimDoubleSpinBox->setSuffix(suffix);
  ui->zDimDoubleSpinBox->setSuffix(suffix);
  ui->xOffsetDoubleSpinBox->setSuffix(suffix);
  ui->yOffsetDoubleSpinBox->setSuffix(suffix);
  ui->zOffsetDoubleSpinBox->setSuffix(suffix);

  // Update Workpiece steps
  double step = isMetric() ? 1 : 0.125;
  ui->xDimDoubleSpinBox->setSingleStep(step);
  ui->yDimDoubleSpinBox->setSingleStep(step);
  ui->zDimDoubleSpinBox->setSingleStep(step);
  ui->xOffsetDoubleSpinBox->setSingleStep(step);
  ui->yOffsetDoubleSpinBox->setSingleStep(step);
  ui->zOffsetDoubleSpinBox->setSingleStep(step);

  updateWorkpiece();
}


void QtWin::setWorkpieceDim(unsigned dim, double value) {
  double scale = isMetric() ? 1 : 25.4;
  Rectangle3D bounds = project->getWorkpiece().getBounds();
  bounds.rmax[dim] = bounds.rmin[dim] + value * scale;
  project->getWorkpiece().setBounds(bounds);

  updateWorkpiece();
  redraw(true);
}


void QtWin::setWorkpieceOffset(unsigned dim, double value) {
  double scale = isMetric() ? 1 : 25.4;
  Rectangle3D bounds = project->getWorkpiece().getBounds();
  bounds.rmax[dim] = bounds.getDimension(dim) + value * scale;
  bounds.rmin[dim] = value * scale;
  project->getWorkpiece().setBounds(bounds);

  updateWorkpiece();
  redraw(true);
}


void QtWin::updateBounds() {
  updateToolPathBounds();
  updateWorkpieceBounds();
}


void QtWin::updateToolPathBounds() {
  if (toolPath.isNull()) return;
  Rectangle3D bounds = *toolPath;
  Vector3D bMin = bounds.getMin();
  Vector3D bMax = bounds.getMax();
  Vector3D bDim = bounds.getDimensions();

  setUnitLabel(ui->toolPathBoundsXMinLabel, bMin.x());
  setUnitLabel(ui->toolPathBoundsXMaxLabel, bMax.x());
  setUnitLabel(ui->toolPathBoundsXDimLabel, bDim.x());

  setUnitLabel(ui->toolPathBoundsYMinLabel, bMin.y());
  setUnitLabel(ui->toolPathBoundsYMaxLabel, bMax.y());
  setUnitLabel(ui->toolPathBoundsYDimLabel, bDim.y());

  setUnitLabel(ui->toolPathBoundsZMinLabel, bMin.z());
  setUnitLabel(ui->toolPathBoundsZMaxLabel, bMax.z());
  setUnitLabel(ui->toolPathBoundsZDimLabel, bDim.z());
}


void QtWin::updateWorkpieceBounds() {
  if (project.isNull()) return;

  Rectangle3D bounds = project->getWorkpiece().getBounds();
  Vector3D bMin = bounds.getMin();
  Vector3D bMax = bounds.getMax();
  Vector3D bDim = bounds.getDimensions();

  setUnitLabel(ui->workpieceBoundsXMinLabel, bMin.x());
  setUnitLabel(ui->workpieceBoundsXMaxLabel, bMax.x());
  setUnitLabel(ui->workpieceBoundsXDimLabel, bDim.x());

  setUnitLabel(ui->workpieceBoundsYMinLabel, bMin.y());
  setUnitLabel(ui->workpieceBoundsYMaxLabel, bMax.y());
  setUnitLabel(ui->workpieceBoundsYDimLabel, bDim.y());

  setUnitLabel(ui->workpieceBoundsZMinLabel, bMin.z());
  setUnitLabel(ui->workpieceBoundsZMaxLabel, bMax.z());
  setUnitLabel(ui->workpieceBoundsZDimLabel, bDim.z());
}


void QtWin::updateStatusDisplay(bool active) {
  const bool playing = view->isFlagSet(View::PLAY_FLAG);
  double ratio = numeric_limits<double>::quiet_NaN();
  int moveIndex = -1;
  uint64_t moveCount = 0;
  if (view->path.isSet() && !view->path->isEmpty()) {
    ratio = view->path->getTimeRatio();
    SmartPointer<const GCode::ToolPath> path = view->path->getPath();
    if (path.isSet()) {
      moveCount = path->size();
      moveIndex = path->find(view->path->getTime());
      if (moveIndex < 0 && view->path->atEnd() && moveCount)
        moveIndex = moveCount - 1;
    }
  }

  QString state;
  if (playing) state = tr("PLAY %1x").arg(view->getSpeed());
  else if (active) state = tr("SIMULATING");
  else if (simulationBackendResolved && isfinite(ratio) && ratio < 1)
    state = tr("PAUSED");
  else state = simulationBackendResolved ? tr("READY") : tr("IDLE");

  QString backend;
  if (!simulationBackendResolved)
    backend = simulationBackendPolicy == SimulationBackendPolicy::AUTO_DEXEL ?
      tr("Auto") : tr("Full MC");
  else if (lastSimulationBackend == SimulationBackend::DEXEL)
    backend = tr("Dexel");
  else if (simulationBackendFallback)
    backend = tr("Full MC fallback");
  else backend = tr("Full MC");

  QString text = tr("%1  |  %2").arg(state).arg(backend);
  if (isfinite(ratio)) {
    text += tr("  |  %1%").arg(100 * ratio, 0, 'f', 2);
    if (moveIndex >= 0 && moveCount)
      text += tr("  |  move %1/%2")
        .arg(QLocale().toString((qulonglong)moveIndex + 1))
        .arg(QLocale().toString((qulonglong)moveCount));
  }
  text += tr("  |  updates %1  |  dropped %2  |  %3")
    .arg(simulationCompleted).arg(simulationDiscarded)
    .arg(tr(view->getReferenceFrameName()));
  if (!disabledSimulationTools.empty())
    text += tr("  |  tools off %1")
      .arg(QLocale().toString
           ((qulonglong)disabledSimulationTools.size()));
  if (machineConstraintWarnings)
    text += tr("  |  machine warnings %1")
      .arg(QLocale().toString
           ((qulonglong)machineConstraintWarnings));

  statusLabel->setText(text);
  QString toolTip =
    (simulationBackendPolicy == SimulationBackendPolicy::AUTO_DEXEL ?
     tr("Safe Auto uses fast Dexel when eligible and Full MC as fallback.") :
     tr("Full marching cubes reference selected; playback stock updates may "
        "be slow."));
  if (!machineConstraintSummary.isEmpty())
    toolTip += "\n" + machineConstraintSummary;
  statusLabel->setToolTip(toolTip);

  ui->actionStop->setEnabled(active);
}


void QtWin::setStatusActive(bool active) {
  if (active == lastStatusActive) return;
  lastStatusActive = active;

  // A retained playback worker starts and completes on nearly every display
  // frame.  Keep one continuous running indicator for the Play session so the
  // idle pixmap and Stop action do not blink or relayout the Windows view.
  updateStatusDisplay(active || view->isFlagSet(View::PLAY_FLAG));
}


void QtWin::showConsole() {
  ui->splitter->setSizes(QList<int>() << 5 << 1);
  ui->actionShowConsole->setChecked(true);
  ui->actionHideConsole->setChecked(false);
}


void QtWin::hideConsole() {
  ui->splitter->setSizes(QList<int>() << 1 << 0);
  ui->actionHideConsole->setChecked(true);
  ui->actionShowConsole->setChecked(false);
}


void QtWin::showText() {
  QList<int> currentSizes = ui->splitterFile->sizes();

  if (!currentSizes[1]) {
    currentSizes[0] -= 400;
    currentSizes[1] += 400;
    ui->splitterFile->setSizes(currentSizes);
  }
}


void QtWin::hideText() {
  ui->splitterFile->setSizes(QList<int>() << 1 << 0);
}


void QtWin::updatePlaySpeed(const string &name, unsigned value) {
  showMessage(tr("Playback speed %1x").arg(view->getSpeed()), false);
  updateStatusDisplay(lastStatusActive ||
                      view->isFlagSet(View::PLAY_FLAG));
}


void QtWin::updateViewFlags(const string &name, unsigned flags) {
  ui->actionPlay->setIcon(flags & View::PLAY_FLAG ? pauseIcon : playIcon);
  ui->actionPlay->setText(flags & View::PLAY_FLAG ? "Pause" : "Play");
  updateStatusDisplay(lastStatusActive || (flags & View::PLAY_FLAG));
}


void QtWin::updatePlayDirection(const string &name, bool reverse) {
  ui->actionDirection->setIcon(reverse ? backwardIcon : forwardIcon);
}


void QtWin::updateTimeRatio(const string &name, double ratio) {
  if (isnan(ratio)) return;

  // During Play, the slider is only a coarse display of the exact playback
  // clock.  Letting its integer valueChanged signal feed back into the path
  // quantizes long programs to totalTime / 10000 and makes stock updates jump
  // by seconds at normal speed.  animate() queues exact-time stock updates.
  if (view->isFlagSet(View::PLAY_FLAG)) {
    QSignalBlocker blocker(ui->positionSlider);
    ui->positionSlider->setValue(10000 * ratio);
  } else ui->positionSlider->setValue(10000 * ratio);
  updateStatusDisplay(lastStatusActive ||
                      view->isFlagSet(View::PLAY_FLAG));
}


void QtWin::updateX(const string &name, double value) {
  setUnitLabel(ui->xLabel, value, isMetric() ? 4 : 5, true);
}


void QtWin::updateY(const string &name, double value) {
  setUnitLabel(ui->yLabel, value, isMetric() ? 4 : 5, true);
}


void QtWin::updateZ(const string &name, double value) {
  setUnitLabel(ui->zLabel, value, isMetric() ? 4 : 5, true);
}


void QtWin::updateCurrentTime(const string &name, double value) {
  QString text =
    QString::fromUtf8(TimeInterval(value, true).toString().c_str());
  ui->currentTimeLabel->setText(text);
}


void QtWin::updateCurrentDistance(const string &name, double value) {
  setUnitLabel(ui->currentDistanceLabel, value);
}


void QtWin::updateRemainingTime(const string &name, double value) {
  QString text =
    QString::fromUtf8(TimeInterval(value, true).toString().c_str());
  ui->remainingTimeLabel->setText(text);
}


void QtWin::updateRemainingDistance(const string &name, double value) {
  setUnitLabel(ui->remainingDistanceLabel, value);
}


void QtWin::updateTotalTime(const string &name, double value) {
  QString text =
    QString::fromUtf8(TimeInterval(value, true).toString().c_str());
  ui->totalTimeLabel->setText(text);
}


void QtWin::updateTotalDistance(const string &name, double value) {
  setUnitLabel(ui->totalDistanceLabel, value);
}


void QtWin::updatePercentTime(const string &name, double value) {
  ui->percentTimeLabel->setText(QString().sprintf("%.2f%%", value));
}


void QtWin::updatePercentDistance(const string &name, double value) {
  ui->percentDistanceLabel->setText(QString().sprintf("%.2f%%", value));
}


void QtWin::updateTool(const string &name, unsigned value) {
  ui->toolLabel->setText(QString().sprintf("%d", value));
}


void QtWin::updateFeed(const string &name, double value) {
  double scale = isMetric() ? 1.0 : 1.0 / 25.4;
  ui->feedLabel->setText(QString().sprintf("%.2f", value * scale));
}


void QtWin::updateSpeed(const string &name, double value) {
  if (numeric_limits<double>::max() == abs(value) || isinf(value) ||
      isnan(value)) ui->speedLabel->setText("nan");
  else ui->speedLabel->setText(QString().sprintf("%.2f RPM", value));
}


void QtWin::updateDirection(const string &name, const char *value) {
  ui->directionLabel->setText(QString::fromUtf8(value));
}


void QtWin::updateProgramLine(const string &name, unsigned value) {
  if (programLine == value) return;
  programLine = value;

  // Open and/or select file tab and highlight current line
  if (0 < ui->splitterFile->sizes()[1] && !programFile.empty())
    activateFile(programFile, value);

  ui->programLineLabel->setText(QString().sprintf("%d", value));
}


void QtWin::updateProgramFile(const string &name, const char *value) {
  if (programFile == value) return;
  programFile = value;

  ui->programNameLabel->
    setText(QString::fromStdString(SystemUtilities::basename(value)));
}


void QtWin::updateNumericLabel(QLabel *label, double value) {
  double scale = isMetric() ? 1.0 : 1.0 / 25.4;
  const char *prec = isMetric() ? "%.2f" : "%.3f";

  label->setText(QString().sprintf(prec, value * scale));
}


void QtWin::updateMoveStartX(const string &name, double value) {
  updateNumericLabel(ui->xStartMoveLabel, value);
}


void QtWin::updateMoveStartY(const string &name, double value) {
  updateNumericLabel(ui->yStartMoveLabel, value);
}


void QtWin::updateMoveStartZ(const string &name, double value) {
  updateNumericLabel(ui->zStartMoveLabel, value);
}


void QtWin::updateMoveEndX(const string &name, double value) {
  updateNumericLabel(ui->xEndMoveLabel, value);
}


void QtWin::updateMoveEndY(const string &name, double value) {
  updateNumericLabel(ui->yEndMoveLabel, value);
}


void QtWin::updateMoveEndZ(const string &name, double value) {
  updateNumericLabel(ui->zEndMoveLabel, value);
}


void QtWin::updateMoveDistanceX(const string &name, double value) {
  updateNumericLabel(ui->xMoveDistanceLabel, value);
}


void QtWin::updateMoveDistanceY(const string &name, double value) {
  updateNumericLabel(ui->yMoveDistanceLabel, value);
}


void QtWin::updateMoveDistanceZ(const string &name, double value) {
  updateNumericLabel(ui->zMoveDistanceLabel, value);
}


void QtWin::updateMoveDistanceTotal(const string &name, double value) {
  updateNumericLabel(ui->moveTotalDistanceLabel, value);
}


void QtWin::taskCompleted() {
  QCoreApplication::postEvent
    (this, new QEvent((QEvent::Type)taskCompleteEvent));
}


bool QtWin::event(QEvent *event) {
  if (event->type() != taskCompleteEvent) return QMainWindow::event(event);

  while (taskMan.hasMore()) {
    SmartPointer<Task> task = taskMan.remove();

    if (task.isInstance<ToolPathTask>())
      toolPathComplete(*task.cast<ToolPathTask>());
    else if (task.isInstance<SurfaceTask>())
      surfaceComplete(*task.cast<SurfaceTask>());
    else if (task.isInstance<ReduceTask>())
      reduceComplete(*task.cast<ReduceTask>());
    else if (task.isInstance<Opt>())
      optimizeComplete(*task.cast<Opt>());
  }

  return true;
}


void QtWin::closeEvent(QCloseEvent *event) {
  if (checkSave()) {
    quit();
    event->accept();
  } else event->ignore();
}


void QtWin::resizeEvent(QResizeEvent *event) {
  updateToolTables();
  QMainWindow::resizeEvent(event);
}


void QtWin::changeEvent(QEvent *event) {
  if (!event) return;

  switch (event->type()) {
  case QEvent::LanguageChange: // New translator loaded
    ui->retranslateUi(this);
    break;

  case QEvent::LocaleChange: { // System language changed
    QString locale = QLocale::system().name();
    locale.truncate(locale.lastIndexOf('_'));
    loadLanguage(locale);
    break;
  }

  default: break;
  }

  QMainWindow::changeEvent(event);
}


void QtWin::animate() {
  try {
    // Check if OpenGL is initialized
    if (!ui->simulationView->isEnabled()) {
      QMessageBox::critical
        (this, "OpenGL Error", "Failed to load OpenGL 3D graphics!\n\n"
         "CAMotics requires OpenGL ES 2.0 or newer.\n"
         "You may need to upgrade your graphics driver.", QMessageBox::Ok);
      app.requestExit();
    }

    // Do not let Play run invisibly while its initial retained state is still
    // being prepared.  The uncut workpiece placeholder remains at time zero.
    const bool wasPlaying = view->isFlagSet(View::PLAY_FLAG);
    const bool playbackReady = !wasPlaying || !simRun.isNull();
    const bool playbackAdvanced = playbackReady && view->update();
    dirty = playbackAdvanced || dirty;

    // Queue the exact playback time independently of the integer position
    // slider.  The scheduler below still bounds retained-state work to 10 Hz
    // and coalesces animation frames while a worker is active.
    if (wasPlaying && playbackAdvanced && !simRun.isNull()) {
      const double ratio = view->path->getRequestedTimeRatio();
      const double target = isnan(ratio) ? 0 :
        ratio * view->path->getTotalTime();
      const double tolerance = max(1.0, fabs(target)) * 1e-12;
      if (!positionChanged ||
          tolerance < fabs(pendingSimulationTime - target)) {
        simulationRequests++;
        if (positionChanged) simulationCoalesced++;
        pendingSimulationTime = target;
        positionChanged = true;
      }
    }

    // Auto close after auto play
    if (!autoPlay && autoClose && !view->isFlagSet(View::PLAY_FLAG) &&
        !positionChanged && !isActive())
      app.requestExit();

    if (dirty) redraw(true);
    if (simDirty) reload(true);

    const double now = Timer::now();
    const bool playing = view->isFlagSet(View::PLAY_FLAG);
    const bool cadenceReady = !playing || !lastSimulationSchedule ||
      0.1 <= now - lastSimulationSchedule;
    if (!app.shouldQuit() && !simRun.isNull() && positionChanged &&
        !isActive() && cadenceReady &&
        view->isFlagSet(View::SHOW_SURFACE_FLAG)) {
      positionChanged = false;
      lastSimulationSchedule = now;
      const uint64_t generation = ++simulationGeneration;
      setStatusActive(true);
      LOG_INFO(1, "GUI simulation scheduled: generation=" << generation
               << " target=" << pendingSimulationTime
               << " requests=" << simulationRequests
               << " coalesced=" << simulationCoalesced);
      taskMan.addTask
        (new SurfaceTask
         (simRun, pendingSimulationTime, generation,
          !playing || view->getSpeed() <= 8));
    }

    // Update progress
    if (!view->isFlagSet(View::PLAY_FLAG)) {
      double progress = taskMan.getProgress();
      string status = taskMan.getStatus();
      if (lastProgress != progress || lastStatus != status) {
        lastProgress = progress;
        lastStatus = status;

        if (progress) {
          double eta = taskMan.getETA();
          ui->progressBar->setValue(10000 * progress);
          string s = status + String::printf(" %.2f%% ", progress * 100);
          if (eta) s += "ETA " + TimeInterval(eta).toString();
          ui->progressBar->setFormat(QString::fromUtf8(s.c_str()));

          showMessage(s.c_str(), false);

        } else {
          ui->progressBar->setValue(0);
          ui->progressBar->setFormat(QString::fromUtf8(status.c_str()));
        }
      }
    }

    // Copy log
    ui->console->writeToConsole();

  } CBANG_CATCH_ERROR;

  bool modified = !project.isNull() && project->isDirty();
  if (isWindowModified() != modified) setWindowModified(modified);

  if (app.shouldQuit()) {
    checkSave(false);
    quit();
  }
}


void QtWin::openRecentProjectsSlot(const QString path) {
  openProject(path.toUtf8().data());
}


void QtWin::on_openProject(QString path) {openProject(path.toStdString());}


void QtWin::on_bbctrlConnect() {
  if (!bbCtrlAPI) {
    bbCtrlAPI = new BBCtrlAPI(this);
    connect(bbCtrlAPI.get(), SIGNAL(connected()), this,
            SLOT(on_bbctrlConnected()));
    connect(bbCtrlAPI.get(), SIGNAL(disconnected()), this,
            SLOT(on_bbctrlDisconnected()));
  }

  bbCtrlAPI->setUseSystemProxy(connectDialog.isSystemProxyEnabled());
  bbCtrlAPI->connectCNC(connectDialog.getAddress());
}


void QtWin::on_bbctrlDisconnect() {
  bbCtrlAPI->disconnectCNC();
  bbCtrlAPI->setFilename("");
}


void QtWin::on_bbctrlConnected() {
  connectDialog.setNetworkStatus(bbCtrlAPI->getStatus());
  if (connectDialog.isVisible()) connectDialog.accept();
  showMessage(tr("Connected to %1").arg(connectDialog.getAddress()));
  uploadGCode();
}


void QtWin::on_bbctrlDisconnected() {
  connectDialog.setNetworkStatus(bbCtrlAPI->getStatus());
  showMessage(tr("Disconnected from %1").arg(connectDialog.getAddress()));
}


void QtWin::on_machineChanged(QString machine, QString path) {
  loadMachine(machine.toUtf8().data());
}


void QtWin::on_editorClicked(QString filename, int line) {
  if (project.isNull()) return;
  redraw();
  view->path->setByLine(filename.toUtf8().data(), line);
}


void QtWin::on_fileTabManager_currentChanged(int index) {
  // Keep Space useful throughout the two visualization tabs without stealing
  // it from editable G-code/TPL tabs.
  playPauseShortcut->setEnabled(index < 2);
  if (!index) ui->simulationView->setFocus(Qt::OtherFocusReason);
  redraw();
  updateActions();
}


void QtWin::on_positionSlider_valueChanged(int position) {
  if (isnan(view->path->getTimeRatio())) return;

  double ratio = position / 10000.0;

  view->path->setByRatio(ratio);
  redraw();

  if (sliderMoving) return;
  simulationRequests++;
  if (positionChanged) simulationCoalesced++;
  pendingSimulationTime = ratio * view->path->getTotalTime();
  positionChanged = true;
}


void QtWin::on_positionSlider_sliderPressed() {
  sliderMoving = true;
  view->setFlag(View::TRANSLUCENT_SURFACE_FLAG);
}


void QtWin::on_positionSlider_sliderReleased() {
  sliderMoving = false;
  view->clearFlag(View::TRANSLUCENT_SURFACE_FLAG);
  on_positionSlider_valueChanged(ui->positionSlider->value());
}


void QtWin::on_filesListView_activated(const QModelIndex &index) {
  if (index.isValid()) editFile(index.row());
}


void QtWin::on_filesListView_customContextMenuRequested(QPoint point) {
  QModelIndex index = ui->filesListView->indexAt(point);
  bool valid = index.isValid();

  ui->actionEditFile->setEnabled(valid);
  ui->actionRemoveFile->setEnabled(valid);

  QMenu menu;

  menu.addAction(ui->actionAddFile);
  menu.addAction(ui->actionEditFile);
  menu.addAction(ui->actionRemoveFile);

  menu.exec(ui->filesListView->mapToGlobal(point));
}


void QtWin::on_toolTableListView_activated(const QModelIndex &index) {
  if (index.isValid())
    editTool(index.data(Qt::UserRole).toUInt());
}


void QtWin::on_toolTableListView_customContextMenuRequested(QPoint point) {
  QModelIndex index = ui->toolTableListView->indexAt(point);
  bool valid = index.isValid();
  unsigned number = valid ? index.data(Qt::UserRole).toUInt() : 0;
  bool removable = valid && !project.isNull() &&
    project->getTools().has(number);

  ui->actionEditTool->setEnabled(valid);
  ui->actionRemoveTool->setEnabled(removable);

  QMenu menu;

  menu.addAction(ui->actionAddTool);
  menu.addAction(ui->actionEditTool);
  menu.addAction(ui->actionRemoveTool);
  menu.addSeparator();
  menu.addAction(ui->actionImportToolTable);
  menu.addAction(ui->actionExportToolTable);
  menu.addSeparator();
  menu.addAction(ui->actionLoadDefaultToolTable);
  menu.addAction(ui->actionSaveDefaultToolTable);

  menu.exec(ui->toolTableListView->mapToGlobal(point));
}


void QtWin::onToolTableItemChanged(QStandardItem *item) {
  if (!item) return;
  setSimulationToolEnabled
    (item->data(Qt::UserRole).toUInt(),
     item->checkState() == Qt::Checked);
}


void QtWin::on_automaticCuboidRadioButton_clicked() {
  ui->automaticCuboidRadioButton->setChecked(true);
  ui->automaticCuboidFrame->setEnabled(true);
  ui->manualCuboidFrame->setEnabled(false);

  PROTECT_UI_UPDATE;

  if (project->getWorkpiece().isAutomatic()) return;

  project->getWorkpiece().setAutomatic(true);
  project->getWorkpiece().update(*toolPath);
  loadWorkpiece();

  redraw(true);
}


void QtWin::on_marginDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;

  project->getWorkpiece().setMargin(value);
  if (!toolPath.isNull()) project->getWorkpiece().update(*toolPath);
  loadWorkpiece();

  redraw(true);
}


void QtWin::on_manualCuboidRadioButton_clicked() {
  ui->manualCuboidRadioButton->setChecked(true);
  ui->automaticCuboidFrame->setEnabled(false);
  ui->manualCuboidFrame->setEnabled(true);

  PROTECT_UI_UPDATE;

  if (!project->getWorkpiece().isAutomatic()) return;

  project->getWorkpiece().setAutomatic(false);
  loadWorkpiece();

  redraw(true);
}


void QtWin::on_xDimDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceDim(0, value);
}


void QtWin::on_yDimDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceDim(1, value);
}


void QtWin::on_zDimDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceDim(2, value);
}


void QtWin::on_xOffsetDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceOffset(0, value);
}


void QtWin::on_yOffsetDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceOffset(1, value);
}


void QtWin::on_zOffsetDoubleSpinBox_valueChanged(double value) {
  PROTECT_UI_UPDATE;
  setWorkpieceOffset(2, value);
}


void QtWin::on_languageChanged(QAction *action) {
  if (action) loadLanguage(action->data().toString());
}


void QtWin::on_actionQuit_triggered() {if (checkSave()) quit();}
void QtWin::on_actionNew_triggered() {newProject();}
void QtWin::on_actionOpen_triggered() {openProject();}
void QtWin::on_actionStop_triggered() {stop();}


void QtWin::on_actionRun_triggered() {
  if (ui->fileTabManager->checkSaveAll()) reload(true);
}


void QtWin::on_actionReduce_triggered() {reduce();}
void QtWin::on_actionOptimize_triggered() {optimize();}
void QtWin::on_actionSlower_triggered() {view->decSpeed();}
void QtWin::on_actionPlay_triggered() {
  togglePlayback();
}


void QtWin::togglePlayback() {
  const bool starting = !view->isFlagSet(View::PLAY_FLAG);
  if (starting && !view->isReverse() && view->path->atEnd()) {
    // Replace the completed stock immediately with the uncut placeholder.
    // The next animation tick schedules the exact time-zero retained state.
    view->path->setByRatio(0);
    view->setSurface(0);
    ui->actionDexelHeightMap->setEnabled(false);
    clearDexelHeightMapWindow(tr("Waiting for the current playback state..."));
  }
  view->toggleFlag(View::PLAY_FLAG);
  redraw(true);
}
void QtWin::on_actionFaster_triggered() {view->incSpeed();}
void QtWin::on_actionDirection_triggered() {view->changeDirection();}
void QtWin::on_actionSimulationEnd_triggered() {
  if (view->path.isNull() || view->path->isEmpty()) return;
  view->clearFlag(View::PLAY_FLAG);
  LOG_INFO(1, "GUI simulation go to end");
  on_positionSlider_valueChanged(10000);
  updateStatusDisplay(true);
}


void QtWin::on_actionExamples_triggered() {
  QAction *action = (QAction *)QObject::sender();
  if (action->toolTip().isEmpty()) return;
  openProject(action->toolTip().toUtf8().data());
}


void QtWin::on_actionSave_triggered() {saveProject();}
void QtWin::on_actionSaveAs_triggered() {saveProject(true);}
void QtWin::on_actionSaveFile_triggered() {ui->fileTabManager->save();}


void QtWin::on_actionSaveFileAs_triggered() {
  ui->fileTabManager->saveAs();
  updateFiles();
}


void QtWin::on_actionRevertFile_triggered() {ui->fileTabManager->revert();}


void QtWin::on_actionSaveDefaultToolTable_triggered() {
  if (project.isSet()) saveDefaultToolTable(project->getTools());
  else warning("No project loaded");
}


void QtWin::on_actionLoadDefaultToolTable_triggered() {
  if (project.isSet()) {
    project->getTools() = loadDefaultToolTable();
    updateToolTables();

  } else warning("No project loaded");
}


void QtWin::on_actionSettings_triggered() {
  if (project.isSet() && settingsDialog.exec(*project, *view))
    updateUnits();
}


void QtWin::on_actionConnect_triggered(bool checked) {
  string status = bbCtrlAPI.isNull() ? "Disconnected" : bbCtrlAPI->getStatus();
  connectDialog.setNetworkStatus(status);
  connectDialog.exec();
}


void QtWin::on_actionFullscreen_triggered(bool checked) {
  if (checked) showFullScreen();
  else showNormal();
}


void QtWin::on_actionDefaultLayout_triggered() {defaultLayout();}
void QtWin::on_actionFullLayout_triggered() {fullLayout();}
void QtWin::on_actionMinimalLayout_triggered() {minimalLayout();}
void QtWin::on_actionAbout_triggered() {aboutDialog.exec();}
void QtWin::on_actionDonate_triggered() {donateDialog.exec();}


void QtWin::on_actionHelp_triggered() {
  QMessageBox msg(this);
  msg.setWindowTitle("CAMotics Help");
  msg.setTextFormat(Qt::RichText);
  msg.setText
    ("<h2>CAMotics Help Resources</h2>"
     "<ul>"
     "<li><a href='https://camotics.org/manual.html'>User Manual</a></li>"
     "<li><a href='https://camotics.org/gcode.html'>Supported GCodes</a></li>"
     "<li><a href='https://github.com/CauldronDevelopmentLLC/camotics'>"
     "GitHub Project</a></li>"
     "<li><a href='https://github.com/CauldronDevelopmentLLC/camotics/issues'>"
     "GitHub Issues</a></li>"
     "<li><a href='http://groups.google.com/group/camotics-users/boxsubscribe'>"
     "Discussion Group</a></li>"
     "<li><a href='mailto:joseph@camotics.org'>joseph@camotics.org</a></li>"
     "</ul>"
     "<p>The above links should open in your browser.</p>");
  msg.exec();
}


void QtWin::on_actionCutSurface_triggered() {
  PROTECT_UI_UPDATE;

  view->setFlag(View::WIRE_FLAG, false);
  view->setFlag(View::SHOW_SURFACE_FLAG, true);
  view->setFlag(View::SHOW_WORKPIECE_FLAG, false);

  ui->actionCutSurface->setChecked(true);
  ui->actionWorkpieceSurface->setChecked(false);
  ui->actionWireSurface->setChecked(false);
  ui->actionHideSurface->setChecked(false);

  redraw();
}


void QtWin::on_actionWorkpieceSurface_triggered() {
  PROTECT_UI_UPDATE;

  view->setFlag(View::WIRE_FLAG, false);
  view->setFlag(View::SHOW_SURFACE_FLAG, false);
  view->setFlag(View::SHOW_WORKPIECE_FLAG, true);

  ui->actionCutSurface->setChecked(false);
  ui->actionWorkpieceSurface->setChecked(true);
  ui->actionWireSurface->setChecked(false);
  ui->actionHideSurface->setChecked(false);

  redraw();
}


void QtWin::on_actionWireSurface_triggered() {
  PROTECT_UI_UPDATE;

  view->setFlag(View::WIRE_FLAG, true);
  view->setFlag(View::SHOW_SURFACE_FLAG, true);
  view->setFlag(View::SHOW_WORKPIECE_FLAG, false);

  ui->actionCutSurface->setChecked(false);
  ui->actionWorkpieceSurface->setChecked(false);
  ui->actionWireSurface->setChecked(true);
  ui->actionHideSurface->setChecked(false);

  redraw();
}


void QtWin::on_actionHideSurface_triggered() {
  PROTECT_UI_UPDATE;

  view->setFlag(View::SHOW_SURFACE_FLAG, false);
  view->setFlag(View::SHOW_WORKPIECE_FLAG, false);

  ui->actionCutSurface->setChecked(false);
  ui->actionWorkpieceSurface->setChecked(false);
  ui->actionWireSurface->setChecked(false);
  ui->actionHideSurface->setChecked(true);

  redraw();
}


void QtWin::on_actionAxes_triggered(bool checked) {
  view->setFlag(View::SHOW_AXES_FLAG, checked);
  redraw();
}


void QtWin::on_actionIntensity_triggered(bool checked) {
  view->setFlag(View::PATH_INTENSITY_FLAG, checked);
  redraw();
}


void QtWin::on_actionTool_triggered(bool checked) {
  view->setFlag(View::SHOW_TOOL_FLAG, checked);
  redraw();
}


void QtWin::on_actionMachine_triggered(bool checked) {
  view->setFlag(View::SHOW_MACHINE_FLAG, checked);
  redraw();
}


void QtWin::on_actionWorkpieceBounds_triggered(bool checked) {
  view->setFlag(View::SHOW_WORKPIECE_BOUNDS_FLAG, checked);
  redraw();
}


void QtWin::on_actionToolPath_triggered(bool checked) {
  view->setFlag(View::SHOW_PATH_FLAG, checked);
  redraw();
}


void QtWin::on_actionAddFile_triggered() {
  if (newDialog.exec() != QDialog::Accepted) return;
  newFile(newDialog.tplSelected());
}


void QtWin::on_actionEditFile_triggered() {
  editFile(ui->filesListView->currentIndex().row());
}


void QtWin::on_actionRemoveFile_triggered() {
  removeFile(ui->filesListView->currentIndex().row());
}


void QtWin::on_actionAddTool_triggered() {addTool();}


void QtWin::on_actionEditTool_triggered() {
  QModelIndex index = ui->toolTableListView->currentIndex();
  if (index.isValid()) editTool(index.data(Qt::UserRole).toUInt());
}


void QtWin::on_actionRemoveTool_triggered() {
  QModelIndex index = ui->toolTableListView->currentIndex();
  if (index.isValid()) removeTool(index.data(Qt::UserRole).toUInt());
}


void QtWin::on_actionHideConsole_triggered() {
  QList<int> sizes = ui->splitter->sizes();
  sizes[0] = 1;
  sizes[1] = 0;
  ui->splitter->setSizes(sizes);
}


void QtWin::on_actionShowConsole_triggered() {
  QList<int> sizes = ui->splitter->sizes();
  sizes[0] = 1;
  sizes[1] = 1;
  ui->splitter->setSizes(sizes);
}


void QtWin::on_actionToggleConsole_triggered() {
  bool isHidden = ui->actionHideConsole->isChecked();
  if (isHidden) showConsole();
  else hideConsole();
}


void QtWin::on_hideConsolePushButton_clicked() {
  on_actionHideConsole_triggered();
}


void QtWin::on_clearConsolePushButton_clicked() {ui->console->clear();}


void QtWin::on_actionToggleText_triggered() {
  QList<int> currentSizes = ui->splitterFile->sizes();
  if (!currentSizes[1]) showText();
  else hideText();
}


void QtWin::on_actionZoomIn_triggered() {
  view->zoomIn();
  redraw(true);
}


void QtWin::on_actionZoomOut_triggered() {
  view->zoomOut();
  redraw(true);
}


void QtWin::on_actionZoomAll_triggered() {
  // TODO Calculate extents of all displayed geometry and scale accordingly.
  view->center();
  redraw(true);
}
