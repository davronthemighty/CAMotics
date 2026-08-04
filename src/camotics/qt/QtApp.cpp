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

#include "QtApp.h"

#include "QtWin.h"
#include "QApplication.h"

#include <cbang/Info.h>
#include <cbang/String.h>
#include <cbang/log/Logger.h>
#include <cbang/os/SystemInfo.h>

#include <QTranslator>

#include <vector>

using namespace std;
using namespace cb;
using namespace CAMotics;


QtApp::QtApp() :
  // NOTE MSVC requires the explicit _hasFeature reference
  CAMotics::Application("CAMotics", CAMotics::Application::_hasFeature),
  threads(SystemInfo::instance().getCPUCount()) {
  options.add("qt-style", "Set Qt style");
  options.add("fullscreen", "Start in fullscreen mode.")->setDefault(false);
  options.add("auto-play", "Automatically start tool path playback.")
    ->setDefault(false);
  options.add("play-speed", "Set playback speed.")->setDefault(1);
  options.add("auto-close", "Automatically exit after tool path playback is "
              "complete.  Only valid with 'auto-play'")
    ->setDefault(false);
  options.add("auto-close-after-simulation", "Automatically exit after the "
              "initial simulation finishes.  Intended for GUI testing.")
    ->setDefault(false);
  options.add("simulation-backend", "Override the GUI simulation backend: "
              "full-mc or auto-dexel.");
  options.add("simulation-output", "Write the completed GUI simulation to a "
              "binary STL path.  Intended for GUI testing.");
  options.add("simulation-seek-ratios", "After the initial GUI simulation, "
              "seek through comma-separated timeline ratios.  Intended for "
              "GUI testing.");
  options.add("simulation-seek-burst-ratios", "After the initial GUI "
              "simulation, inject comma-separated timeline ratios before "
              "the next task is scheduled.  Intended for coalescing tests.");
  options.add("simulation-go-to-end", "After test seeks, invoke the GUI End "
              "action and publish the exact final state.")
    ->setDefault(false);
  options.add("disable-tools", "Comma-separated tool numbers excluded from "
              "GUI stock removal.  Intended for GUI testing.");
  options.add("machine", "Initial machine profile override.");
  options.add("view-frame", "Initial GUI reference frame: stock or tool.");
  options.add("test-view-controls", "Exercise Space, precision-trackpad "
              "navigation, tool-table population, and dock recovery.  "
              "Intended for GUI testing.")
    ->setDefault(false);
  options.add("test-dexel-grid-window", "Open and verify the current Dexel "
              "height-map window after simulation.  Intended for GUI "
              "testing.")
    ->setDefault(false);
  options.add("validate-dexel-topology", "Run the diagnostic topology scan "
              "after every accepted GUI dexel simulation.")
    ->setDefault(false);
  options.addTarget("threads", threads, "GCode::Number of simulation threads.");

  // Configure Logger
  Logger &logger = Logger::instance();
  logger.setLogTime(false);
  logger.setLogNoInfoHeader(true);
  logger.setLogCRLF(false);
  logger.setLogColor(true);

  // Configure command line
  cmdLine.setAllowConfigAsFirstArg(false);
  cmdLine.setAllowPositionalArgs(true);
  cmdLine.setWarnOnInvalidArgs(true);
}


QtApp::~QtApp() {}


int QtApp::init(int argc, char *argv[]) {
  this->argc = argc;
  this->argv = argv;

  int ret = Application::init(argc, argv);
  if (ret < 0) return ret;

  // Rendering attributes must be selected before the first Qt application
  // object is constructed.  init() creates a temporary QGuiApplication to
  // query the primary screen before run() creates the QApplication.
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
#ifdef _WIN32
  // MSYS2 Qt uses native Windows OpenGL and does not ship ANGLE's
  // libGLESv2.dll.
  QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
#else
  QCoreApplication::setAttribute(Qt::AA_UseOpenGLES);
#endif

  QGuiApplication guiApp(argc, argv);
  QScreen *screen = guiApp.primaryScreen();
  Info::instance().add("System", "DPI", String(screen->logicalDotsPerInch()));

  printInfo();

  // Get project file
  const vector<string> &args = cmdLine.getPositionalArgs();
  if (1 <= args.size()) {
    projectFile = args[0];
    if (1 < args.size()) LOG_WARNING("Ignoring extra positional arguments");
  }

  return 0;
}


void QtApp::run() {
  vector<const char *> args;
  args.push_back(argv[0]);
  if (options["qt-style"].hasValue()) {
    args.push_back("-style");
    args.push_back(strdup(options["qt-style"].toString().c_str()));
  }

  // These must come before QApplication is constructed.
  string org = Info::instance().get(getName(), "Org");
  QCoreApplication::setOrganizationName(QString::fromUtf8(org.c_str()));
  QCoreApplication::setApplicationName(QString::fromUtf8(getName().c_str()));

  int argc = args.size();
  QApplication qtApp(argc, (char **)&args[0]);

  QtWin qtWin(*this, qtApp);
  qtWin.init();

  // Options
  if (options["fullscreen"].toBoolean())
    qtWin.setWindowState(qtWin.windowState() | Qt::WindowFullScreen);

  if (projectFile.empty()) qtWin.loadDefaultExample();
  else qtWin.openProject(projectFile);

  qtWin.getView().setSpeed(options["play-speed"].toInteger());
  if (options["machine"].hasValue())
    qtWin.loadMachine(options["machine"].toString());
  if (options["view-frame"].hasValue()) {
    string frame = String::toLower(options["view-frame"].toString());
    if (frame == "stock") qtWin.setReferenceFrame(View::STOCK_FRAME);
    else if (frame == "tool") qtWin.setReferenceFrame(View::TOOL_FRAME);
    else LOG_WARNING("Unknown GUI view frame '" << frame
                     << "'; using stock");
  }

  if (options["disable-tools"].hasValue()) {
    string values = options["disable-tools"].toString();
    size_t begin = 0;
    while (begin < values.size()) {
      size_t end = values.find(',', begin);
      string value = values.substr(begin, end - begin);
      if (value.empty()) THROW("GUI disabled tool number cannot be empty");
      qtWin.setSimulationToolEnabled(String::parseU32(value), false);
      if (end == string::npos) break;
      begin = end + 1;
    }
  }

  if (options["auto-play"].toBoolean()) {
    qtWin.setAutoPlay();
    if (options["auto-close"].toBoolean()) qtWin.setAutoClose();
  }

  if (options["auto-close-after-simulation"].toBoolean())
    qtWin.setAutoCloseAfterSimulation();

  // Start it up
  qtWin.show();
  qtApp.exec();
}


void QtApp::requestExit() {Application::requestExit();}
