/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2026 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "QtWin.h"

#include <camotics/sim/DexelHeightMap.h>
#include <camotics/sim/DexelSimulation.h>

#include <cbang/Exception.h>
#include <cbang/log/Logger.h>

#include <QAction>
#include <QApplication>
#include <QCryptographicHash>
#include <QDesktopWidget>
#include <QDialog>
#include <QKeySequence>
#include <QLabel>
#include <QPalette>
#include <QScrollArea>
#include <QScrollBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <QVariant>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  class HeightMapScrollArea : public QScrollArea {
  public:
    function<void(int)> zoomSteps;

    explicit HeightMapScrollArea(QWidget *parent = 0) : QScrollArea(parent) {}

  protected:
    void wheelEvent(QWheelEvent *event) override {
      if ((event->modifiers() & Qt::ControlModifier) && zoomSteps) {
        QPoint delta = event->angleDelta();
        if (delta.isNull()) delta = event->pixelDelta();
        if (delta.y()) {
          zoomSteps(0 < delta.y() ? 1 : -1);
          event->accept();
          return;
        }
      }

      QScrollArea::wheelEvent(event);
    }
  };


  QPixmap getHeightMapSource(QDialog *dialog) {
    QLabel *imageLabel =
      dialog->findChild<QLabel *>("dexelHeightMapImage");
    return imageLabel ?
      qvariant_cast<QPixmap>(imageLabel->property("sourcePixmap")) :
      QPixmap();
  }


  void setHeightMapControlsEnabled(QDialog *dialog, bool enabled) {
    static const char *names[] = {
      "dexelHeightMapZoomIn", "dexelHeightMapZoomOut",
      "dexelHeightMapActualSize", "dexelHeightMapFitWindow"
    };
    for (const char *name: names) {
      QAction *action = dialog->findChild<QAction *>(name);
      if (action) action->setEnabled(enabled);
    }
  }


  void setHeightMapZoom(QDialog *dialog, double requested,
                        bool recenter = true) {
    QLabel *imageLabel =
      dialog->findChild<QLabel *>("dexelHeightMapImage");
    QScrollArea *scroll =
      dialog->findChild<QScrollArea *>("dexelHeightMapScrollArea");
    QLabel *zoomLabel =
      dialog->findChild<QLabel *>("dexelHeightMapZoomPercent");
    QPixmap sourcePixmap = getHeightMapSource(dialog);
    if (!imageLabel || !scroll || !zoomLabel || sourcePixmap.isNull()) return;

    double oldScale = dialog->property("zoomScale").toDouble();
    if (oldScale <= 0) oldScale = 1;
    double scale = min(16.0, max(0.0625, requested));
    QSize size(max(1, qRound(sourcePixmap.width() * scale)),
               max(1, qRound(sourcePixmap.height() * scale)));
    imageLabel->setFixedSize(size);
    dialog->setProperty("zoomScale", scale);
    zoomLabel->setText(QString("%1%").arg(qRound(scale * 100)));

    if (recenter && fabs(scale - oldScale) >= 1e-12) {
      double factor = scale / oldScale;
      auto adjust = [factor](QScrollBar *bar) {
        bar->setValue(qRound(factor * bar->value() +
                             (factor - 1) * bar->pageStep() / 2.0));
      };
      adjust(scroll->horizontalScrollBar());
      adjust(scroll->verticalScrollBar());
    }
  }


  void fitHeightMapToWindow(QDialog *dialog) {
    QScrollArea *scroll =
      dialog->findChild<QScrollArea *>("dexelHeightMapScrollArea");
    QPixmap sourcePixmap = getHeightMapSource(dialog);
    if (!scroll || sourcePixmap.isNull()) return;

    int width = max(1, scroll->viewport()->width() - 4);
    int height = max(1, scroll->viewport()->height() - 4);
    setHeightMapZoom(dialog, min(width / (double)sourcePixmap.width(),
                                 height / (double)sourcePixmap.height()));
  }
}


void QtWin::showDexelHeightMap() {
  QDialog *existing = findChild<QDialog *>("dexelHeightMapWindow");
  if (existing) {
    updateDexelHeightMapWindow();
    existing->show();
    existing->raise();
    existing->activateWindow();
    return;
  }

  Dexel::GridSurface *grid =
    dynamic_cast<Dexel::GridSurface *>(surface.get());
  if (!grid) {
    warning(tr("No Dexel grid is available.\nRun an Auto simulation that "
               "selects the Dexel backend first."));
    return;
  }

  uint64_t gridWidth = (uint64_t)grid->getNX() + 1;
  uint64_t gridHeight = (uint64_t)grid->getNY() + 1;
  if (numeric_limits<int>::max() < gridWidth ||
      numeric_limits<int>::max() < gridHeight)
    THROW("Dexel height map exceeds Qt image dimensions");

  QDialog *dialog = new QDialog(this, Qt::Window);
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->setObjectName("dexelHeightMapWindow");
  dialog->setProperty("gridWidth", 0);
  dialog->setProperty("gridHeight", 0);
  dialog->setProperty("zoomScale", 1.0);
  dialog->setProperty("fitMode", false);
  dialog->setProperty("imageAvailable", false);
  dialog->setProperty("liveRevision", (qulonglong)0);
  dialog->setProperty("staleClearCount", (qulonglong)0);
  dialog->setWindowTitle(tr("Dexel Height Map"));

  QLabel *rangeLabel = new QLabel(tr("Loading current Dexel result..."),
                                  dialog);
  rangeLabel->setObjectName("dexelHeightMapRange");
  QLabel *imageLabel = new QLabel(dialog);
  imageLabel->setObjectName("dexelHeightMapImage");
  imageLabel->setAlignment(Qt::AlignCenter);
  imageLabel->setProperty("sourcePixmap", QVariant::fromValue(QPixmap()));
  imageLabel->setFixedSize(1, 1);

  HeightMapScrollArea *scroll = new HeightMapScrollArea(dialog);
  scroll->setObjectName("dexelHeightMapScrollArea");
  scroll->setBackgroundRole(QPalette::Dark);
  scroll->setWidget(imageLabel);
  scroll->setWidgetResizable(false);

  QToolBar *controls = new QToolBar(dialog);
  controls->setObjectName("dexelHeightMapControls");
  controls->setMovable(false);
  controls->setFloatable(false);
  controls->setToolButtonStyle(Qt::ToolButtonTextOnly);

  QAction *zoomOut = controls->addAction(tr("Zoom Out"));
  zoomOut->setObjectName("dexelHeightMapZoomOut");
  zoomOut->setShortcut(QKeySequence::ZoomOut);
  zoomOut->setToolTip(tr("Zoom Out (Ctrl+-)"));
  QAction *zoomIn = controls->addAction(tr("Zoom In"));
  zoomIn->setObjectName("dexelHeightMapZoomIn");
  zoomIn->setShortcut(QKeySequence::ZoomIn);
  zoomIn->setToolTip(tr("Zoom In (Ctrl++)"));
  QAction *actualSize = controls->addAction(tr("Actual Size"));
  actualSize->setObjectName("dexelHeightMapActualSize");
  actualSize->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
  actualSize->setToolTip(tr("Actual Size (Ctrl+0)"));
  QAction *fitWindow = controls->addAction(tr("Fit to Window"));
  fitWindow->setObjectName("dexelHeightMapFitWindow");
  fitWindow->setShortcut(QKeySequence(Qt::Key_F));
  fitWindow->setToolTip(tr("Fit to Window (F)"));

  controls->addSeparator();
  QLabel *zoomLabel = new QLabel("100%", controls);
  zoomLabel->setObjectName("dexelHeightMapZoomPercent");
  zoomLabel->setAlignment(Qt::AlignCenter);
  zoomLabel->setMinimumWidth(58);
  controls->addWidget(zoomLabel);
  setHeightMapControlsEnabled(dialog, false);

  auto zoomBy = [dialog](double factor) {
    dialog->setProperty("fitMode", false);
    setHeightMapZoom
      (dialog, dialog->property("zoomScale").toDouble() * factor);
  };

  connect(zoomOut, &QAction::triggered, dialog,
          [zoomBy]() {zoomBy(1 / 1.25);});
  connect(zoomIn, &QAction::triggered, dialog,
          [zoomBy]() {zoomBy(1.25);});
  connect(actualSize, &QAction::triggered, dialog,
          [dialog]() {
            dialog->setProperty("fitMode", false);
            setHeightMapZoom(dialog, 1);
          });
  connect(fitWindow, &QAction::triggered, dialog,
          [dialog]() {
            dialog->setProperty("fitMode", true);
            fitHeightMapToWindow(dialog);
          });
  scroll->zoomSteps = [zoomBy](int steps) {zoomBy(pow(1.25, steps));};

  QVBoxLayout *layout = new QVBoxLayout(dialog);
  layout->addWidget(rangeLabel);
  layout->addWidget(controls);
  layout->addWidget(scroll);

  QRect available = QApplication::desktop()->availableGeometry(this);
  int dialogWidth = min(available.width() * 4 / 5,
                        max(420, (int)gridWidth + 32));
  int dialogHeight = min(available.height() * 4 / 5,
                         max(320, (int)gridHeight + 120));
  dialog->resize(dialogWidth, dialogHeight);
  dialog->show();
  updateDexelHeightMapWindow();
}


void QtWin::clearDexelHeightMapWindow(const QString &status) {
  QDialog *dialog = findChild<QDialog *>("dexelHeightMapWindow");
  if (!dialog) return;

  QLabel *rangeLabel =
    dialog->findChild<QLabel *>("dexelHeightMapRange");
  QLabel *imageLabel =
    dialog->findChild<QLabel *>("dexelHeightMapImage");
  QScrollArea *scroll =
    dialog->findChild<QScrollArea *>("dexelHeightMapScrollArea");
  QLabel *zoomLabel =
    dialog->findChild<QLabel *>("dexelHeightMapZoomPercent");
  if (!rangeLabel || !imageLabel || !scroll || !zoomLabel) return;

  const bool hadImage = dialog->property("imageAvailable").toBool();
  if (hadImage) {
    dialog->setProperty("preservedGridWidth", dialog->property("gridWidth"));
    dialog->setProperty("preservedGridHeight",
                        dialog->property("gridHeight"));
    dialog->setProperty("preservedHorizontalScroll",
                        scroll->horizontalScrollBar()->value());
    dialog->setProperty("preservedVerticalScroll",
                        scroll->verticalScrollBar()->value());
    dialog->setProperty
      ("staleClearCount",
       dialog->property("staleClearCount").toULongLong() + 1);
    dialog->setProperty
      ("liveRevision", dialog->property("liveRevision").toULongLong() + 1);
  }

  imageLabel->setProperty("sourcePixmap", QVariant::fromValue(QPixmap()));
  imageLabel->setScaledContents(false);
  imageLabel->clear();
  imageLabel->setText(status);
  imageLabel->setFixedSize(max(320, scroll->viewport()->width()), 80);
  rangeLabel->setText(status);
  zoomLabel->setText(QString::fromUtf8("—"));
  setHeightMapControlsEnabled(dialog, false);
  dialog->setProperty("imageAvailable", false);
  dialog->setProperty("contentHash", QByteArray());
  dialog->setWindowTitle(tr("Dexel Height Map — updating"));

  if (hadImage)
    LOG_INFO(1, "GUI Dexel height map cleared: revision="
             << dialog->property("liveRevision").toULongLong()
             << " reason=" << status.toStdString());
}


void QtWin::updateDexelHeightMapWindow() {
  QDialog *dialog = findChild<QDialog *>("dexelHeightMapWindow");
  if (!dialog) return;

  Dexel::GridSurface *grid =
    dynamic_cast<Dexel::GridSurface *>(surface.get());
  if (!grid) {
    clearDexelHeightMapWindow
      (tr("The current result does not expose a Dexel grid."));
    return;
  }

  Dexel::HeightMap map = Dexel::makeHeightMap(*grid);
  if (numeric_limits<int>::max() < map.width ||
      numeric_limits<int>::max() < map.height)
    THROW("Dexel height map exceeds Qt image dimensions");

  QImage image((int)map.width, (int)map.height, QImage::Format_Grayscale8);
  for (unsigned y = 0; y < map.height; y++)
    memcpy(image.scanLine(y),
           map.pixels.data() + (uint64_t)y * map.width, map.width);

  QLabel *rangeLabel =
    dialog->findChild<QLabel *>("dexelHeightMapRange");
  QLabel *imageLabel =
    dialog->findChild<QLabel *>("dexelHeightMapImage");
  QScrollArea *scroll =
    dialog->findChild<QScrollArea *>("dexelHeightMapScrollArea");
  if (!rangeLabel || !imageLabel || !scroll) return;

  const int preservedWidth = dialog->property("preservedGridWidth").toInt();
  const int preservedHeight =
    dialog->property("preservedGridHeight").toInt();
  const bool restoreNavigation =
    preservedWidth == (int)map.width && preservedHeight == (int)map.height;
  const int horizontalScroll =
    dialog->property("preservedHorizontalScroll").toInt();
  const int verticalScroll =
    dialog->property("preservedVerticalScroll").toInt();

  QPixmap sourcePixmap = QPixmap::fromImage(image);
  imageLabel->setText(QString());
  imageLabel->setScaledContents(true);
  imageLabel->setProperty("sourcePixmap", QVariant::fromValue(sourcePixmap));
  imageLabel->setPixmap(sourcePixmap);
  dialog->setProperty("gridWidth", map.width);
  dialog->setProperty("gridHeight", map.height);
  dialog->setProperty("imageAvailable", true);
  dialog->setProperty
    ("liveRevision", dialog->property("liveRevision").toULongLong() + 1);
  if (options["test-dexel-grid-window"].toBoolean())
    dialog->setProperty
      ("contentHash", QCryptographicHash::hash
       (QByteArray((const char *)map.pixels.data(), map.pixels.size()),
        QCryptographicHash::Sha256));

  rangeLabel->setText
    (tr("Black/deepest: Z %1   White/highest: Z %2   +Y is up")
     .arg(map.minZ, 0, 'g', 8).arg(map.maxZ, 0, 'g', 8));
  dialog->setWindowTitle
    (tr("Dexel Height Map — %1 × %2").arg(map.width).arg(map.height));
  setHeightMapControlsEnabled(dialog, true);
  if (dialog->property("fitMode").toBool()) fitHeightMapToWindow(dialog);
  else setHeightMapZoom(dialog, dialog->property("zoomScale").toDouble(),
                        false);

  if (restoreNavigation) {
    scroll->horizontalScrollBar()->setValue(horizontalScroll);
    scroll->verticalScrollBar()->setValue(verticalScroll);
  }
  dialog->setProperty("preservedGridWidth", 0);
  dialog->setProperty("preservedGridHeight", 0);

  LOG_INFO(1, "GUI Dexel height map refreshed: revision="
           << dialog->property("liveRevision").toULongLong()
           << " width=" << map.width << " height=" << map.height
           << " min_z=" << map.minZ << " max_z=" << map.maxZ
           << " disabled_tools=" << disabledSimulationTools.size());
}
