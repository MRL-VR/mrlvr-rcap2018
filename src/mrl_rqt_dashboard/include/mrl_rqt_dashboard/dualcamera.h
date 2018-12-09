#ifndef mrl_rqt_dashboard__DualCamera_H
#define mrl_rqt_dashboard__DualCamera_H

#include "mrl_rqt_dashboard/ui_dualcamera.h"
#include "mrl_rqt_dashboard/image_view.h"

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>

namespace mrl_rqt_dashboard {

class DualCamera
    : public QWidget
{

  Q_OBJECT

public:

  DualCamera(QWidget *parent = 0);

  QString botName;
  QString botType;
  QString botRgbTopic;
  QString botThermalTopic;


  QString getBotName() const;
  void setBotName(const QString &value);

  QString getBotType() const;
  void setBotType(const QString &value);

  QString getBotRgbTopic() const;
  void setBotRgbTopic(const QString &value);

  QString getBotThermalTopic() const;
  void setBotThermalTopic(const QString &value);

  void connect();

protected slots:

protected:
  virtual void resizeEvent(QResizeEvent *event);
  Ui::DualCameraWidget ui;

protected slots:

protected:
  QWidget* widget;

private:
  mrl_rqt_dashboard::ImageView *rgb_view;
  mrl_rqt_dashboard::ImageView *thermal_view;
};

}

#endif 
