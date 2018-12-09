#ifndef CAMERAFORM_H
#define CAMERAFORM_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <QWidget>
#include <QDialog>
#include <std_msgs/String.h>
#include <rqt_gui_cpp/plugin.h>

namespace Ui {
  class CameraForm;
}

class CameraForm : public QDialog
{
  Q_OBJECT

public:
  explicit CameraForm(QWidget *parent = 0);
  ~CameraForm();
  void loadSettings();

signals:


private slots:

private:
  void init();

  Ui::CameraForm *ui;
};

#endif // CAMERAFORM_H
