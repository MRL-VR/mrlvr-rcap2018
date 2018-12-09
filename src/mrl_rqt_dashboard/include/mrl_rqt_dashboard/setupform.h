#ifndef SETUPFORM_H
#define SETUPFORM_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <QWidget>
#include <std_msgs/String.h>
#include <rqt_gui_cpp/plugin.h>
#include "mrl_rqt_dashboard/robotitem.h"
#include <QListWidgetItem>
#include <QDialog>

namespace Ui {
  class SetupForm;
}

class SetupForm : public QDialog
{
  Q_OBJECT

public:
  explicit SetupForm(QWidget *parent = 0);
  ~SetupForm(); 

private slots:
  virtual void openAddRobotDialog();
  virtual void editItemsEvent(RobotItem *item);
  virtual void removeItemsEvent(QListWidgetItem *item);

private:
  void init();
  void saveSettings();
  void loadSettings();

  Ui::SetupForm *ui;
};

#endif // CAMERAFORM_H
