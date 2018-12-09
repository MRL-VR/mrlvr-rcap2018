#ifndef mrl_rqt_dashboard__RobotWidget_H
#define mrl_rqt_dashboard__RobotWidget_H

#include "mrl_rqt_dashboard/ui_robot_widget.h"
#include "mrl_rqt_dashboard/robotmodel.h"
#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QFrame>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <QProcess>

#define IDLE 0
#define JOSYTICK 1
#define AUTONOMOUS 2

namespace mrl_rqt_dashboard {

class RobotWidget
    : public QWidget
{

  Q_OBJECT

public:

  RobotWidget(QWidget *parent = 0);
  ~RobotWidget();

  RobotModel *getModel() const;
  void setModel(RobotModel *value);

  void init();
  void select(bool value);
  void autonomous(bool value);

  int getStatus() const;
  void setStatus(int value);
  ros::Publisher victim_pub;
  ros::Publisher teleop_keyboard_pub;
  ros::Publisher teleop_joy_pub;

  ros::Publisher explore_control_pub;
  QProcess *rProccess;

protected slots:

protected:
  virtual void resizeEvent(QResizeEvent *event);
  Ui::RobotWidget ui;


protected slots:

signals:
  void joystickButtonEvent(mrl_rqt_dashboard::RobotWidget *item);
  void autonomousButtonEvent(mrl_rqt_dashboard::RobotWidget *item);

private slots:
  virtual void joystickbuttonClicked();
  virtual void autonomousButtonClicked();
  virtual void aliveButtonClicked();
  virtual void deadButtonClicked();



protected:
  QWidget* widget;

private:
  QFrame* frame;
  RobotModel* model;
  int status = IDLE;

};

}

#endif 
