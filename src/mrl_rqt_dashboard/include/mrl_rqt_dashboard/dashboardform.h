#ifndef DASHBOARDFORM_H
#define DASHBOARDFORM_H

#include "ros/ros.h"

#include "mrl_rqt_dashboard/robot_widget.h"
#include "rviz/visualization_frame.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <QWidget>
#include <QKeyEvent>
#include <QObject>
#include <QEvent>
#include <std_msgs/String.h>
#include <QList>
#include <QProcess>
#include <QTimer>

namespace Ui {
class DashboardForm;
}

class DashboardForm : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardForm(QWidget *parent = 0);
    ~DashboardForm();

    void loadSettings();
    void startJoystick();

    QProcess *joyProcess;

    QProcess *bringupProcess;
    QProcess *spawnProcess;

    bool isMapLaunch;

    QTimer *timer;
    int time_counter;
    int selectedRobot = 0;


signals:


private slots:
    virtual void joystickSelectEvent(mrl_rqt_dashboard::RobotWidget *item);
    virtual void autonomousSelectEvent(mrl_rqt_dashboard::RobotWidget *item);
    virtual void openSettingDialog();
    virtual void openCameraDialog();
    virtual void openMapDialog();
    virtual void updateCounter();


    virtual void joystickAllEvent();
    virtual void autonomousAllEvent();

private:
    void init();
    void selectRobot(int r);
    QList<mrl_rqt_dashboard::RobotWidget*> robots;
    Ui::DashboardForm *ui;
    rviz::VisualizationFrame *rviz_frame;


protected:
    bool eventFilter( QObject* obj, QEvent *event );

};

#endif // DASHBOARDFORM_H
