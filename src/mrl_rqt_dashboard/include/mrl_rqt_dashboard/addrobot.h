#ifndef ADDROBOT_H
#define ADDROBOT_H

#include "mrl_rqt_dashboard/robotmodel.h"
#include "ros/ros.h"

#include <std_msgs/String.h>
#include <QDialog>
#include <image_transport/image_transport.h>
#include <ros/subscriber.h>
#include <nav_msgs/Odometry.h>

namespace Ui {
class AddRobot;
}

class AddRobot : public QDialog
{
    Q_OBJECT

public:
    explicit AddRobot(QWidget *parent = 0);
    ~AddRobot();
    RobotModel *getModel();
    void setModel(RobotModel *value);
    QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void updateTopicList();

private slots:
    virtual void getInitPositions();

private:
    Ui::AddRobot *ui;
    ros::Subscriber map_odom;
};

#endif // ADDROBOT_H
