#ifndef COMMONS_H
#define COMMONS_H

#include "mrl_rqt_dashboard/robot_widget.h"
#include <QList>

class Commons
{
public:
    static Commons *instance;

    Commons();
    QList<mrl_rqt_dashboard::RobotWidget *> getRobots() const;
    void setRobots(const QList<mrl_rqt_dashboard::RobotWidget *> &value);
    void appendRobot(mrl_rqt_dashboard::RobotWidget* newRobot);


private:
    QList<mrl_rqt_dashboard::RobotWidget*> robots;

};

#endif // COMMONS_H

