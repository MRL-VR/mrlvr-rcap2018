#include "mrl_rqt_dashboard/commons.h"

Commons::Commons()
{
//  instance = new Commons();
}

QList<mrl_rqt_dashboard::RobotWidget *> Commons::getRobots() const
{
  return robots;
}

void Commons::setRobots(const QList<mrl_rqt_dashboard::RobotWidget *> &value)
{
  robots = value;
}

void Commons::appendRobot(mrl_rqt_dashboard::RobotWidget * newRobot)
{
  robots.append(newRobot);
}
