#include <mrl_rqt_dashboard/robotmodel.h>

RobotModel::RobotModel()
{

}

RobotModel::~RobotModel()
{

}


bool RobotModel::getIsActive() const
{
  return isActive;
}

void RobotModel::setIsActive(bool value)
{
  isActive = value;
}


QString RobotModel::getName() const
{
    return name;
}

void RobotModel::setName(const QString &value)
{
    name = value;
}

QString RobotModel::getType() const
{
    return type;
}

void RobotModel::setType(const QString &value)
{
    type = value;
}


void RobotModel::setRgbTopic(const QString &value)
{
    rgbTopic = value;
}

QString RobotModel::getRgbTopic() const
{

    return rgbTopic;
}

QString RobotModel::getThermalTopic() const
{
    return thermalTopic;
}

void RobotModel::setThermalTopic(const QString &value)
{
    thermalTopic = value;
}

QString RobotModel::getLaserTopic() const
{
    return laserTopic;
}

void RobotModel::setLaserTopic(const QString &value)
{
    laserTopic = value;
}

QString RobotModel::getOdomTopic() const
{
    return odomTopic;
}

void RobotModel::setOdomTopic(const QString &value)
{
    odomTopic = value;
}

QJsonObject RobotModel::getJsonObject() const
{
  QJsonObject *obj = new QJsonObject();

  obj->insert("active",isActive);
  obj->insert("name",name);
  obj->insert("type",type);
  obj->insert("init_pos_x",initPosX);
  obj->insert("init_pos_y",initPosY);
  obj->insert("init_pos_z",initPosZ);
  obj->insert("init_rot_x",initRotX);
  obj->insert("init_rot_y",initRotY);
  obj->insert("init_rot_z",initRotZ);
  obj->insert("rgb_topic",rgbTopic);
  obj->insert("thermal_topic",thermalTopic);
  obj->insert("laser_topic",laserTopic);
  obj->insert("odom_topic",odomTopic);
  obj->insert("cmd_topic",cmdTopic);
  obj->insert("nav_strategy",navStrategy);
  obj->insert("joy_type_strategy",joyTypeStrategy);

  return *obj;
}

void RobotModel::setJsonObject(const QJsonObject &obj)
{
  isActive = obj["active"].toBool();
  name = obj["name"].toString();
  type = obj["type"].toString();
  initPosX = obj["init_pos_x"].toString();
  initPosY = obj["init_pos_y"].toString();
  initPosZ = obj["init_pos_z"].toString();

  initRotX = obj["init_rot_x"].toString();
  initRotY = obj["init_rot_y"].toString();
  initRotZ = obj["init_rot_z"].toString();

  rgbTopic = obj["rgb_topic"].toString();
  thermalTopic = obj["thermal_topic"].toString();
  laserTopic = obj["laser_topic"].toString();
  odomTopic = obj["odom_topic"].toString();
  cmdTopic = obj["cmd_topic"].toString();
  navStrategy = obj["nav_strategy"].toString();
  joyTypeStrategy = obj["joy_type_strategy"].toString();
}

QString RobotModel::getCmdTopic() const
{
    return cmdTopic;
}

void RobotModel::setCmdTopic(const QString &value)
{
    cmdTopic = value;
}

QString RobotModel::getInitPosX() const
{
    return initPosX;
}

void RobotModel::setInitPosX(const QString &value)
{
    initPosX = value;
}

QString RobotModel::getInitPosY() const
{
    return initPosY;
}

void RobotModel::setInitPosY(const QString &value)
{
    initPosY = value;
}

QString RobotModel::getInitPosZ() const
{
    return initPosZ;
}

void RobotModel::setInitPosZ(const QString &value)
{
    initPosZ = value;
}

QString RobotModel::getInitRotX() const
{
    return initRotX;
}

void RobotModel::setInitRotX(const QString &value)
{
    initRotX = value;
}

QString RobotModel::getInitRotY() const
{
    return initRotY;
}

void RobotModel::setInitRotY(const QString &value)
{
    initRotY = value;
}

QString RobotModel::getInitRotZ() const
{
    return initRotZ;
}

void RobotModel::setInitRotZ(const QString &value)
{
    initRotZ = value;
}

QString RobotModel::getNavStrategy() const
{
    return navStrategy;
}

void RobotModel::setNavStrategy(const QString &value)
{
    navStrategy = value;
}

QString RobotModel::getJoyTypeStrategy() const
{
    return joyTypeStrategy;
}

void RobotModel::setJoyTypeStrategy(const QString &value)
{
    joyTypeStrategy = value;
}
