#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <QString>
#include <QJsonObject>

class RobotModel
{

public:
  RobotModel();
  ~RobotModel();

  bool getIsActive() const;
  void setIsActive(bool value);

  QString getName() const;
  void setName(const QString &value);

  QString getType() const;
  void setType(const QString &value);

  QString getRgbTopic() const;
  void setRgbTopic(const QString &value);

  QString getThermalTopic() const;
  void setThermalTopic(const QString &value);

  QString getLaserTopic() const;
  void setLaserTopic(const QString &value);

  QString getOdomTopic() const;
  void setOdomTopic(const QString &value);

  QJsonObject getJsonObject() const;
  void setJsonObject(const QJsonObject &value);

  QString getCmdTopic() const;
  void setCmdTopic(const QString &value);

  QString getInitPosX() const;
  void setInitPosX(const QString &value);

  QString getInitPosY() const;
  void setInitPosY(const QString &value);

  QString getInitPosZ() const;
  void setInitPosZ(const QString &value);

  QString getInitRotX() const;
  void setInitRotX(const QString &value);

  QString getInitRotY() const;
  void setInitRotY(const QString &value);

  QString getInitRotZ() const;
  void setInitRotZ(const QString &value);

  QString getNavStrategy() const;
  void setNavStrategy(const QString &value);

  QString getJoyTypeStrategy() const;
  void setJoyTypeStrategy(const QString &value);

private:
  bool isActive;
  QString name;
  QString type;
  QString initPosX;
  QString initPosY;
  QString initPosZ;
  QString initRotX;
  QString initRotY;
  QString initRotZ;
  QString rgbTopic;
  QString thermalTopic;
  QString laserTopic;
  QString odomTopic;
  QString cmdTopic;
  QString navStrategy;
  QString joyTypeStrategy;
};

#endif // ROBOTMODEL_H

