#include "mrl_rqt_dashboard/addrobot.h"
#include "mrl_rqt_dashboard/ui_addrobot.h"
#include "stdlib.h"

AddRobot::AddRobot(QWidget *parent) : QDialog(parent), ui(new Ui::AddRobot)
{
  ui->setupUi(this);
  connect(ui->getPosPushButton, SIGNAL(pressed()), this, SLOT(getInitPositions()));

  updateTopicList();
}

AddRobot::~AddRobot()
{
  delete ui;
}

RobotModel *AddRobot::getModel()
{
  RobotModel *model = new RobotModel();

  model->setName(ui->nameLineEdit->text());
  model->setType(ui->typeComboBox->currentText());

  model->setInitPosX(ui->initPosXLineEdit->text());
  model->setInitPosY(ui->initPosYLineEdit->text());
  model->setInitPosZ(ui->initPosZLineEdit->text());
  model->setInitRotX(ui->initRotXLineEdit->text());
  model->setInitRotY(ui->initRotYLineEdit->text());
  model->setInitRotZ(ui->initRotZLineEdit->text());

  model->setRgbTopic(ui->rgbCameraComboBox->itemData(ui->rgbCameraComboBox->currentIndex()).toString());
  model->setThermalTopic(ui->thermalCameraComboBox->itemData(ui->thermalCameraComboBox->currentIndex()).toString());
  model->setLaserTopic(ui->laserScannerComboBox->itemData(ui->laserScannerComboBox->currentIndex()).toString());
  model->setOdomTopic(ui->odomtryComboBox->itemData(ui->odomtryComboBox->currentIndex()).toString());
  model->setCmdTopic(ui->cmdvelLineEdit->text());
  if(ui->nav1RadioButton->isChecked())
      model->setNavStrategy("nav1");
  else
      model->setNavStrategy("nav2");

  if(ui->logitechJoyRadioButton->isChecked())
      model->setJoyTypeStrategy("logitech");
  else
      model->setJoyTypeStrategy("xbox");

  return model;
}

void AddRobot::setModel(RobotModel *value)
{
  ui->nameLineEdit->setText(value->getName());
  ui->typeComboBox->setCurrentText(value->getType());

  ui->initPosXLineEdit->setText(value->getInitPosX());
  ui->initPosYLineEdit->setText(value->getInitPosY());
  ui->initPosZLineEdit->setText(value->getInitPosZ());


  ui->initRotXLineEdit->setText(value->getInitRotX());
  ui->initRotYLineEdit->setText(value->getInitRotY());
  ui->initRotZLineEdit->setText(value->getInitRotZ());

  int idx;
  QString rgb = value->getRgbTopic();

  idx = ui->rgbCameraComboBox->findText(value->getRgbTopic().replace(" ", "/"));
  if(idx!=-1)
    ui->rgbCameraComboBox->setCurrentIndex(idx);

  idx = ui->thermalCameraComboBox->findText(value->getThermalTopic().replace(" ", "/"));
  if(idx!=-1)
    ui->thermalCameraComboBox->setCurrentIndex(idx);

  idx = ui->laserScannerComboBox->findText(value->getLaserTopic().replace(" ", "/"));
  if(idx!=-1)
    ui->laserScannerComboBox->setCurrentIndex(idx);

  idx = ui->odomtryComboBox->findText(value->getOdomTopic().replace(" ", "/"));
  if(idx!=-1)
    ui->odomtryComboBox->setCurrentIndex(idx);

  if(value->getNavStrategy()=="nav1")
    ui->nav1RadioButton->setChecked(true);
  else
      ui->nav2RadioButton->setChecked(true);


  if(value->getJoyTypeStrategy()=="logitech")
    ui->logitechJoyRadioButton->setChecked(true);
  else
      ui->xboxJoyButton->setChecked(true);

  ui->cmdvelLineEdit->setText(value->getCmdTopic());
}

QSet<QString> AddRobot::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;

  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    QString topic = it->name.c_str();

    // add raw topic
    topics.insert(topic);

    if (message_types.contains(it->datatype.c_str()))
    {
      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
        }
      }
    }

    if (message_sub_types.contains(it->datatype.c_str()))
    {
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
      }
    }
  }
  return topics;
}
void AddRobot::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  ros::NodeHandle nodeHandle("~");
  image_transport::ImageTransport it(nodeHandle);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");

    ui->rgbCameraComboBox->addItem(label, QVariant(*it));
    ui->thermalCameraComboBox->addItem(label, QVariant(*it));
    ui->laserScannerComboBox->addItem(label, QVariant(*it));
    ui->odomtryComboBox->addItem(label, QVariant(*it));
  }
}

void AddRobot::getInitPositions()
{
    ros::NodeHandle nodeHandle("~");

    std::string odom_topic = ros::names::append("/"+ui->nameLineEdit->text().toStdString(),"/odom");
    map_odom =  nodeHandle.subscribe(odom_topic, 50, &AddRobot::odomCallback,this);


}

void AddRobot::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    ui->initPosXLineEdit->setText(QString::number(odom_msg->pose.pose.position.x,'f',2));
    ui->initPosYLineEdit->setText(QString::number(odom_msg->pose.pose.position.y,'f',2));
    ui->initPosZLineEdit->setText(QString::number(odom_msg->pose.pose.position.z,'f',2));

    ui->initRotXLineEdit->setText(QString::number(odom_msg->pose.pose.orientation.x,'f',2));
    ui->initRotYLineEdit->setText(QString::number(odom_msg->pose.pose.orientation.y,'f',2));
    ui->initRotZLineEdit->setText(QString::number(odom_msg->pose.pose.orientation.z,'f',2));

    map_odom.shutdown();
}
