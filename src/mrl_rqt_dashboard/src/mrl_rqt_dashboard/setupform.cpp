#include "mrl_rqt_dashboard/setupform.h"
#include "mrl_rqt_dashboard/ui_setupform.h"
#include "mrl_rqt_dashboard/bringup.h"
#include "mrl_rqt_dashboard/addrobot.h"

#include <std_msgs/String.h>
#include <stdlib.h>
#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QListWidget>
#include <QProcess>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableView>
#include <QSettings>
#include <QVariant>
#include <QJsonArray>
#include <QJsonValue>
#include <QJsonDocument>
#include <QCloseEvent>

SetupForm::SetupForm(QWidget *parent) : QDialog(parent),
  ui(new Ui::SetupForm)
{
  ui->setupUi(this);

  init();

  connect(ui->newRobotButton, SIGNAL(pressed()), this, SLOT(openAddRobotDialog()));
  this->setWindowFlags(Qt::Tool);
  loadSettings();
}

void SetupForm::editItemsEvent(RobotItem *item)
{
  saveSettings();
}

void SetupForm::removeItemsEvent(QListWidgetItem *item)
{
  delete item;
  saveSettings();
}

SetupForm::~SetupForm()
{    
  delete ui;
}

void SetupForm::init()
{

//  ros::NodeHandle nodeHandle("~");
}

void SetupForm::openAddRobotDialog()
{
  AddRobot *newForm = new AddRobot();

  if(newForm->exec())
  {
    if(!newForm->getModel()->getName().isEmpty())
    {
      RobotItem *newItem = new RobotItem();
      newItem->setModel(newForm->getModel());

      connect(newItem, SIGNAL(editButtonEvent(RobotItem*)), this, SLOT(editItemsEvent(RobotItem*)));
      connect(newItem, SIGNAL(removeButtonEvent(QListWidgetItem*)), this, SLOT(removeItemsEvent(QListWidgetItem*)));

      QListWidgetItem* wItem = new QListWidgetItem(ui->robotListWidget);
      wItem->setSizeHint(newItem->size());

      ui->robotListWidget->setItemWidget(wItem, newItem);
      newItem->setParent(wItem);

      ui->robotListWidget->addItem(wItem);
      editItemsEvent(newItem);
    }
  }
}

void SetupForm::saveSettings()
{
  QSettings settings;
  QJsonArray jArray;
  QJsonDocument jDoc;

  int c = ui->robotListWidget->count();

  for (int i = 0; i < c; ++i) {
    RobotItem *item =  (RobotItem*)ui->robotListWidget->itemWidget(ui->robotListWidget->item(i));
    RobotModel *model = item->getModel();
    jArray.append(QJsonValue(model->getJsonObject()));
  }

  jDoc.setArray(jArray);
  settings.clear();
  settings.setValue("MRL_Settings",jDoc.toBinaryData());
}

void SetupForm::loadSettings()
{

  QSettings settings;

  QJsonDocument jDoc;

  jDoc = QJsonDocument::fromBinaryData(settings.value("MRL_Settings").toByteArray());

  QJsonArray jArray = jDoc.array();

  for (int i = 0; i < jArray.count(); ++i) {
    RobotModel *newModel = new RobotModel();
    newModel->setJsonObject(jArray[i].toObject());

    RobotItem *newItem = new RobotItem();
    newItem->setModel(newModel);
    connect(newItem, SIGNAL(editButtonEvent(RobotItem*)), this, SLOT(editItemsEvent(RobotItem*)));
    connect(newItem, SIGNAL(removeButtonEvent(QListWidgetItem*)), this, SLOT(removeItemsEvent(QListWidgetItem*)));

    QListWidgetItem* wItem = new QListWidgetItem(ui->robotListWidget);
    wItem->setSizeHint(newItem->size());

    ui->robotListWidget->setItemWidget(wItem, newItem);
    newItem->setParent(wItem);

    ui->robotListWidget->addItem(wItem);
  }

}



