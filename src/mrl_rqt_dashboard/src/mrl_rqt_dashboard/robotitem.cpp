#include "mrl_rqt_dashboard/robotitem.h"
#include<stdlib.h>

RobotItem::RobotItem(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::RobotItemWidget)
{
  ui->setupUi(this);
  this->setVisible(true);

  connect(ui->editButton, SIGNAL(pressed()), this, SLOT(editButton_clicked()));
  connect(ui->removeButton, SIGNAL(pressed()), this, SLOT(removeButton_clicked()));
  connect(ui->robotCheckBox,SIGNAL(toggled(bool)),this,SLOT(checkBox_changed(bool)));
}

RobotItem::~RobotItem()
{
  delete ui;
}

void RobotItem::editButton_clicked()
{
  AddRobot *editForm = new AddRobot();

  editForm->setModel(model);
  if(editForm->exec())
  {
    setModel(editForm->getModel());
    emit editButtonEvent(this);
  }
}

void RobotItem::removeButton_clicked()
{
  emit removeButtonEvent(parent);
}

void RobotItem::checkBox_changed(bool value)
{
  model->setIsActive(value);
  emit editButtonEvent(this);
}

RobotModel *RobotItem::getModel() const
{
  return model;
}

void RobotItem::setModel(RobotModel *value)
{
  model = value;
  update();
}

void RobotItem::setParent(QListWidgetItem *value)
{
  parent = value;
}

void RobotItem::update()
{
  if(model!= NULL)
  {
    ui->botNameLabel->setText(model->getName());
    ui->botTypeLabel->setText(model->getType());
    ui->robotCheckBox->setChecked(model->getIsActive());
  }
}
