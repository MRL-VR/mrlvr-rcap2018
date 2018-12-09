#include "mrl_rqt_dashboard/camerarobotwidget.h"
//#include "ui_camerarobotwidget.h"
#include <QMouseEvent>
#include <QGraphicsView>


CameraRobotWidget::CameraRobotWidget(QLabel* MainMonitor,QWidget *parent) :
  QWidget(parent),
//  ui(new Ui::CameraRobotWidget),
  mainMonitor(MainMonitor),
  isSelected(false)

{
//  ui->setupUi(this);

  QPalette Pal(palette());
  Pal.setColor(QPalette::Background, Qt::yellow);
//  ui->label->setAutoFillBackground(true);
//  ui->label->setPalette(Pal);
}

CameraRobotWidget::~CameraRobotWidget()
{
//  delete ui;
}

void CameraRobotWidget::setVictimDetected(bool isDetected)
{
//  ui->label->setText(originalText + (isDetected?" (Victim)":"") );
}


void CameraRobotWidget::SetVideo(QPixmap image)
{
//  ui->label_2->setPixmap(image.scaled(ui->label_2->size(), Qt::KeepAspectRatio));
  if (isSelected)
    mainMonitor->setPixmap(image.scaled(mainMonitor->size(), Qt::KeepAspectRatio));
}

void CameraRobotWidget::setRobotNumber(int robotNumber)
{
//  _robotnumber = robotNumber;
//  originalText = QString("Robot %1").arg(_robotnumber);
  setVictimDetected(false);
}

void CameraRobotWidget::setAsSelected(bool isSelect)
{
  isSelected = isSelect;
  QPalette Pal(palette());
//  switch (isSelect)
//  {
//  case true:
//    Pal.setColor(QPalette::Background, Qt::green);
//    break;
//  case false:
//    Pal.setColor(QPalette::Background, Qt::yellow);
//    break;
//  }
//  ui->label->setAutoFillBackground(true);
//  ui->label->setPalette(Pal);
}

void CameraRobotWidget::mouseReleaseEvent(QMouseEvent *event)
{
//  ImSelected(_robotnumber);
}
