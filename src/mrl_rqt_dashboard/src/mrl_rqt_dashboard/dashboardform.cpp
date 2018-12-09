#include "mrl_rqt_dashboard/dashboardform.h"
#include "mrl_rqt_dashboard/ui_dashboardform.h"
#include "mrl_rqt_dashboard/bringup.h"
#include "mrl_rqt_dashboard/robot_widget.h"
#include "mrl_rqt_dashboard/setupform.h"
#include "mrl_rqt_dashboard/cameraform.h"
#include "mrl_rqt_dashboard/commons.h"
#include "rviz/config.h"
#include "rviz/yaml_config_reader.h"
#include "rviz/display.h"

#include <std_msgs/String.h>
#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QSettings>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <stdlib.h>
#include <QTimer>
#include <ros/package.h>

DashboardForm::DashboardForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DashboardForm)
{



    SetupForm *newForm = new SetupForm();

    newForm->exec();
    isMapLaunch = false;

    ui->setupUi(this);


    connect(ui->joystickButton,SIGNAL(pressed()),this,SLOT(joystickAllEvent()));
    connect(ui->autonomousButton,SIGNAL(pressed()),this,SLOT(autonomousAllEvent()));
    connect(ui->settingButton,SIGNAL(pressed()),this,SLOT(openSettingDialog()));
    connect(ui->cameraButton,SIGNAL(pressed()),this,SLOT(openCameraDialog()));
    connect(ui->mapButton,SIGNAL(pressed()),this,SLOT(openMapDialog()));

    window()->installEventFilter(this);
    init();

    loadSettings();
}


void DashboardForm::startJoystick()
{


}


void DashboardForm::openSettingDialog()
{
}

void DashboardForm::openCameraDialog()
{
    QRect rect = this->geometry();
    CameraForm *newForm = new CameraForm();
    newForm->move(rect.width(), rect.y());

    newForm->show();
}

void DashboardForm::openMapDialog()
{

    if(isMapLaunch==false)
    {
        bringupProcess = new QProcess();
        bringupProcess->start("gnome-terminal  --tab -e \"roslaunch mrl_navigation_v1 mrl_map_merge.launch\"");
        rviz_frame = new rviz::VisualizationFrame();

        rviz_frame->setSplashPath("");
        rviz_frame->setMenuBar(NULL);
        rviz_frame->setStatusBar(NULL);
        rviz_frame->setAutoFillBackground(false);
        //        rviz_frame
        rviz_frame->initialize();

        int robot_count = robots.length();
        std::string bring_up_path = ros::package::getPath("bring_up");


        QString rviz_file_path  =  QString(bring_up_path.c_str()) +"/rviz/map_merge_"+ QString::number(robot_count) +".rviz";

        rviz::YamlConfigReader reader;
        rviz::Config config = rviz::Config();

        reader.readFile(config,rviz_file_path);
        rviz_frame->load(config);


        QVBoxLayout *layout = new QVBoxLayout();

        layout->addWidget(rviz_frame);
        ui->rviz_frame->setLayout(layout);
        isMapLaunch = true;

    }
}

void DashboardForm::updateCounter()
{
    time_counter++;
    int min = time_counter / 60;
    int sec = time_counter % 60;

    QString mString = QString::number(min).rightJustified(2, '0');
    QString sString = QString::number(sec).rightJustified(2, '0');
    ui->timeLabel->setText("Time Spend: " + mString + ":" + sString);

}

DashboardForm::~DashboardForm()
{
    if (bringupProcess!= NULL)
    {
        bringupProcess->kill();
        bringupProcess->close();
    }

    delete ui;

}


void DashboardForm::init()
{
    ui->verticalLayout->setStretch(0,25);
    ui->verticalLayout->setStretch(1,100);

    timer = new QTimer(this);
    time_counter = 0;
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCounter()));
    timer->start(1000);


    ros::NodeHandle nodeHandle("~");



}

void DashboardForm::selectRobot(int r)
{
    if(robots.length()>r)
    {
        for (int i = 0; i < robots.size(); ++i) {
            if(robots.at(i)->getStatus()!=AUTONOMOUS)
            {
                robots.at(i)->select(false);
            }
        }
        robots[r]->autonomous(false);
        robots[r]->select(true);
        selectedRobot = r;
    }
}

bool DashboardForm::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::KeyPress)
    {
        QKeyEvent *ke = static_cast<QKeyEvent *>(event);\

        int keyValue = ke->key();

        switch (keyValue) {
        case Qt::Key_0:
            selectRobot(9);

            break;
        case Qt::Key_1:
            selectRobot(0);

            break;
        case Qt::Key_2:
            selectRobot(1);

            break;
        case Qt::Key_3:
            selectRobot(2);

            break;
        case Qt::Key_4:
            selectRobot(3);

            break;
        case Qt::Key_5:
            selectRobot(4);

            break;
        case Qt::Key_6:
            selectRobot(5);

            break;
        case Qt::Key_7:
            selectRobot(6);

            break;
        case Qt::Key_8:
            selectRobot(7);

            break;
        case Qt::Key_9:
            selectRobot(8);

            break;
        }
        return true;
    }
    return QObject::eventFilter(obj,event);

}


void DashboardForm::loadSettings()
{
    QSettings settings;

    QJsonDocument jDoc;

    jDoc = QJsonDocument::fromBinaryData(settings.value("MRL_Settings").toByteArray());

    QJsonArray jArray = jDoc.array();
    int nRows = 1;
    QString cmd="gnome-terminal";

    for (int i = 0; i < jArray.count(); ++i) {
        RobotModel *newModel = new RobotModel();

        newModel->setJsonObject(jArray[i].toObject());
        mrl_rqt_dashboard::RobotWidget* robot = new mrl_rqt_dashboard::RobotWidget;
        robot->setModel(newModel);
        robot->init();


        connect(robot, SIGNAL(joystickButtonEvent(mrl_rqt_dashboard::RobotWidget*)), this, SLOT(joystickSelectEvent(mrl_rqt_dashboard::RobotWidget*)));
        connect(robot, SIGNAL(autonomousButtonEvent(mrl_rqt_dashboard::RobotWidget*)), this, SLOT(autonomousSelectEvent(mrl_rqt_dashboard::RobotWidget*)));

        ui->topLayout->addWidget(robot,(int)floor(i%nRows),(int)floor(i/nRows));

        robots.append(robot);
        if(newModel->getType()=="Pioneer3D")
        {
                cmd+=" --tab  -e \"roslaunch bring_up p3at_configure_v1.launch joy_type:="+newModel->getJoyTypeStrategy() +" robot_name:="+newModel->getName()+ " init_x:="+newModel->getInitPosX()+ " init_y:="+newModel->getInitPosY()+"\"";

        }


    }
    spawnProcess = new QProcess();
    spawnProcess->start(cmd);


}

void DashboardForm::joystickSelectEvent(mrl_rqt_dashboard::RobotWidget *item)
{
    for (int i = 0; i < robots.size(); ++i) {
        if(robots.at(i)==item)
        {

            robots.at(i)->autonomous(false);
            robots.at(i)->select(true);
            selectedRobot = i;
        }
        else
        {
            if(robots.at(i)->getStatus()!=AUTONOMOUS)
            {
                robots.at(i)->select(false);
            }
        }
    }
}



void DashboardForm::autonomousSelectEvent(mrl_rqt_dashboard::RobotWidget *item)
{
    item->autonomous(true);
}

void DashboardForm::joystickAllEvent()
{

    for (int i = 0; i < robots.size(); ++i) {
        robots.at(i)->autonomous(false);
    }
}

void DashboardForm::autonomousAllEvent()
{
    for (int i = 0; i < robots.size(); ++i) {
        robots.at(i)->autonomous(true);
    }
}
