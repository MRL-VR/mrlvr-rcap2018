#include "mrl_rqt_dashboard/cameraform.h"
#include "mrl_rqt_dashboard/ui_cameraform.h"
#include "mrl_rqt_dashboard/bringup.h"
#include "mrl_rqt_dashboard/image_view.h"
#include "mrl_rqt_dashboard/dualcamera.h"
#include <mrl_rqt_dashboard/robotmodel.h>

#include <std_msgs/String.h>
#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QListWidget>
#include <QProcess>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QStackedWidget>
#include <QSettings>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <math.h>
#include <stdlib.h>

CameraForm::CameraForm(QWidget *parent) :QDialog(parent),
    ui(new Ui::CameraForm)
{
    ui->setupUi(this);

    init();
    loadSettings();
}

CameraForm::~CameraForm()
{
    delete ui;
}


void CameraForm::init()
{
    ros::NodeHandle nodeHandle("~");
}
void CameraForm::loadSettings()
{

    QGridLayout* layout = new QGridLayout(this);


    QSettings settings;

    QJsonDocument jDoc;

    jDoc = QJsonDocument::fromBinaryData(settings.value("MRL_Settings").toByteArray());

    QJsonArray jArray = jDoc.array();

    int nRows = ceil(sqrt(jArray.count()));

    for (int i = 0; i < jArray.count(); ++i) {
        RobotModel *newModel = new RobotModel();
        newModel->setJsonObject(jArray[i].toObject());
        mrl_rqt_dashboard::DualCamera* camera = new mrl_rqt_dashboard::DualCamera(this);
        camera->setBotName(newModel->getName());
        camera->setBotType(newModel->getType());
        camera->setBotRgbTopic(newModel->getRgbTopic());
        camera->setBotThermalTopic(newModel->getThermalTopic());

        layout->addWidget(camera,(int)floor(i/nRows),(int)floor(i%nRows));

        camera->connect();
    }

}
