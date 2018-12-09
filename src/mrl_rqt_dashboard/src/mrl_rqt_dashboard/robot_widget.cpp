
#include <mrl_rqt_dashboard/robot_widget.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <stdlib.h>
#include <QStackedLayout>
#include <QGridLayout>

namespace mrl_rqt_dashboard {

RobotWidget::RobotWidget(QWidget *parent) :
    QWidget(parent)
{
    frame = new QFrame(this);
    frame->setFrameStyle(QFrame::Box);

    QPalette* palette = new QPalette();
    palette->setColor(QPalette::Foreground,Qt::red);
    frame->setPalette(*palette);

    ui.setupUi(this);

    this->setVisible(true);
    connect(ui.joystickButton, SIGNAL(pressed()), this, SLOT(joystickbuttonClicked()));
    connect(ui.autonomousButton, SIGNAL(pressed()), this, SLOT(autonomousButtonClicked()));
    connect(ui.aliveButton,SIGNAL(pressed()),this,SLOT(aliveButtonClicked()));
    connect(ui.deadButton,SIGNAL(pressed()),this,SLOT(deadButtonClicked()));
}


RobotWidget::~RobotWidget()
{
    rProccess->terminate();
    rProccess->close();
    rProccess->kill();
}

void RobotWidget::init()
{
    ui.nameLabel->setText(model->getName());

    if(model->getType().contains("Quadrator"))
    {
        ui.typeWidget->setStyleSheet("image: url(:/icons/quadrator.png);");
    }
    ros::NodeHandle n;

    victim_pub = n.advertise<std_msgs::Int32>((getModel()->getName()+"/victim_marker").toUtf8().constData(), 1000);
    teleop_keyboard_pub = n.advertise<std_msgs::String>((getModel()->getName()+"/teleop").toUtf8().constData(), 1000);
    teleop_joy_pub = n.advertise<std_msgs::String>((getModel()->getName()+"/joy_control").toUtf8().constData(), 1000);

    explore_control_pub = n.advertise<std_msgs::String>((getModel()->getName()+"/explore/control").toUtf8().constData(), 1000);

}

void RobotWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    frame->resize(ui.verticalLayout->geometry().size());
    frame->move(ui.verticalLayout->geometry().topLeft());
}

void RobotWidget::joystickbuttonClicked()
{
    emit joystickButtonEvent(this);
}

void RobotWidget::autonomousButtonClicked()
{
    emit autonomousButtonEvent(this);
}

void RobotWidget::aliveButtonClicked()
{
    std_msgs::Int32 cmd;
    cmd.data = 1;
    victim_pub.publish(cmd);

}

void RobotWidget::deadButtonClicked()
{
    std_msgs::Int32 cmd;
    cmd.data = 2;
    victim_pub.publish(cmd);
}

int RobotWidget::getStatus() const
{
    return status;
}

void RobotWidget::setStatus(int value)
{
    status = value;
}

void RobotWidget::select(bool value)
{
    if(value)
    {
        this->setStyleSheet("background-color: rgb(1, 118, 9);color: rgb(255, 255, 255);");
        setStatus(JOSYTICK);

        
        std_msgs::String cmd;
        cmd.data = "start";
        teleop_keyboard_pub.publish(cmd);
        teleop_joy_pub.publish(cmd);

        std_msgs::String cmd2;
        cmd2.data = "stop";
        explore_control_pub.publish(cmd2);

        
    }
    else
    {
        this->setStyleSheet("");
         std_msgs::String cmd;
        cmd.data = "stop";
        teleop_keyboard_pub.publish(cmd);
        teleop_joy_pub.publish(cmd);

        setStatus(IDLE);
    }
    this->update();

}

void RobotWidget::autonomous(bool value)
{
    if(value)
    {
        this->setStyleSheet("background-color: rgb(2, 78, 255);color: rgb(255, 255, 255);");
        std_msgs::String cmd;
        cmd.data = "start";
        explore_control_pub.publish(cmd);
        std_msgs::String cmd2;
        cmd2.data = "stop";
        teleop_keyboard_pub.publish(cmd2);
        teleop_joy_pub.publish(cmd2);
        setStatus(AUTONOMOUS);
    }
    else
    {
        this->setStyleSheet("");
        setStatus(IDLE);
        std_msgs::String cmd;
        cmd.data = "stop";
        explore_control_pub.publish(cmd);

    }


    this->update();
}

RobotModel *RobotWidget::getModel() const
{
    return model;
}

void RobotWidget::setModel(RobotModel *value)
{
    model = value;
}

}

