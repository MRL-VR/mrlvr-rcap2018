/*
  Copyright 2016 Lucas Walter
*/

#include "mrl_rqt_dashboard/dashboard.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QLayout>
#include <QLabel>
#include "ros/ros.h"


namespace mrl_rqt_dashboard
{

Dashboard::Dashboard()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("Dashboard");

}

void Dashboard::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments

//    QStringList args = context.argv();
//    char *argv[args.size()];
//    int argc = args.size();
//    int i =0;

//    foreach(QString s, args)
//    {
//        argv[i] = s.toLocal8Bit().data();
//        i++;
//    }

//    ros::init(argc,argv , "mrl_rqt_dashboard");


    // create QWidget
    widget_ = new QWidget();


    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    widget_->setWindowTitle("Dashboard");
    // add widget to the user interface
    context.addWidget(widget_);

}

void Dashboard::shutdownPlugin()
{
    // unregister all publishers here
}

void Dashboard::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                             qt_gui_cpp::Settings& instance_settings) const
{
    // instance_settings.setValue(k, v)
}

void Dashboard::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                const qt_gui_cpp::Settings& instance_settings)
{
    // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace mrl_rqt_dashboard
PLUGINLIB_DECLARE_CLASS(mrl_rqt_dashboard, Dashboard, mrl_rqt_dashboard::Dashboard, rqt_gui_cpp::Plugin)
