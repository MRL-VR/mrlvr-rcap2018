/*
  Copyright 2016 Lucas Walter
*/
#ifndef MRL_RQT_DASHBOARD_PLUGIN_H
#define MRL_RQT_DASHBOARD_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include "mrl_rqt_dashboard/ui_dashboard.h"

namespace mrl_rqt_dashboard
{

class Dashboard
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  Dashboard();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::DashboardUi ui_;
  QWidget* widget_;
};
}  // namespace rqt_example_cpp
#endif  // MRL_RQT_DASHBOARD_PLUGIN_H
