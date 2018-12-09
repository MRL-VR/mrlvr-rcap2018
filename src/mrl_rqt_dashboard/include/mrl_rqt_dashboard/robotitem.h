#ifndef ROBOTITEM_H
#define ROBOTITEM_H

#include <mrl_rqt_dashboard/ui_robotitem.h>
#include <mrl_rqt_dashboard/addrobot.h>
#include <mrl_rqt_dashboard/robotmodel.h>

#include <QWidget>
#include <QListWidgetItem>


#include <QString>
namespace Ui {
class RobotItem;
}

class RobotItem : public QWidget
{
  Q_OBJECT

public:
  explicit RobotItem(QWidget *parent = 0);
  ~RobotItem();

  void setParent(QListWidgetItem *value);
  RobotModel *getModel() const;
  void setModel(RobotModel *value);
  void update();

signals:
  void editButtonEvent(RobotItem *item);
  void removeButtonEvent(QListWidgetItem *item);

private slots:
  virtual void editButton_clicked();
  virtual void removeButton_clicked();
  virtual void checkBox_changed(bool value);
private:
  Ui::RobotItemWidget *ui;
  RobotModel *model;
  QListWidgetItem *parent;
};

#endif // ROBOTITEM_H
