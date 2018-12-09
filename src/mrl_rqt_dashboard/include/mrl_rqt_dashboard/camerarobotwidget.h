#ifndef CAMERAROBOTWIDGET_H
#define CAMERAROBOTWIDGET_H

#include <qlabel.h>
#include <QWidget>
#include <QMouseEvent>
#include <boost/signals2.hpp>
#include <QGraphicsView>

namespace Ui {
class CameraRobotWidget;
}

class CameraRobotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit CameraRobotWidget(QLabel* MainMonitor, QWidget *parent = 0);
  ~CameraRobotWidget();
  boost::signals2::signal<void (int)> ImSelected;
//  int _robotnumber = -1;
  void setAsSelected(bool isSelect);
  void setRobotNumber(int robotNumber);

  void SetVideo(QPixmap image);
  void setVictimDetected(bool isDetected);

private:
  Ui::CameraRobotWidget *ui;
  QLabel* mainMonitor;
  bool isSelected;
  QString originalText;

protected:
  void mouseReleaseEvent(QMouseEvent * event);
};

#endif // CAMERAROBOTWIDGET_H
