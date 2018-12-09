/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mrl_rqt_dashboard/dualcamera.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <stdlib.h>
#include <QStackedLayout>
#include <QGridLayout>

namespace mrl_rqt_dashboard {

DualCamera::DualCamera(QWidget *parent) :
  QWidget(parent)
{
  ui.setupUi(this);

  this->setVisible(true);

  QGridLayout *layout = new QGridLayout(this);
  layout->setMargin(0);

  rgb_view = new mrl_rqt_dashboard::ImageView;
  layout->addWidget(rgb_view);

  thermal_view = new mrl_rqt_dashboard::ImageView(this);
  thermal_view->changeToolbar();
}

QString DualCamera::getBotThermalTopic() const
{
  return botThermalTopic;
}

void DualCamera::setBotThermalTopic(const QString &value)
{
  botThermalTopic = value;
}

void DualCamera::connect()
{
  rgb_view->setRobotName(botName);
  rgb_view->setRobotType(botType);
  rgb_view->setCameraTopic(botRgbTopic);
  thermal_view->setCameraTopic(botThermalTopic);
  rgb_view->connect();
  thermal_view->connect();
}

QString DualCamera::getBotType() const
{
  return botType;
}

void DualCamera::setBotType(const QString &value)
{
  botType = value;
}

QString DualCamera::getBotRgbTopic() const
{
  return botRgbTopic;
}

void DualCamera::setBotRgbTopic(const QString &value)
{
  botRgbTopic = value;
}

QString DualCamera::getBotName() const
{
  return botName;
}

void DualCamera::setBotName(const QString &value)
{
  botName = value;
}

void DualCamera::resizeEvent(QResizeEvent *event)
{
  QWidget::resizeEvent(event);

  QSize tSize(this->width()/4,this->height()/4);
  QPoint tPoint(this->width()-tSize.width(),this->height()-tSize.height());

  thermal_view->resize(tSize);
  thermal_view->move(tPoint);
}

}

