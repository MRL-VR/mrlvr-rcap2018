#include <QDebug>
#include "mrl_rqt_dashboard/bringup.h"
#include "mrl_rqt_dashboard/projecttheme.h"

BringUp::BringUp()
{

}

void BringUp::setApplicationFont(QApplication& app)
{

  QFontDatabase::addApplicationFont(":/resource/fonts/OpenSans-Light.ttf");
  QFontDatabase::addApplicationFont(":/resource/fonts/IranSans.ttf");
  QFontDatabase::addApplicationFont(":/resource/fonts/DroidNaskh-Regular.ttf");
  QFontDatabase::addApplicationFont(":/fonts/ScopeOne-Regular.ttf.ttf");

  qApp->setFont(QFont("Open Sans", 11));
//  qApp->setFont(QFont("IRAN Sans", 9));
//  qApp->setFont(QFont("Droid Arabic Naskh", 9));

}

void BringUp::setApplicationStyle(QApplication& app)
{

  QFile f(":qdarkstyle/style.qss");

  if (!f.exists())
    {
      qWarning()<<"Unable to set stylesheet, file not found\n";
    }
  else
    {
      f.open(QFile::ReadOnly | QFile::Text);
      QTextStream ts(&f);
      QString textFile = ts.readAll();

      loadApplicationTheme(textFile, true);

      app.setStyleSheet(textFile);
    }

//  app.setOverrideCursor(Qt::BlankCursor);

}

void BringUp::loadVirtualKeyboard()
{
  qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
}

void BringUp::loadApplicationTheme(QString &applicationTheme, const bool& isDark)
{
  if (isDark)
    {
      applicationTheme.replace(ProjectTheme::ColorText::PRIMERY_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::ACCENT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::ACCENT_COLD_NORMAL));

      applicationTheme.replace(ProjectTheme::ColorText::PRIMARY_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_TEXT_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::SECONDARY_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::SECONDARY_TEXT_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::DISABLED_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DISABLED_TEXT_DARK));

      applicationTheme.replace(ProjectTheme::ColorText::STATUS_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::STATUS_BAR_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::APP_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::APP_BAR_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::DIALOGS_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DIALOGS_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));


      applicationTheme.replace(ProjectTheme::ColorText::STATUS_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::STATUS_BAR_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::APP_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::DIALOGS_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));

      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_NORMAL_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_NORMAL_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_FOCUSED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_FOCUSED_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_PRESSED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_PRESSED_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_DISABLED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_DISABLED_DARK));

      applicationTheme.replace(ProjectTheme::ColorText::DIVIDER_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DIVIDER_DARK));

      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_1_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_2_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_3_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_4_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_5_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_6_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_DARK));

      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_1_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_2_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_3_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_4_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_5_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_6_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));


    }
  else
    {
      applicationTheme.replace(ProjectTheme::ColorText::PRIMERY_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_COLOR_NORMAL));
      applicationTheme.replace(ProjectTheme::ColorText::ACCENT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::ACCENT_COLD_NORMAL));

      applicationTheme.replace(ProjectTheme::ColorText::PRIMARY_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::PRIMARY_TEXT_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::SECONDARY_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::SECONDARY_TEXT_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::DISABLED_TEXT_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DISABLED_TEXT_LIGHT));

      applicationTheme.replace(ProjectTheme::ColorText::STATUS_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::STATUS_BAR_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::APP_BAR_COLOR_TEXT, getRGBA(ProjectTheme::Colors::APP_BAR_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::DIALOGS_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DIALOGS_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));

      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_NORMAL_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_NORMAL_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_FOCUSED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_FOCUSED_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_PRESSED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_PRESSED_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BUTTON_BACKGROUND_DISABLED_TEXT, getRGBA(ProjectTheme::Colors::BUTTON_BACKGROUND_DISABLED_LIGHT));

      applicationTheme.replace(ProjectTheme::ColorText::DIVIDER_COLOR_TEXT, getRGBA(ProjectTheme::Colors::DIVIDER_LIGHT));

      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_1_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_2_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_3_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_4_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_5_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
      applicationTheme.replace(ProjectTheme::ColorText::BACKGROUND_6_COLOR_TEXT, getRGBA(ProjectTheme::Colors::BACKGROUND_LIGHT));
    }
}
