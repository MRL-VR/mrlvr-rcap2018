#ifndef PROJECTTHEME
#define PROJECTTHEME

#include <QColor>
#include <QPixmap>
#include <QRgb>

namespace ProjectTheme {
  namespace Colors
  {

    static QColor getRGBA(const QString& colorText)
    {
      bool ok;
      QColor color;
      if(colorText.length() == 7)
        {
          color.setRgb(qRgb (colorText.mid(1, 2).toInt(&ok, 16), colorText.mid(3, 2).toInt(&ok, 16),
                        colorText.mid(5, 2).toInt(&ok, 16)));
        }
      else if (colorText.length() == 9)
        {
          color.setRgba(qRgba(colorText.mid(3, 2).toInt(&ok, 16), colorText.mid(5, 2).toInt(&ok, 16),
                        colorText.mid(7, 2).toInt(&ok, 16), colorText.mid(1, 2).toInt(&ok, 16)));
        }

      return color;
    }
    static const QColor PRIMARY_COLOR_DARK(getRGBA("#1C262F"));
    static const QColor PRIMARY_COLOR_NORMAL(getRGBA("#252F39"));
    static const QColor PRIMARY_COLOR_LIGHT(getRGBA("#1F2933"));

    static const QColor ACCENT_WARM_DARK(getRGBA("#BF360C"));
    static const QColor ACCENT_WARM_NORMAL(getRGBA("#ffe23229"));
    static const QColor ACCENT_WARM_LIGHT(getRGBA("#FF7043"));

    static const QColor ACCENT_COLD_DARK(getRGBA("#0277BD"));
    static const QColor ACCENT_COLD_NORMAL(getRGBA("#03A9F4"));
    static const QColor ACCENT_COLD_LIGHT(getRGBA("#4FC3F7"));

    static const QColor PRIMARY_TEXT_DARK(getRGBA("#FFFFFF"));
    static const QColor SECONDARY_TEXT_DARK(getRGBA("#B4FFFFFF"));
    static const QColor DISABLED_TEXT_DARK(getRGBA("#80FFFFFF"));

    static const QColor STATUS_BAR_DARK(getRGBA("#000000"));
    static const QColor APP_BAR_DARK(getRGBA("#212121"));
    static const QColor BACKGROUND_DARK(getRGBA("#303030"));
    static const QColor DIALOGS_DARK(getRGBA("#424242"));
    static const QColor BUTTON_BACKGROUND_NORMAL_DARK(/*ACCENT_COLD_NORMAL*/PRIMARY_COLOR_LIGHT);
    static const QColor BUTTON_BACKGROUND_FOCUSED_DARK(ACCENT_COLD_NORMAL.darker(112));
    static const QColor BUTTON_BACKGROUND_PRESSED_DARK(ACCENT_COLD_NORMAL.darker(110));
    static const QColor BUTTON_BACKGROUND_DISABLED_DARK(getRGBA("#404040"));
    static const QColor DIVIDER_DARK(getRGBA("#1EFFFFFF"));

    static const QColor PRIMARY_TEXT_LIGHT(getRGBA("#DD000000"));
    static const QColor SECONDARY_TEXT_LIGHT(getRGBA("#89000000"));
    static const QColor DISABLED_TEXT_LIGHT(getRGBA("#60000000"));

    static const QColor STATUS_BAR_LIGHT(getRGBA("#E0E0E0"));
    static const QColor APP_BAR_LIGHT(getRGBA("#F5F5F5"));
    static const QColor BACKGROUND_LIGHT(getRGBA("#FAFAFA"));
    static const QColor DIALOGS_LIGHT(getRGBA("#FFFFFF"));
    static const QColor BUTTON_BACKGROUND_NORMAL_LIGHT(getRGBA("#E0E0E0"));
    static const QColor BUTTON_BACKGROUND_FOCUSED_LIGHT(getRGBA("#D5D5D5"));
    static const QColor BUTTON_BACKGROUND_PRESSED_LIGHT(getRGBA("#D6D6D6"));
    static const QColor BUTTON_BACKGROUND_DISABLED_LIGHT(getRGBA("#DFDFDF"));
    static const QColor DIVIDER_LIGHT(getRGBA("#1E000000"));


    static const QColor DIVIDER_COLORED(getRGBA("#BDBDBD"));



    //    SCROLL_BAR_COLOR: %1A237E;
    //    SCROLL_BAR_BACKGROUND_COLOR: %0D47A1;



    //    DISABLED_COLOR: %9E9E9E;

  }

  namespace ColorText {

    static const QString PRIMERY_COLOR_TEXT = "%PRIMERY_COLOR";
    static const QString ACCENT_COLOR_TEXT = "%ACCENT_COLOR";


    static const QString PRIMARY_TEXT_COLOR_TEXT = "%PRIMARY_TEXT_COLOR";
    static const QString SECONDARY_TEXT_COLOR_TEXT = "%SECONDARY_TEXT_COLOR";
    static const QString DISABLED_TEXT_COLOR_TEXT = "%DISABLED_TEXT_COLOR";


    static const QString STATUS_BAR_COLOR_TEXT = "%STATUS_BAR_COLOR";
    static const QString APP_BAR_COLOR_TEXT = "%APP_BAR_COLOR";
    static const QString DIALOGS_COLOR_TEXT = "%DIALOGS_COLOR";
    static const QString BACKGROUND_COLOR_TEXT = "%BACKGROUND_COLOR";


    static const QString BUTTON_BACKGROUND_NORMAL_TEXT = "%BUTTON_BACKGROUND_NORMAL_COLOR";
    static const QString BUTTON_BACKGROUND_FOCUSED_TEXT = "%BUTTON_BACKGROUND_FOCUSED_COLOR";
    static const QString BUTTON_BACKGROUND_PRESSED_TEXT = "%BUTTON_BACKGROUND_PRESSED_COLOR";
    static const QString BUTTON_BACKGROUND_DISABLED_TEXT = "%BUTTON_BACKGROUND_DISABLED_COLOR";
    static const QString DIVIDER_COLOR_TEXT = "%DIVIDER_COLOR";


    static const QString BACKGROUND_1_COLOR_TEXT = "%BACKGROUND_1_COLOR";
    static const QString BACKGROUND_2_COLOR_TEXT = "%BACKGROUND_2_COLOR";
    static const QString BACKGROUND_3_COLOR_TEXT = "%BACKGROUND_3_COLOR";
    static const QString BACKGROUND_4_COLOR_TEXT = "%BACKGROUND_4_COLOR";
    static const QString BACKGROUND_5_COLOR_TEXT = "%BACKGROUND_5_COLOR";
    static const QString BACKGROUND_6_COLOR_TEXT = "%BACKGROUND_6_COLOR";

  }

  namespace Resource {
    static const QString ICON_ONLY_DAY          (":/res/icons/options_day_only");
    static const QString ICON_ONLY_NIGHT        (":/res/icons/options_night_only");
    static const QString ICON_DAY_AND_NIGHT     (":/res/icons/options_day_night");
    static const QString ICON_WIND              (":/res/icons/options_wind_only");
    static const QString ICON_NO_WIND           (":/res/icons/options_no_wind_only");
    static const QString ICON_WIND_AND_NO_WIND  (":/res/icons/options_wind_nowind");

    static const QString ICON_CHECKED           (":/res/icons/checked");
    static const QString ICON_ERROR             (":/res/icons/error");


    static const QString ICON_POWER             (":/res/icons/procedure_battery.svg");
    static const QString ICON_NORMAL            (":/res/icons/procedure_normal.svg");
    static const QString ICON_WAITING           (":/res/icons/procedure_waiting.svg");
    static const QString ICON_WARNING           (":/res/icons/procedure_warning.svg");
    static const QString ICON_BACKWASH          (":/res/icons/procedure_water.svg");



    static const QString ICON_PLAYBACK_SHUFFLE  (":/res/icons/playback_shuffle");
    static const QString ICON_PLAYBACK_LOOP     (":/res/icons/playback_loop");
    static const QString ICON_PLAYBACK_LOOP_ONE (":/res/icons/playback_loop_one");
    static const QString ICON_PLAYBACK_SEQUENCE (":/res/icons/playback_sequence");

    static const QString ICON_OVAL (":/res/icons/oval.svg");

    static const QString ICON_DIVIDER (":/res/icons/divider.svg");

  }
}


#endif // PROJECTTHEME

