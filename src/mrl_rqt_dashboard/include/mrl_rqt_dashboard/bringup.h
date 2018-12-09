#ifndef BRINGUP_H
#define BRINGUP_H

#include <QApplication>
#include <QFile>
#include <QFontDatabase>
#include <QTextStream>
#include <QColor>
#include <sstream>

class BringUp
{
public:
    BringUp();

    static void setApplicationFont(QApplication &app);

    static void setApplicationStyle(QApplication &app);

    static void loadVirtualKeyboard();

    static void loadApplicationTheme(QString& applicationTheme, const bool &isDark);

    inline static QString getRGBA(const QColor& color)
    {

        QString name = "rgba(" + QString::number(color.red()) + "," +
                QString::number(color.green()) + "," +
                QString::number(color.blue()) + "," +
                QString::number((int)(((double)(color.alpha()) / 255.0) * 100.0)) + "%)";

        int a = color.alpha();
        return name;
    }
};

#endif // BRINGUP_H
