/********************************************************************************
** Form generated from reading UI file 'my_plugin.ui'
**
** Created: Wed Dec 30 17:16:32 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MY_PLUGIN_H
#define UI_MY_PLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyPluginWidget
{
public:

    void setupUi(QWidget *MyPluginWidget)
    {
        if (MyPluginWidget->objectName().isEmpty())
            MyPluginWidget->setObjectName(QString::fromUtf8("MyPluginWidget"));
        MyPluginWidget->resize(400, 300);

        retranslateUi(MyPluginWidget);

        QMetaObject::connectSlotsByName(MyPluginWidget);
    } // setupUi

    void retranslateUi(QWidget *MyPluginWidget)
    {
        MyPluginWidget->setWindowTitle(QApplication::translate("MyPluginWidget", "Formulaic", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MyPluginWidget: public Ui_MyPluginWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MY_PLUGIN_H
