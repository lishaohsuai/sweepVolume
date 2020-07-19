/********************************************************************************
** Form generated from reading UI file 'curveAndSurface.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CURVEANDSURFACE_H
#define UI_CURVEANDSURFACE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_curveAndSurfaceClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *curveAndSurfaceClass)
    {
        if (curveAndSurfaceClass->objectName().isEmpty())
            curveAndSurfaceClass->setObjectName(QString::fromUtf8("curveAndSurfaceClass"));
        curveAndSurfaceClass->resize(600, 400);
        menuBar = new QMenuBar(curveAndSurfaceClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        curveAndSurfaceClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(curveAndSurfaceClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        curveAndSurfaceClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(curveAndSurfaceClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        curveAndSurfaceClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(curveAndSurfaceClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        curveAndSurfaceClass->setStatusBar(statusBar);

        retranslateUi(curveAndSurfaceClass);

        QMetaObject::connectSlotsByName(curveAndSurfaceClass);
    } // setupUi

    void retranslateUi(QMainWindow *curveAndSurfaceClass)
    {
        curveAndSurfaceClass->setWindowTitle(QApplication::translate("curveAndSurfaceClass", "curveAndSurface", nullptr));
    } // retranslateUi

};

namespace Ui {
    class curveAndSurfaceClass: public Ui_curveAndSurfaceClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CURVEANDSURFACE_H
