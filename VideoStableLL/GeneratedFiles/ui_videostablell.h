/********************************************************************************
** Form generated from reading UI file 'videostablell.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VIDEOSTABLELL_H
#define UI_VIDEOSTABLELL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_VideoStableLLClass
{
public:
    QAction *actionStart;
    QAction *actionView;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *VideoStableLLClass)
    {
        if (VideoStableLLClass->objectName().isEmpty())
            VideoStableLLClass->setObjectName(QStringLiteral("VideoStableLLClass"));
        VideoStableLLClass->resize(840, 617);
        actionStart = new QAction(VideoStableLLClass);
        actionStart->setObjectName(QStringLiteral("actionStart"));
        actionView = new QAction(VideoStableLLClass);
        actionView->setObjectName(QStringLiteral("actionView"));
        centralWidget = new QWidget(VideoStableLLClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        VideoStableLLClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(VideoStableLLClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 840, 26));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        VideoStableLLClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(VideoStableLLClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        VideoStableLLClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(VideoStableLLClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        VideoStableLLClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionStart);

        retranslateUi(VideoStableLLClass);

        QMetaObject::connectSlotsByName(VideoStableLLClass);
    } // setupUi

    void retranslateUi(QMainWindow *VideoStableLLClass)
    {
        VideoStableLLClass->setWindowTitle(QApplication::translate("VideoStableLLClass", "VideoStableLL", 0));
        actionStart->setText(QApplication::translate("VideoStableLLClass", "Open", 0));
        actionView->setText(QApplication::translate("VideoStableLLClass", "view", 0));
        menuFile->setTitle(QApplication::translate("VideoStableLLClass", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class VideoStableLLClass: public Ui_VideoStableLLClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIDEOSTABLELL_H
