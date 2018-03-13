#ifndef VIDEOSTABLELL_H
#define VIDEOSTABLELL_H

#include <QtWidgets/QMainWindow>
#include "ui_videostablell.h"
#include "MeshFlow.h"

#include <iostream>

#include <QFileDialog>

class VideoStableLL : public QMainWindow
{
	Q_OBJECT

public:
	VideoStableLL(QWidget *parent = 0);
	~VideoStableLL();

private:
	Ui::VideoStableLLClass ui;
	QHBoxLayout *m_mainLayout;

	MeshFlowVS *m_videoStabThread;

	public slots:
	void start();
	void viewVideo();
};

#endif // VIDEOSTABLELL_H
