#include "videostablell.h"

VideoStableLL::VideoStableLL(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);


	m_mainLayout = new QHBoxLayout(this);

	m_videoStabThread = new MeshFlowVS(this);

	setCentralWidget(m_videoStabThread);
	 
	connect(ui.actionStart,SIGNAL(triggered()),this,SLOT(start()));
	connect(ui.actionView, SIGNAL(triggered()), this, SLOT(viewVideo()));
}

VideoStableLL::~VideoStableLL()
{

}

void VideoStableLL::start()
{
	Mat	tempFrame = imread("1.jpg");
	QString video = QFileDialog::getOpenFileName(this, tr("select video to open"),
		"F:\\videoset\\Parallax\\Parallax");//获取视频完整路径
	if (video.isEmpty())
	{
		return;
	}
	string videoDir = video.toStdString();

	m_videoStabThread->startExe(videoDir);
}


void VideoStableLL::viewVideo()
{
	//QString video = QFileDialog::getOpenFileName(this, tr("select video to open"),
	//	"F:\\videoset\\Parallax\\Parallax");//获取视频完整路径
	//if (video.isEmpty())
	//{
	//	return;
	//}
	//string videoDir = video.toStdString();
	m_videoStabThread->viewV();
}