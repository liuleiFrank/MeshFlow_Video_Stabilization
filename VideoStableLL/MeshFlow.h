

#include <QtWidgets/QMainWindow>
#include "ui_videostablell.h"
#include <QFile>
#include <QDir>
#include <QtOpenGL/QtOpenGL>
#include <QtWidgets>
#include <QTextStream>
#include <QList>
#include <QWidget>  
#include <QGLWidget>  
#include<gl/glu.h>

#include <list>

#include "Mesh.h"

using namespace std;
using namespace cv;


class MeshFlowVS : public  QGLWidget
{
	Q_OBJECT

public:
	MeshFlowVS(QWidget *parent = 0);
	~MeshFlowVS();

private:
	bool fullscreen;                                //�Ƿ�ȫ����ʾ  

	GLfloat m_xRot;                                 //��x����ת�ĽǶ�  
	GLfloat m_yRot;                                 //��y����ת�ĽǶ�  
	GLfloat m_zRot;                                 //��z����ת�ĽǶ�  

	QString m_FileName;                             //ͼƬ��·�����ļ���  
	GLuint m_Texture;                               //����һ������ 
	vector<GLuint> vTexture;                        //�洢16*16������

	GLuint texture[1];
	Mat m_currentFrame;

	QImage tex1, tex2, buf1, buf2;
	bool m_ProcessFlag;


	vector<vector<Point2f>> localFeatures;
	vector<Point2f> glFeatures;

	vector<vector<Point2f>> localFeatures2;
	vector<Point2f> glFeatures2;

	vector<Quad> vquad;
	vector<Mat> homographV;//�洢ÿ֡��ȫ�ֵ�Ӧ����
	vector<Point2f> originPath;//�������������е�C(t),����v(t)�Ķ�v(t)����ÿ֡���˶�����
	vector<Point2f> optimizePath;

	vector<Mat> frameV; //�洢���40����Ƶ֡
	Mesh* m_mesh;

	vector<vector<Point2f>> originPathV; //���ڴ洢ÿһ֡ʱ��ԭʼ·������Ҫ��ȷ�������СӦ����FrameVʱ�̱�����ͬ
	vector<vector<Point2f>> optimizePathV;//���ڴ洢һ�ε��������л�õĻ�����������֡���Ż�·��
	vector<vector<vector<Point2f>>> optimizationPathVV;//�洢���е�����������õ��Ż���·��
	vector<Mat>  subImages;

	int m_width;
	int m_height;
	int subImageWidth;
	int subImageHeight;

	Mat xMat;
	Mat yMat;
	Mat m_globalHomography;
	//������˶�
	vector<Point2f> m_vertexMotion;
	Mesh* m_warpedemesh;

	int m_quadWidth, m_quadHeight;//quad
	int m_meshHeight, m_meshwidth;//mesh


	int m_xGridNum;
	int m_yGridNum;

	int iter;
	vector<Mat> subimagt;
	vector<Point2f> updatet;

	vector<Quad> paintQuad;
	vector<Quad> originQuad;

	

	int m_imagePos;

	QString m_saveName;
	string imageSavepath;
	int frameNum;
	string videoname;
	

	Mat subImageX;
	Mat subImageY;

	Mat subgray;

	Mat xMat2;
	Mat yMat2;


	vector<Mat> homos;

	vector<double> b;

	vector<double> sV;

	int num;
	Mat sum;
	int matSize;

	string savevideopath;
	string videopath1;



protected:
	//��3�����麯�����ض���  
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void loadGLTextures();

	float pixel2GLCoor(float pCoor, float glCoor, bool textureCoorFlag = true, bool xFlag = true, double ratio = 1);

public:
	void startExe(string videopath);
	void initOriginPath();
	Point2f Trans(cv::Mat H, cv::Point2f pt);
	void ComputeMotionbyFeature(vector<Point2f> &spt, vector<Point2f> &tpt);
	void computeOriginPath();
	Point2f getVertex(int i, int j);
	void DistributeMotion2MeshVertexes_MedianFilter();
	void SpatialMedianFilter();
	vector<Quad> getQuad();
	void Frame2subFrames(Mat m_frame, vector<Mat> &frames);
	void DetectFeatures(Mat m_frame);
	void initMeshFlow(int width, int height);
	void local2glFratures(vector<Point2f> &glFeatures, vector<vector<Point2f> > lcFeatures);
	Point2f TransPtbyH(Mat H, Point2f &pt);
	double calcTv(Mat Ft);
	double calcuFa(Mat Ft);
	double predictLamda(double Tv, double Fa);
	double calcuLamda(Mat Ft);
	double calcuGuassianWeight(int r, int t, int Omegat);
	void imshowMany(const std::string& _winName, const vector<Mat>& _imgs);
	void initOptPath();
	void setTemporSmotthRange(int &beginframe, int &endframe, int t);
	void getPathT(int t, vector<Point2f> &optedPath);
	void ViewSynthesis(int t, vector<Point2f> optPath, vector<Point2f> Ct);
	void Jacobicompute();
	void Jacobicompute2(int index);
	void addFeatures();
	void removefilesindir(const QString& path);
	void image2Video();

	int getSubIndex(Point2f p);

	void CreateShapepreCons(int k);

	
	void getWeight(Point2f V1, Point2f V2, Point2f V3, double &u, double &v, double &s);

	void computerHomos(Mat& img1, Mat& img2, int& w, int& h, int& index, int num, bool *isCancel);

	void compute(int k, int matSize, int matrows, vector<double> b, Mat sum, Mesh* m1, Mesh *m2);
	void getHomos(Mesh *m);

	Point2f getPoint(int x, int y);

	//�ڶ�����ֵ�˲�
	void SpatialMedianFilter2();

	void viewV();
    
};


