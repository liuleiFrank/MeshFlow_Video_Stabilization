#include <vector>
#include <iostream>
#include <time.h>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

#ifndef __QUAD__
#define __QUAD__

//用于表示网格的四个角的坐标
class Quad
{
public:
	Point2f V00;//左上角
	Point2f V01;//左下角
	Point2f V10;//右上角
	Point2f V11;//右下角

	Quad();
	Quad(const Quad &inQuad);//复制构造函数
	Quad(Point2f &inV00, const Point2f &inV01, const Point2f &inV10, const Point2f &inV11);//fz构造函数

	~Quad();

	//重载
	void operator=(const Quad &inQuad);

	//
	double getMinX() const;
	double getMaxX() const;
	double getMinY() const;
	double getMaxY() const;

	bool isPointIn(const Point2f &pt) const;
	bool getBilinearCoordinates(const Point2f &pt, vector<double> &coefficient) const;
	bool getBilinearCoordinates(const Point2f &pt, double* &coefficient) const;

	inline void printfQuad()
	{
		printf("V00=%f %f\n", V00.x, V00.y);
		printf("V01=%f %f\n", V01.x, V01.y);
		printf("V10=%f %f\n", V10.x, V10.y);
		printf("V11=%f %f\n", V11.x, V11.y);
	}

	Point2f getPointByBilinearCoordinates(const vector<double> &coefficient) const;

};

#endif

#ifndef __MESH__
#define __MESH__

class Mesh
{
public:
	Mesh();
	Mesh(const Mesh &inMesh);//复制构造函数
	Mesh(int rows, int cols);
	Mesh(int rows, int cols, double quadWidth, double quadHeight);

	~Mesh();

	int imgRows; //视频帧的高度
	int imgCols;//视频帧的宽度

	int width;
	int height;

	int quadWidth;
	int quadHeight;

	void operator=(const Mesh &inMesh);
	double differemtFrom(const Mesh &inMesh) const;

	//网格中的某一个点
	Point2f getVertex(int i, int j)const;
	void setVertex(int i, int j, const Point2f &pos);

	void initialize(int w, int h);
	void buildMesh(double quadWidth, double quadHeight);
	void drawMesh(Mat &targetImg);
	bool selfCheck();
	void smoothMesh();

	void HomographyTransformation(const Mat &H);

	Quad getQuad(int i, int j) const;

	Mat getXMat();
	Mat getYMat();


private:
	Mat xMat;
	Mat yMat;
};


#endif
bool isPointInTriangular(const Point2f &pt, const Point2f &V0, const Point2f &V1, const Point2f &v2);
void meshWarp(const Mat src, Mat dst, Mesh &m1, const Mesh &m2);
void quadWarp(const Mat src, Mat dst, const Quad &q1, const Quad &q2);
void meshWarp_multicore(const Mat src, Mat dst, const Mesh &m1, const Mesh &m2);
void meshWarpRemap(Mat &src, Mat &dst, Mat &mapX, Mat &mapY, Mesh &m1, Mesh &m2);
void myQuickSort(vector<float> &arr, int left, int right);


