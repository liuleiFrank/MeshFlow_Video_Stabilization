#include "Mesh.h"

Quad::Quad()
{
	V00.x = 0.0; V00.y = 0.0;
	V01.x = 0.0; V01.y = 0.0;
	V10.x = 0.0; V10.y = 0.0;
	V11.x = 0.0; V11.y = 0.0;
}

Quad::Quad(const Quad &inQuad)
{
	V00.x = inQuad.V00.x; V00.y = inQuad.V00.y;
	V01.x = inQuad.V01.x; V01.y = inQuad.V01.y;
	V10.x = inQuad.V10.x; V10.y = inQuad.V10.y;
	V11.x = inQuad.V11.x; V11.y = inQuad.V11.y;
}

Quad::Quad(Point2f &inV00, const Point2f &inV01, const Point2f &inV10, const Point2f &inV11)
{
	V00.x = inV00.x; V00.y = inV00.y;
	V01.x = inV01.x; V01.y = inV01.y;
	V10.x = inV10.x; V10.y = inV10.y;
	V11.x = inV11.x; V11.y = inV11.y;
}

Quad::~Quad()
{

}

void Quad::operator=(const Quad &inQuad)
{
	V00.x = inQuad.V00.x; V00.y = inQuad.V00.y;
	V01.x = inQuad.V01.x; V01.y = inQuad.V01.y;
	V10.x = inQuad.V10.x; V10.y = inQuad.V10.y;
	V11.x = inQuad.V11.x; V11.y = inQuad.V11.y;
}

double Quad::getMinX() const
{
	float minx = min(V00.x, V01.x);
	minx = min(minx, V10.x);
	minx = min(minx, V11.x);

	return 1.0*minx;
}

double Quad::getMaxX() const
{
	float maxX = max(V00.x, V01.y);
	maxX = max(maxX, V10.x);
	maxX = max(maxX, V11.x);

	return 1.0*maxX;
}


double Quad::getMinY() const
{
	float minY = min(V00.y, V01.y);
	minY = min(minY, V10.y);
	minY = min(minY, V11.y);
	return 1.0*minY;
}

double Quad::getMaxY() const
{
	float maxY = max(V00.y, V01.y);
	maxY = max(maxY, V10.y);
	maxY = max(maxY, V11.y);

	return 1.0*maxY;
}




bool isPointInTriangular(const Point2f &pt, Point2f &V0, const Point2f &V1, const Point2f V2)
{
	double lambda1 = ((V1.y - V2.y)*(pt.x - V2.x) + (V2.x - V1.x)*(pt.y - V2.y)) / ((V1.y - V2.y)*(V0.x - V2.x) + (V2.x - V1.x)*(V0.y - V2.y));
	double lambda2 = ((V2.y - V0.y)*(pt.x - V2.x) + (V0.x - V2.x)*(pt.y - V2.y)) / ((V2.y - V0.y)*(V1.x - V2.x) + (V0.x - V2.x)*(V1.y - V2.y));

	if (lambda1 >= 0.0&&lambda1 <= 1.0&&lambda2 >= 0.0&&lambda2 <= 1.0)
	{
		return true;
	}
	else
	{
		return false;
	}
}


Mesh::Mesh()
{
	imgRows = 0;
	imgCols = 0;
	width = 0;
	height = 0;
}

Mesh::Mesh(const Mesh &inMesh)
{
	imgRows = inMesh.imgRows;
	imgCols = inMesh.imgCols;
	width = inMesh.width;
	height = inMesh.height;
	xMat = cv::Mat::zeros(height, width, CV_64FC1);
	yMat = cv::Mat::zeros(height, width, CV_64FC1);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			setVertex(i, j, inMesh.getVertex(i, j));
		}
	}
}

Mesh::Mesh(int rows, int cols)
{
	imgRows = rows;
	imgCols = cols;
	width = 0;
	height = 0;
}

Mesh::Mesh(int rows, int cols, double quadWidth, double quadHeight)
{
	imgRows = rows;
	imgCols = cols;
	buildMesh(quadWidth, quadHeight);
	this->quadWidth = quadWidth;
	this->quadHeight = quadHeight;
}

Mesh::~Mesh()
{
}

//将一个点通过单应矩阵变换为另外一个坐标点
Point2f matMyPointCVMat(const Point2f &pt, const Mat &H)
{
	Mat cvPt = Mat::zeros(3, 1, CV_64F);
	cvPt.at<double>(0, 0) = pt.x;
	cvPt.at<double>(1, 0) = pt.y;
	cvPt.at<double>(2, 0) = 1.0;

	Mat cvResult = H*cvPt;

	Point2f result;
	result.x = cvResult.at<double>(0, 0) / cvResult.at<double>(2, 0);
	result.y = cvResult.at<double>(1, 0) / cvResult.at<double>(2, 0);
	return result;
}


//将一个网格中的所有点通过单应矩阵变换
void Mesh::HomographyTransformation(const Mat &H)
{
	for (int i = 0; i <height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			Point2f pt = this->getVertex(i, j);
			Point2f ptTrans = matMyPointCVMat(pt, H);
			this->setVertex(i, j, ptTrans);
		}
	}
}


void Mesh::setVertex(int i, int j, const Point2f &pos)
{
	xMat.at<double>(i, j) = pos.x;
	yMat.at<double>(i, j) = pos.y;
}

Quad Mesh::getQuad(int i, int j) const
{
	Point2f V00;
	Point2f V01;
	Point2f V10;
	Point2f V11;


	V00 = getVertex(i - 1, j - 1);//网格左上角
	V01 = getVertex(i - 1, j);//网格左上角
	V10 = getVertex(i, j - 1);//网格右上角
	V11 = getVertex(i, j);//网格右下角

	Quad qd(V00, V01, V10, V11);
	return qd;
}




void Mesh::drawMesh(Mat &targetImg)
{
	Mat temp = targetImg.clone();
	Scalar color(255, 255, 255);
	int gap = 0;
	int lineWidth = 3;

	for (int i = 0; i <height; i++)
	{
		for (int j = 0; j <width; j++)
		{
			Point2f pUp = getVertex(i - 1, j);
			Point2f pLeft = getVertex(i, j - 1);
			Point2f pCur = getVertex(i, j);

			pUp.x += gap;
			pUp.y += gap;
			pLeft.x += gap;
			pLeft.y += gap;
			pCur.x += gap;
			pCur.y += gap;

			if (pUp.x > -9999.0 && pUp.y > -9999.0 && pCur.x > -9999.0 && pCur.y > -9999.0){
				double dis = sqrt((pUp.x - pCur.x)*(pUp.x - pCur.x) + (pUp.y - pCur.y)*(pUp.y - pCur.y));
			
				line(temp, cv::Point2f(pUp.x, pUp.y), cv::Point2f(pCur.x, pCur.y), color, lineWidth, CV_AA);
				
			}
			if (pLeft.x > -9999.0 && pLeft.y > -9999.0 && pCur.x > -9999.0 && pCur.y > -9999.0){
				double dis = sqrt((pLeft.x - pCur.x)*(pLeft.x - pCur.x) + (pLeft.y - pCur.y)*(pLeft.y - pCur.y));
			
				line(temp, cv::Point2f(pLeft.x, pLeft.y), cv::Point2f(pCur.x, pCur.y), color, lineWidth, CV_AA);
			
			}
			cv::circle(temp, cv::Point(pUp.x, pUp.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
			cv::circle(temp, cv::Point(pLeft.x, pLeft.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
			cv::circle(temp, cv::Point(pCur.x, pCur.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
		}
	}

	for (int i = 1; i < height; i++)
	{
		cv::Point2f pLeft = getVertex(i, 0);
		cv::Point2f pLeftUp = getVertex(i - 1, 0);

		pLeftUp.x += gap;
		pLeftUp.y += gap;
		pLeft.x += gap;
		pLeft.y += gap;

		if (pLeft.x > -9999.0 && pLeft.y > -9999.0 && pLeftUp.x > -9999.0 && pLeftUp.y > -9999.0){
			double dis = sqrt((pLeft.x - pLeftUp.x)*(pLeft.x - pLeftUp.x) + (pLeft.y - pLeftUp.y)*(pLeft.y - pLeftUp.y));
			
			line(temp, cv::Point2f(pLeft.x, pLeft.y), cv::Point2f(pLeftUp.x, pLeftUp.y), color, lineWidth, CV_AA);
			
		}
		cv::circle(temp, cv::Point(pLeftUp.x, pLeftUp.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
		cv::circle(temp, cv::Point(pLeft.x, pLeft.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
	}

	for (int j = 1; j < width; j++)
	{
		cv::Point2f pLeftUp = getVertex(0, j - 1);
		cv::Point2f pUp = getVertex(0, j);

		pLeftUp.x += gap;
		pLeftUp.y += gap;
		pUp.x += gap;
		pUp.y += gap;

		if (pLeftUp.x > -9999.0 && pLeftUp.y > -9999.0 && pUp.x > -9999.0 && pUp.y > -9999.0){
			double dis = sqrt((pLeftUp.x - pUp.x)*(pLeftUp.x - pUp.x) + (pLeftUp.y - pUp.y)*(pLeftUp.y - pUp.y));
			
			line(temp, cv::Point2f(pLeftUp.x, pLeftUp.y), cv::Point2f(pUp.x, pUp.y), color, lineWidth, CV_AA);
		
		}
		cv::circle(temp, cv::Point(pUp.x, pUp.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
		cv::circle(temp, cv::Point(pLeftUp.x, pLeftUp.y), lineWidth + 2, cv::Scalar(45, 57, 167), -1);
	}
	targetImg = (2.0 / 5 * targetImg + 3.0 / 5 * temp);
}



void meshWarpRemap(Mat &src, Mat dst, Mat &mapX, Mat mapY, Mesh &m1, Mesh &m2)
{
	int height = src.size().height;
	int width = src.size().width;

	vector<Point2f> source(4);
	vector<Point2f> target(4);
	Mat H;

	for (int i = 1; i < m1.height; i++)
	{
		for (int j = 1; j < m1.width; i++)
		{
			Quad s = m1.getQuad(i, j);
			Quad t = m2.getQuad(i, j);

			source[0] = s.V00;
			source[1] = s.V01;
			source[2] = s.V10;
			source[3] = s.V11;

			target[0] = t.V00;
			target[1] = t.V01;
			target[2] = t.V10;
			target[3] = t.V11;

			H = findHomography(source, target, 0);

			for (int ii = source[0].y; ii < source[3].y; ii++)
			{
				for (int jj = source[0].x; jj < source[3].x; jj++)
				{
					double x = 1.0*jj;
					double y = 1.0*ii;

					//矩阵相乘，
					double X = H.at<double>(0, 0)*x + H.at<double>(0, 1)*y + H.at<double>(0, 2)*1.0;
					double Y = H.at<double>(1, 0)*x + H.at<double>(1, 1)*y + H.at<double>(1, 2)*1.0;
					double W = H.at<double>(2, 0)*x + H.at<double>(2, 1)*y + H.at<double>(2, 2)*1.0;

					W = W ? 1.0 / W : 0;
					mapX.at<float>(ii, jj) = X*W;
					mapY.at<float>(ii, jj) = Y*W;
				}
			}
		}
	}
}

void myQuickSort(vector<float> &arr, int left, int right)
{
	int i = left, j = right;
	float tmp;
	float pivot = arr[(left + right) / 2];

	while (i <= j)
	{
		while (arr[i]<pivot)
			i++;
		while (arr[j]>pivot)
			j--;

		if (i <= j)
		{
			tmp = arr[i];
			arr[i] = arr[j];
			arr[j] = tmp;
			i++;
			j--;
		}

	}

	if (left < j)
		myQuickSort(arr, left, j);
	if (i < right)
		myQuickSort(arr, i, right);
}


//构建网格
void Mesh::buildMesh(double quadWidth, double quadHeight)
{
	vector<int> xSet;
	vector<int> ySet;

	for (int x = 0; imgCols - x > 0.5*quadWidth; x += quadWidth)
	{
		xSet.push_back(x);
	}

	//加上最后一列
	xSet.push_back(imgCols - 1);

	
	for (int y = 0; imgRows - y>0.5*quadHeight; y += quadHeight)
	{
		ySet.push_back(y);
	}

	ySet.push_back(imgRows - 1);
	

	width = xSet.size();//
	height = ySet.size();

	xMat.create(height, width, CV_64FC1);
	yMat.create(height, width, CV_64FC1);

	for (int y = 0; y <height; y++)
	{
		for (int x = 0; x <width; x++)
		{
			xMat.at<double>(y, x) = xSet[x];
			yMat.at<double>(y, x) = ySet[y];
		}
	}

}


Point2f Mesh::getVertex(int i, int j) const
{
	double x;
	double y;

	x = xMat.at<double>(i, j);
	y = yMat.at<double>(i, j);
	return Point2f(x, y);
}


Mat Mesh::getXMat()
{
	return xMat;
}

Mat Mesh::getYMat()
{
	return yMat;
}