#include "MeshFlow.h"
#include "Mesh.h"
#include <string>
#include <io.h>
#include <direct.h>    //头文件  
#include <omp.h>
#include <time.h>

vector<Mat> v;
#define meshcount 16
#define phi 40
#define mesh 16
#define subImageCount 4
#define Radius 50
#define neighborsnum 20
#define iternum 20

using namespace std;
clock_t start, finish;
double duration;

struct node
{
	vector<Point2f> features;
	vector<Point2f> motions;
};


node n;



void MeshFlowVS::initOriginPath()
{//原始路径由网格坐标点的运动矢量相加而成
	for (int i = 0; i <mesh + 1; i++)
	{
		for (int j = 0; j < mesh + 1; j++)
		{
			Point2f p(0, 0);
			originPath.push_back(p);
		}
	}
	originPathV.push_back(originPath);
}


//
Point2f MeshFlowVS::Trans(cv::Mat H, cv::Point2f pt){
	cv::Point2f result;

	double a = H.at<double>(0, 0) * pt.x + H.at<double>(0, 1) * pt.y + H.at<double>(0, 2);
	double b = H.at<double>(1, 0) * pt.x + H.at<double>(1, 1) * pt.y + H.at<double>(1, 2);
	double c = H.at<double>(2, 0) * pt.x + H.at<double>(2, 1) * pt.y + H.at<double>(2, 2);

	result.x = a / c;
	result.y = b / c;

	return result;
}


//在这里，网格坐标点的个数是17*17,在第一次中值滤波时可能会有所不同
void MeshFlowVS::computeOriginPath()
{
	//#pragma omp parallel for
	for (int i = 0; i < originPath.size(); i++)
	{
		originPath[i].x += m_vertexMotion[i].x;
		originPath[i].y += m_vertexMotion[i].y;
	}
	originPathV.push_back(originPath);
}


Point2f MeshFlowVS::getVertex(int i, int j)
{
	double x;
	double y;

	x = xMat.at<double>(i, j);
	y = yMat.at<double>(i, j);
	return Point2f(x, y);
}


//这个函数只有从第二帧才可以正常使用，利用中值滤波的方法，将特征点的运动传播到坐标点上，相当于论文中的f1
void MeshFlowVS::DistributeMotion2MeshVertexes_MedianFilter()
{
	vector<vector<float>> motionx, motiony;
	motionx.resize(m_meshHeight*m_meshwidth);
	motiony.resize(m_meshHeight*m_meshwidth);

	#pragma omp parallel for
	for (int i = 0; i < m_meshHeight; i++)
	{
		for (int j = 0; j < m_meshwidth; j++)
		{
			//获取坐标点的坐标
			Point2f pt = getVertex(i, j);
			//找出该坐标周围的特征点,不一定正确，也有可能是根据特征点坐标找出网格坐标
			for (int k = 0; k < n.features.size(); k++)
			{
				Point2f pt2 = n.features[k];
				//计算两者之间的距离
				float dis = sqrt((pt.x - pt2.x)*(pt.x - pt2.x) + (pt.y - pt2.y)*(pt.y - pt2.y));

				if (dis < 50)
				{
					motionx[i*m_meshwidth + j].push_back(n.motions[k].x);
					motiony[i*m_meshwidth + j].push_back(n.motions[k].y);
				}
			}
		}
	}




	//中值选择
  //  #pragma omp parallel for
	for (int i = 0; i < m_meshHeight; i++)
	{
		//#pragma omp parallel for
		for (int j = 0; j <m_meshwidth; j++)
		{
			if (motionx[i*m_meshwidth + j].size()>1)
			{

				//快速对x,y方向的运动进行排序
				myQuickSort(motionx[i*m_meshwidth + j], 0, motionx[i*m_meshwidth + j].size() - 1);
				myQuickSort(motiony[i*m_meshwidth + j], 0, motiony[i*m_meshwidth + j].size() - 1);

				//取中位值
				m_vertexMotion[i*m_meshwidth + j].x = motionx[i*m_meshwidth + j][motionx[i*m_meshwidth + j].size() / 2];
				m_vertexMotion[i*m_meshwidth + j].y = motiony[i*m_meshwidth + j][motiony[i*m_meshwidth + j].size() / 2];
			}
		}
	}
}



//第二次中值滤波
void MeshFlowVS::SpatialMedianFilter()
{
	//由于网格的个数是16*16,所以坐标是17*17
	vector<Point2f> tempVertexMotion(m_meshHeight*m_meshwidth);
	for (int i = 0; i < m_meshHeight; i++)
	{
		for (int j = 0; j <m_meshwidth; j++)
		{
			tempVertexMotion[i*m_meshwidth + j] = m_vertexMotion[i*m_meshwidth + j];
		}
	}


	//这个地方有问题
	int radius = 5;
	for (int i = 0; i < m_meshHeight; i++)
	{
		for (int j = 0; j < m_meshwidth; j++)
		{
			vector<float> motionx;
			vector<float> motiony;

			//k从-5到5
			for (int k = -radius; k <= radius; k++)
			{
				//l从-5到5
				for (int l = -radius; l <= radius; l++)
				{
					if (k >= 0 && k < m_meshHeight&&l >= 0 && l < m_meshwidth)
					{
							
						motionx.push_back(tempVertexMotion[i*m_meshwidth + j].x);
						motiony.push_back(tempVertexMotion[i*m_meshwidth + j].y);

					}
				}
			}

			myQuickSort(motionx, 0, motionx.size() - 1);
			myQuickSort(motiony, 0, motiony.size() - 1);

			m_vertexMotion[i*m_meshwidth + j].x = motionx[motionx.size() / 2];
			m_vertexMotion[i*m_meshwidth + j].y = motiony[motiony.size() / 2];

		}
	}

}


//第二次中值滤波
void MeshFlowVS::SpatialMedianFilter2()
{
	//由于网格的个数是16*16,所以坐标是17*17
	vector<Point2f> tempVertexMotion(m_meshHeight*m_meshwidth);
	for (int i = 0; i < m_meshHeight; i++)
	{
		for (int j = 0; j <m_meshwidth; j++)
		{
			tempVertexMotion[i*m_meshwidth + j] = m_vertexMotion[i*m_meshwidth + j];
		}
	}



	int radius = 10;
	#pragma omp parallel for
	for (int i = 0; i <= m_meshHeight-radius; i++)
	{
      //  #pragma omp parallel for
		for (int j = 0; j <= m_meshwidth-radius; j++)
		{
			vector<float> motionx;
			vector<float> motiony;

			for (int k = 0; k < radius; k++)
			{
				int m = (i+k) * 17 + j;
	
				for (int l = 0; l < radius; l++)
				{
					
					int t = m + l;
					if (t < tempVertexMotion.size())
					{
						motionx.push_back(tempVertexMotion[t].x);
						motiony.push_back(tempVertexMotion[t].y);
					}
				}
			}

		


			myQuickSort(motionx, 0, motionx.size() - 1);
			myQuickSort(motiony, 0, motiony.size() - 1);

			if (j == m_meshwidth - radius)
			{

				for (int tt = 0; tt < radius; tt++)
				{
					//这里只是给这一行
					m_vertexMotion[i*m_meshwidth + j+tt].x = motionx[motionx.size() / 2];
					m_vertexMotion[i*m_meshwidth + j+tt].y = motiony[motiony.size() / 2];

				}

			}
			else
			{
				m_vertexMotion[i*m_meshwidth + j].x = motionx[motionx.size() / 2];
				m_vertexMotion[i*m_meshwidth + j].y = motiony[motiony.size() / 2];
			}
		}


		if (i == m_meshHeight - radius)
		{
	
			for (int j = 0; j <= m_meshwidth - radius; j++)
			{
				vector<float> motionx;
				vector<float> motiony;
				for (int k = 0; k < radius; k++)
				{
					int m = (i + k) * 17 + j;

					for (int l = 0; l < radius; l++)
					{

						int t = m + l;
						if (t < tempVertexMotion.size())
						{
							motionx.push_back(tempVertexMotion[t].x);
							motiony.push_back(tempVertexMotion[t].y);
						}
					}
				}



				myQuickSort(motionx, 0, motionx.size() - 1);
				myQuickSort(motiony, 0, motiony.size() - 1);

				if (j == m_meshwidth - radius)
				{
					for (int p = 0; p < radius; p++)
					{
						for (int tt = 0; tt <radius ; tt++)
						{
							m_vertexMotion[(i + p)*m_meshwidth + j+tt].x = motionx[motionx.size() / 2];
							m_vertexMotion[(i + p)*m_meshwidth + j+tt].y = motiony[motiony.size() / 2];
						}
					}
				}
				else
				{

					for (int tt = 0; tt < radius; tt++)
					{

						m_vertexMotion[(i+tt)*m_meshwidth + j].x = motionx[motionx.size() / 2];
						m_vertexMotion[(i+tt)*m_meshwidth + j].y = motiony[motiony.size() / 2];

					}

				}

			}
		}		
		
	}

}


vector<Quad> MeshFlowVS::getQuad()
{
	vector<Quad> vq;
	for (int i = 1; i < m_mesh->height; i++)
	{
		for (int j = 1; j < m_mesh->width; j++)
		{
			Quad q = m_mesh->getQuad(i, j);
			vq.push_back(q);
		}
	}
	return vq;
}



void MeshFlowVS::Frame2subFrames(Mat m_frame, vector<Mat> &frames)
{
	frames.clear();
	for (int i = 0; i <10 - 1; i++)
	{
		for (int j = 0; j < 10 - 1; j++)
		{
			Rect rect(j*subImageWidth, i*subImageHeight, subImageWidth, subImageHeight);
			Mat frame = m_frame(rect);
			frames.push_back(frame);
		}
		Rect rect1(9 * subImageWidth, i*subImageHeight, m_width - 9* subImageWidth, subImageHeight);
		Mat frame1 = m_frame(rect1);
		frames.push_back(frame1);
	}

	for (int k = 0; k < 10; k++)
	{
		if (k != 9)
		{
			Rect rect2(k* subImageWidth, 9 * subImageHeight, subImageWidth, m_height - 9 * subImageHeight);
			Mat frame2 = m_frame(rect2);
			frames.push_back(frame2);
		}
		else
		{
			Rect rect3(k* subImageWidth, 9* subImageHeight, m_width - k* subImageWidth, m_height - 9 * subImageHeight);
			Mat frame3 = m_frame(rect3);
			frames.push_back(frame3);
		}
	}
}



//参数是图片的宽和高
void MeshFlowVS::initMeshFlow(int width, int height)
{
	m_height = height;
	m_width = width;
	//分为4*4个网格
	m_quadWidth = 1.0*m_width / pow(2.0, 4);
	m_quadHeight = 1.0*m_height / pow(2.0, 4);

	m_mesh = new Mesh(m_height, m_width, 1.0*m_quadWidth, 1.0*m_quadHeight);
	m_warpedemesh = new Mesh(m_height, m_width, 1.0*m_quadWidth, 1.0*m_quadHeight);

	m_meshHeight = m_mesh->height;//每列网格的个数
	m_meshwidth = m_mesh->width;//每行网格的个数

	m_vertexMotion.resize(m_meshHeight*m_meshwidth);
}



//将局部特征点转化为全局的特征点
void MeshFlowVS::local2glFratures(vector<Point2f> &glFeatures, vector<vector<Point2f> > lcFeatures)
{
	//int k = 0;

	for (int i = 0; i < lcFeatures.size(); i++)
	{
		int t = i % 4;
		int k = i / 4;
		for (int j = 0; j < lcFeatures[i].size(); j++)
		{
			float x = lcFeatures[i][j].x + t*subImageWidth;
			float y = lcFeatures[i][j].y + k*subImageHeight;
			glFeatures.push_back(Point2f(x, y));
		}
	}
}



//计算特征点与单应矩阵相乘
Point2f MeshFlowVS::TransPtbyH(Mat H, Point2f &pt)
{
	Point2f result;
	double a = H.at<double>(0, 0)*pt.x + H.at<double>(0, 1)*pt.y + H.at<double>(0, 2);
	double b = H.at<double>(1, 0)*pt.x + H.at<double>(1, 1)*pt.y + H.at<double>(1, 2);
	double c = H.at<double>(2, 0)*pt.x + H.at<double>(2, 1)*pt.y + H.at<double>(2, 2);

	result.x = a / c;
	result.y = b / c;

	return result;
}



//计算Tv，其参数Ft是全局的单应矩阵
double MeshFlowVS::calcTv(Mat Ft)
{
	double vx = Ft.at<double>(0, 2);
	double vy = Ft.at<double>(1, 2);
	return sqrt(vx*vx + vy*vy);
}


//计算Fa
double MeshFlowVS::calcuFa(Mat Ft)
{
	//首先补齐为3*3的
	Ft.at<double>(2, 0) = 0;
	Ft.at<double>(2, 1) = 0;
	Ft.at<double>(2, 2) = 1;

	//求特征值
	Mat eValuesMat;

	eigen(Ft, eValuesMat);
	vector<double> a;
	for (int i = 0; i < eValuesMat.rows; i++)
	{
		for (int j = 0; j < eValuesMat.cols; j++)
		{
			a.push_back(eValuesMat.at<double>(i, j));
		}
	}

	//求出其中的两个最大值
	double max1, max2;
	//首先进行排序
	sort(a.begin(), a.begin() + 3);

	max1 = a[1];
	max2 = a[2];
	return max1 / max2;
}


//预测lamda
double MeshFlowVS::predictLamda(double Tv, double Fa)
{
	double lamda1 = -1.93*Tv + 0.95;
	double lamda2 = 5.83*Fa + 4.88;

	double result = lamda1 < lamda2 ? lamda1 : lamda2;

	return result>0 ? result : 0;
}


//计算lamda
double MeshFlowVS::calcuLamda(Mat Ft)
{
	double	Fa = calcuFa(Ft);
	double  Tv = calcTv(Ft);
	return predictLamda(Tv, Fa);
}



//首先是对前t-1个帧进行计算,Omega代表时间平滑半径，r是其中的一个值，t是一个帧序号
double MeshFlowVS::calcuGuassianWeight(int r, int t, int Omegat)
{
	return exp(-pow((r - t), 2) / (pow(Omegat / 3, 2)));
}





//对本次迭代的缓冲区中的所有帧的优化路径进行初始化
void MeshFlowVS::initOptPath()
{

	for (int i = 0; i < optimizePathV.size(); i++)
	{
		optimizePathV[i] = originPathV[i];
	}

}



//根据不同情况设置时间平滑的起始位置以及结束位置
void MeshFlowVS::setTemporSmotthRange(int &beginframe, int &endframe, int t)
{
	const int k = 10 / 2;
	int range;
	//求帧t到两端的距离
	int ds = t;
	int de = frameV.size() - 1 - t;

	range = ds < de ? ds : de;
	range = range < k ? range : k;
	beginframe = t - range;
	endframe = t + range;
}



//从本质上讲，每次求优化路径需要20次迭代，每次优化最终只是一个结果，就是最后一帧,参数中的t本来就是每次迭代的中心位置
void MeshFlowVS::getPathT(int t, vector<Point2f> &optedPath)
{
	const int k = neighborsnum / 2;

	int beginframe = 0, endframe = frameV.size() - 1;


	setTemporSmotthRange(beginframe, endframe, t);

	double lamda = calcuLamda(homographV[t - 1]);

	vector<Point2f> Ct = originPathV[t];

	//第一次迭代时，并没有上一次的结果
	vector<Point2f> prePt = optimizationPathVV[optimizationPathVV.size() - 1][t - 1];
	double sum_wtr = 0;
	double gamme = 0;
	int omegat = endframe - beginframe + 1;
	vector<Point2f> sum_wtrP(Ct.size(), Point2f(0, 0));

	for (int r = beginframe; r <= endframe; r++)
	{

		vector<Point2f> Pr = optimizePathV[r];
		if (r != t)
		{
			/*for (int ri = beginframe; ri <r; ri++)
			{*/

			//第一次迭代设置初始值，计算并保存权重
			double tempWeight = calcuGuassianWeight(r, t, omegat);
			sum_wtr += tempWeight;
			//}
			//Pr乘tempWeight
			for (int pri = 0; pri < Pr.size(); pri++)
			{
				Pr[pri].x *= tempWeight;
				Pr[pri].y *= tempWeight;
				sum_wtrP[pri].x += Pr[pri].x;
				sum_wtrP[pri].y += Pr[pri].y;
			}
		}
	}



	gamme = 2 + lamda*sum_wtr;
	double wc = 1, wpre = 1, we = 1;
	wc = wc / gamme;
	wpre = wpre / gamme;
	we = we / gamme;

	//Ct乘wc
	for (int i = 0; i < Ct.size(); i++)
	{
		Ct[i].x *= wc;
		Ct[i].y *= wc;
	}

	//求prePt乘wpre
	for (int i = 0; i < prePt.size(); i++)
	{
		prePt[i].x *= wpre;
		prePt[i].y *= wpre;
	}

	//we与sum_wtrP相乘
	for (int wi = 0; wi < sum_wtrP.size(); wi++)
	{
		sum_wtrP[wi].x *= we;
		sum_wtrP[wi].y *= we;
	}


	//三个部分相加
	for (int pi = 0; pi < Ct.size(); pi++)
	{
		optedPath[pi].x = Ct[pi].x + prePt[pi].x + sum_wtrP[pi].x;
		optedPath[pi].y = Ct[pi].y + prePt[pi].y + sum_wtrP[pi].y;
	}

}



//合成视频帧
void MeshFlowVS::ViewSynthesis(int t, vector<Point2f> optPath, vector<Point2f> Ct)
{
	updatet.clear();
	//计算update
	updatet.resize(Ct.size());
	for (int i = 0; i < Ct.size(); i++)
	{
		updatet[i] = optPath[i] - Ct[i];
	}
	originQuad.clear();
	 originQuad = getQuad();
}


void MeshFlowVS::Jacobicompute2(int index)
{
	//vector<Point2f> optpath = optimizationPathVV[optimizationPathVV.size()-1][index];
	vector<Point2f> optpath = originPathV[originPathV.size() - 1];
	vector<Point2f> originPath = originPathV[originPathV.size() - 1];

	ViewSynthesis(index, optpath, originPath);
}

//雅克比计算
void MeshFlowVS::Jacobicompute()
{
	if (iter == 0)
	{//此时处理的是第三帧，第一次迭代，由于迭代过程中涉及到上次的优化结果，在这初始化
		vector<vector<Point2f>> optiPath;
		vector<Point2f> p1 = originPathV[0];
		vector<Point2f> p2 = originPathV[1];
		optiPath.push_back(p1);
		optiPath.push_back(p2);
		//将第1、2两帧的原始路径作为第一次优化的结果
		optimizationPathVV.push_back(optiPath);
	}

	//本次优化得到的路径将保存到optimizePathV中
	optimizePathV.clear();
	optimizePathV.resize(frameV.size());
	for (int pri = 0; pri <frameV.size(); pri++)
	{
		//首先将本次优化的帧的优化路径初始化为原始路径
		optimizePathV[pri] = originPathV[pri];
	}

	//#pragma omp  parallel for
	for (int it = 0; it < 10; it++)
	{//迭代10次

		for (int t = 1; t < frameV.size() - 1; t++)
		{
			//由于高斯分布的原因，从第1帧开始优化，到倒数第2帧结束，最后一帧用其他方法优化
			int beginframe = 0, endframe = frameV.size() - 1;

			//获取被优化的帧的时间优化半径
			setTemporSmotthRange(beginframe, endframe, t);

			//单应矩阵的个数应该比视频帧的个数少1
			double lamda = calcuLamda(homographV[t - 1]);
			
			//获取要优化的帧的原始路径
			vector<Point2f> Ct = originPathV[t];

			//获取该帧在上次迭代时的优化路径
			vector<Point2f> prePt = optimizationPathVV[optimizationPathVV.size() - 1][t];

			double sum_wtr = 0;
			double gamme = 0;
			//时间平滑半径
			int omegat = (endframe - beginframe + 1)/2;
			
			//初始化
			vector<Point2f> sum_wtrP(Ct.size(), Point2f(0, 0));


			for (int r = beginframe; r <= endframe; r++)
			{
				if (r != t)
				{//获取
					vector<Point2f> Pr = optimizePathV[r];
					//第一次迭代设置初始值，计算并保存权重
					double tempWeight = calcuGuassianWeight(r, t, omegat);
					sum_wtr += tempWeight;
					
					for (int pri = 0; pri < Pr.size(); pri++)
					{
						Pr[pri].x *= tempWeight;
						Pr[pri].y *= tempWeight;
						sum_wtrP[pri].x += Pr[pri].x;
						sum_wtrP[pri].y += Pr[pri].y;
					}
				}
			}



			gamme = 2 + lamda*sum_wtr;
			double wc = 1, wpre = 1, we = 1;
			wc = wc / gamme;
			wpre = wpre / gamme;
			we = we / gamme;

			//Ct乘wc
			for (int i = 0; i < Ct.size(); i++)
			{
				Ct[i].x *= wc;
				Ct[i].y *= wc;
			}

			//求prePt乘wpre
			for (int i = 0; i < prePt.size(); i++)
			{
				prePt[i].x *= wpre;
				prePt[i].y *= wpre;
			}

			//we与sum_wtrP相乘
			for (int wi = 0; wi < sum_wtrP.size(); wi++)
			{
				sum_wtrP[wi].x *= we;
				sum_wtrP[wi].y *= we;
			}


			//三个部分相加
			for (int pi = 0; pi < Ct.size(); pi++)
			{
				optimizePathV[t][pi].x = Ct[pi].x + prePt[pi].x + sum_wtrP[pi].x;
				optimizePathV[t][pi].y = Ct[pi].y + prePt[pi].y + sum_wtrP[pi].y;
			}
		}


	}

	updatet.clear();
	updatet.resize(originPathV[originPathV.size() - 1].size());
	int t = optimizePathV.size() - 1;
	vector<Point2f> tempoptim;

	//此时如果为三则是前两个，如果大于3则是前3个
	if (frameV.size() == 3)
	{
		for (int i = 0; i < originPathV[originPathV.size() - 1].size(); i++)
		{
			tempoptim.push_back(optimizePathV[t - 1][i] * 0.7 + optimizePathV[t - 2][i] * 0.3);
			updatet[i] = optimizePathV[t - 1][i] * 0.7 + optimizePathV[t - 2][i] * 0.3 - originPathV[originPathV.size() - 1][i];
		}
	}
	else
	{
		for (int i = 0; i < originPathV[originPathV.size() - 1].size(); i++)
		{
			tempoptim.push_back(optimizePathV[t][i] * 0.4 + optimizePathV[t - 1][i] * 0.4 + optimizePathV[t - 2][i] * 0.2);//+ optimizePathV[t - 2][i] * 0.2);//+ optimizePathV[t - 2][i] * 0.2 
			updatet[i] = optimizePathV[t][i] * 0.4 + optimizePathV[t - 1][i] * 0.4 + optimizePathV[t - 2][i] * 0.2 - originPathV[originPathV.size() - 1][i];
		}
	}


	optimizePathV[optimizePathV.size()-1] = tempoptim;
	optimizationPathVV.push_back(optimizePathV);
	iter++;
}




MeshFlowVS::MeshFlowVS(QWidget *parent)
	: QGLWidget(parent)
{
	m_imagePos = 0;
	fullscreen = false;
	m_xRot = 0.0f;
	m_yRot = 0.0f;
	m_zRot = 0.0f;
	m_FileName = "0.jpg";        //应根据实际存放图片的路径进行修改  
	m_ProcessFlag = false;
	initializeGL();
	m_currentFrame = cv::imread("1.jpg");
}

MeshFlowVS::~MeshFlowVS()
{

}



//
void MeshFlowVS::resizeGL(int w, int h)                 //重置OpenGL窗口的大小  
{
	glViewport(0, 0, (GLint)w, (GLint)h);               //重置当前的视口  
	glMatrixMode(GL_PROJECTION);                        //选择投影矩阵  
	glLoadIdentity();                                   //重置投影矩阵  
	//设置视口的大小  
	glMatrixMode(GL_MODELVIEW);                         //选择模型观察矩阵  
	glLoadIdentity();                                   //重置模型观察矩阵  
}


//加载纹理
void MeshFlowVS::loadGLTextures()
{

	//生成纹理
	glGenTextures(1, texture);

	//设置过滤
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);//最小化，线性插值
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);//最大化，线性插值

}

//初始化
void MeshFlowVS::initializeGL()                         //此处开始对OpenGL进行所以设置  
{
	loadGLTextures();
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	glClearColor(0, 0, 0, 0);
	glClearDepth(0.5);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_EQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}



//当textureCoorFlag为true时，计算纹理坐标，当为false时 计算世界坐标
//textureCoorFlag = true : compute the texture coordinate , false : compute the world coordinate
//当xFlag为true时，计算x坐标，当为false时计算y坐标
//xFlag  : true  : compute the x coordinate , false : compute y coordinate
//将像素转换到opengl的坐标  
//
float MeshFlowVS::pixel2GLCoor(float pCoor, float length, bool textureCoorFlag, bool xFlag, double ratio)
{
	//return pCoor;
	float glCoor;
	//
	if (textureCoorFlag)
	{
		//计算纹理坐标
		if (xFlag)
		{//计算x坐标
			glCoor = pCoor / length;
		}
		else
		{//计算y坐标
			glCoor = (length - pCoor) / length;
		}
	}
	else
	{
		//计算世界坐标
		if (xFlag)
		{
			//计算x轴坐标
			glCoor = (pCoor - length / 2) / (length / 2 * ratio);
		}
		else
		{//计算y坐标
			glCoor = (length / 2 - pCoor) / (length / 2 * ratio);
		}
	}
	return glCoor;
}


void MeshFlowVS::paintGL()                              //从这里开始进行所以的绘制
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除屏幕和深度缓存
	glLoadIdentity();                                   //重置模型观察矩阵
	
      
	cv::Mat rgb;
	cv::cvtColor(m_currentFrame, rgb, CV_BGR2RGB);



	//将图像
	buf1 = QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, rgb.cols*rgb.channels(), QImage::Format_RGB888);

	double cropRatio = 1;

	int width = buf1.width()*cropRatio;
	int height = buf1.height()*cropRatio;
	//设置纹理位置坐标
	setGeometry(100, 100, width, height);
	//将buf1从QImage形式转化为QGL形式
	tex1 = QGLWidget::convertToGLFormat(buf1);
	//绑定纹理texture[0]
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	//将纹理赋给GL_TEXTURE_2D
	glTexImage2D(GL_TEXTURE_2D, 0, 3, tex1.width(), tex1.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());
	//设置过滤
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	if (!m_ProcessFlag)
	{
		glBegin(GL_QUADS);

		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0, 0.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(1.0, -1.0, 0.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(1.0, 1.0, 0.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0, 1.0, 0.0);

		glEnd();
		glFlush();

	}
	else
	{

		//cout << "执行了" << endl;

		/****************首先计算四个边的平均值**************************/
		//上边
		double sumy = 0;
		double avgy;
		for (size_t i = 0; i < meshcount; i++)
		{

			sumy += originQuad[i].V00.y+updatet[i + i / meshcount].y;
			sumy += originQuad[i].V01.y+updatet[i + i / meshcount+1].y;
		}
		avgy = sumy / (meshcount*2);
	//	cout << "上边" << avgy << endl;

		//下边
		double sumy1 = 0;
		double avgy1;
		for (size_t i = originQuad.size() - meshcount; i < originQuad.size(); i++)
		{
			sumy1 += originQuad[i].V10.y+ updatet[i + i / meshcount + meshcount + 1].y;;
			sumy1 += originQuad[i].V11.y+ updatet[i + i / meshcount + meshcount + 2].y;
		}

		avgy1 = sumy1 / (meshcount * 2);

		//左边
		double sumy2 = 0;
		double avgy2;
		for (size_t i = 0; i < originQuad.size(); i += meshcount)
		{
			sumy2 += originQuad[i].V00.x +updatet[i + i / meshcount].x;
			sumy2 += originQuad[i].V10.x + updatet[i + i / meshcount + meshcount + 1].x;
		}

		avgy2 = sumy2 / (meshcount * 2);


		//右边
		double sumy3 = 0;
		double avgy3;
		for (size_t i = meshcount-1; i < originQuad.size(); i += meshcount)
		{
			sumy3 += originQuad[i].V01.x + updatet[i + i / meshcount + 1].x;
			sumy3 += originQuad[i].V11.x + updatet[i + i / meshcount + meshcount + 2].x;
		}
		avgy3 = sumy3 / (meshcount * 2);



		//既然边缘的中间不直，那就让其只根据四个角的坐标显示
		for (size_t i = 0; i < originQuad.size(); i++)
		{
			glBegin(GL_QUADS);
			float texX, texY, tarX, tarY;
	      	 
			//左上角
			double yltx = originQuad[i].V00.x;
			double ylty = originQuad[i].V00.y;

			//右上
			double yrtx = originQuad[i].V01.x;
			double yrty = originQuad[i].V01.y;

			//右下
			double yrdx = originQuad[i].V11.x;
			double yrdy = originQuad[i].V11.y;

			//左下
			double yldx = originQuad[i].V10.x;
			double yldy = originQuad[i].V10.y;

			//左上
			double nltx = yltx + updatet[i + i / meshcount].x;
			double nlty = ylty +updatet[i + i /meshcount].y;

			//右上
			double nrtx = yrtx + updatet[i + i /meshcount + 1].x;
			double nrty = yrty + updatet[i + i /meshcount + 1].y;

			//右下
			double nrdx = yrdx + updatet[i + i / meshcount + meshcount+2].x;
			double nrdy = yrdy + updatet[i + i / meshcount + meshcount+2].y;

			//左下
			double nldx = yldx + updatet[i + i / meshcount + meshcount+1].x;
			double nldy = yldy + updatet[i + i / meshcount + meshcount+1].y;

		

		   //最上边一行
			//if (i<meshcount)
			//{
			//	//1.左上
			//	texX = pixel2GLCoor(yltx, m_width, true);
			//	texY = pixel2GLCoor(ylty, m_height, true, false);

			//	tarX = pixel2GLCoor(nltx, m_width, false, true, cropRatio);
			//	tarY = avgy;
			//	glTexCoord2f(texX, texY);
			//	glVertex3f(tarX, tarY, 0.0);
			//	
			//	//2右上
			//	texX = pixel2GLCoor(yrtx, m_width, true);
			//	texY = pixel2GLCoor(yrty, m_height, true, false);

			//	tarX = pixel2GLCoor(nrtx, m_width, false, true, cropRatio);
			//	tarY = avgy;
			//	glTexCoord2f(texX, texY);
			//	glVertex3f(tarX, tarY, 0.0);

			//	//3右下
			//	texX = pixel2GLCoor(yrdx, m_width, true);
			//	texY = pixel2GLCoor(yrdy, m_height, true, false);

			//	tarX = pixel2GLCoor(nrdx, m_width, false, true, cropRatio);
			//	tarY = pixel2GLCoor(nrdy, m_height, false, false, cropRatio);
			//	glTexCoord2f(texX, texY);
			//	glVertex3f(tarX, tarY, 0.0);


			//	//4左下
			//	texX = pixel2GLCoor(yldx, m_width, true);
			//	texY = pixel2GLCoor(yldy, m_height, true, false);

			//	tarX = pixel2GLCoor(nldx, m_width, false, true, cropRatio);
			//	tarY = pixel2GLCoor(nldy, m_height, false, false, cropRatio);
			//	glTexCoord2f(texX, texY);
			//	glVertex3f(tarX, tarY, 0.0);
			//	continue;
			//}

			/*最左边一行
		     else if (i%meshcount == 0)
			{
				1左上
				texX = pixel2GLCoor(yltx, m_width, true);
				texY = pixel2GLCoor(ylty, m_height, true, false);

				tarX = avgy2;
				tarY = pixel2GLCoor(nlty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);


				2右上
				texX = pixel2GLCoor(yrtx, m_width, true);
				texY = pixel2GLCoor(yrty, m_height, true, false);

				tarX = pixel2GLCoor(nrtx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nrty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				3右下
				texX = pixel2GLCoor(yrdx, m_width, true);
				texY = pixel2GLCoor(yrdy, m_height, true, false);

				tarX = pixel2GLCoor(nrdx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nrdy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				4左下
				texX = pixel2GLCoor(yldx, m_width, true);
				texY = pixel2GLCoor(yldy, m_height, true, false);

				tarX = avgy2;
				tarY = pixel2GLCoor(nldy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				continue;
			}
			最右边一行
			else if (i%meshcount == meshcount-1)
			{

				1左上
				texX = pixel2GLCoor(yltx, m_width, true);
				texY = pixel2GLCoor(ylty, m_height, true, false);

				tarX = pixel2GLCoor(nltx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nlty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				2右上
				texX = pixel2GLCoor(yrtx, m_width, true);
				texY = pixel2GLCoor(yrty, m_height, true, false);

				tarX = avgy3;
				tarY = pixel2GLCoor(nrty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);


				3右下
				texX = pixel2GLCoor(yrdx, m_width, true);
				texY = pixel2GLCoor(yrdy, m_height, true, false);

				tarX = avgy3;
				tarY = pixel2GLCoor(nrdy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				4左下
				texX = pixel2GLCoor(yldx, m_width, true);
				texY = pixel2GLCoor(yldy, m_height, true, false);

				tarX = pixel2GLCoor(nldx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nldy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				continue;
			}
			 最下边一行
			else if (i > originQuad.size()-meshcount)
			{
				左上
				tarX = pixel2GLCoor(nltx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nlty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				左下
				texX = pixel2GLCoor(yldx, m_width, true);
				texY = pixel2GLCoor(yldy, m_height, true, false);

				tarX = pixel2GLCoor(nldx, m_width, false, true, cropRatio);
				tarY = avgy1;
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				右上
				texX = pixel2GLCoor(yrtx, m_width, true);
				texY = pixel2GLCoor(yrty, m_height, true, false);

				tarX = pixel2GLCoor(nrtx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nrty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);
				右下



				3右下
				texX = pixel2GLCoor(yrdx, m_width, true);
				texY = pixel2GLCoor(yrdy, m_height, true, false);

				tarX = pixel2GLCoor(nrdx, m_width, false, true, cropRatio);
				tarY = avgy1;
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				continue;
			}*/
			/*else
			{*/
				//1左上
				texX = pixel2GLCoor(yltx, m_width, true);
				texY = pixel2GLCoor(ylty, m_height, true, false);

				tarX = pixel2GLCoor(nltx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nlty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);



				//2右上
				texX = pixel2GLCoor(yrtx, m_width, true);
				texY = pixel2GLCoor(yrty, m_height, true, false);

				tarX = pixel2GLCoor(nrtx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nrty, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);


				//3右下
				texX = pixel2GLCoor(yrdx, m_width, true);
				texY = pixel2GLCoor(yrdy, m_height, true, false);

				tarX = pixel2GLCoor(nrdx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nrdy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);

				//4左下
				texX = pixel2GLCoor(yldx, m_width, true);
				texY = pixel2GLCoor(yldy, m_height, true, false);

				tarX = pixel2GLCoor(nldx, m_width, false, true, cropRatio);
				tarY = pixel2GLCoor(nldy, m_height, false, false, cropRatio);
				glTexCoord2f(texX, texY);
				glVertex3f(tarX, tarY, 0.0);
			//}
				

			glEnd();
		}

		glFlush();
		QImage img = QImage(width, height, QImage::Format_RGBA8888);
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, img.bits());
		img = img.mirrored();
		m_saveName =QString::fromStdString(imageSavepath) + "%1.jpg";
		
		QString saveName = m_saveName.arg(m_imagePos);
		img.save(saveName);
		m_imagePos++;
	}
	
}


void MeshFlowVS::viewV()
{
	VideoCapture vc(savevideopath);
	VideoCapture vc1(videopath1);
	if (!vc.isOpened())
	{
		return ;
	}

	Mat img,img2;
	vc >> img;
	vc1 >> img2;
	while (img.cols!=0&&img.rows!=0)
	{
		imshow("fdsf", img);
		imshow("fdsf2", img2);
		vc >> img;
		vc1 >> img2;
		waitKey(35);
	}
	return;



}


//将做稳定处理后的视频帧文件合成视频
void MeshFlowVS::image2Video()
{

	QString stableVideo = QFileDialog::getExistingDirectory(this, tr("Please select the directory for stable video"), "F:/stbdata/");
	if (stableVideo.isEmpty())
	{
		return;
	}

	Mat frame;
	//读第一帧，获取视频帧的大小，作为VideoWriter初始化时的参数
	frame = cv::imread(imageSavepath + "0.jpg");
	string tmp = imageSavepath.substr(0, imageSavepath.length() - 1);
	int t = tmp.find_last_of('/');
	string s = tmp.substr(0, t + 1);
	cv::VideoWriter vw(stableVideo.toStdString() + "/stable_" + videoname + ".avi", CV_FOURCC_DEFAULT, 25, frame.size(), true);
	
	savevideopath = stableVideo.toStdString() + "/stable_" + videoname + ".avi";
	for (int i = 0; i < frameNum; i++)
	{
		frame = cv::imread(imageSavepath + to_string(i) + ".jpg");
		vw << frame;
	}
	vw.release();

	////删除临时文件和文件夹
	//清除全局变量内容，用于下一次处理
	frameV.clear();
	glFeatures.clear();
	glFeatures2.clear();
	homographV.clear();
	optimizationPathVV.clear();
	optimizePathV.clear();
	originPathV.clear();
	
	removefilesindir(QString::fromStdString(imageSavepath));
	QMessageBox::StandardButton reply;
	reply = QMessageBox::information(this, "Information", "Save successfully");

	frameNum = 0;
}


//删除文件和文件夹
void MeshFlowVS::removefilesindir(const QString& path)
{
	QDir dir(path);
	QFileInfoList info_list = dir.entryInfoList(QDir::Files | QDir::Hidden | QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::AllDirs);
	foreach(QFileInfo file_info, info_list)
	{
		if (file_info.isDir())
		{
			removefilesindir(file_info.absoluteFilePath());
		}
		else if (file_info.isFile())
		{
			QFile file(file_info.absoluteFilePath());
			file.remove();
		}
	}
	QDir temp_dir;
	temp_dir.rmdir(path);
}


//判断每个特征点所属的网格子图
int MeshFlowVS::getSubIndex(Point2f p)
{
	int x = (int)p.x/subImageWidth;
	int y = (int)p.y/subImageHeight;
	return y * 10 + x;
}


Point2f MeshFlowVS::getPoint(int x, int y)
{
	return Point2f(xMat.at<double>(x,y),yMat.at<double>(x,y));
}



void MeshFlowVS::compute(int k,int matSize,int matrows,vector<double> b,Mat sum,Mesh* m1,Mesh *m2)
{
	double* x = new double[matSize];
	//值向量
	Mat bb = Mat::zeros(matrows,1,CV_64F);
	for (int i = 0; i < matrows; i++)
	{
		bb.at<double>(i, 0) = b[i];
	}
	
	//存储方程组的解
	Mat c1;
	solve(sum,bb,c1,DECOMP_NORMAL);
	
	for (int i = 0; i <c1.rows; i++)
	{
		x[i] = c1.at<double>(i, 0);
	}


	for (int i = 0; i <m_meshHeight; i++)
	{
		for (int j = 0; j < m_meshwidth; j++)
		{
			m2->setVertex(i,j,Point2f(x[i*m_meshwidth+j],x[i*m_meshwidth+j+matSize/2]));
		}
	}
}


void MeshFlowVS::computerHomos(Mat& img1, Mat& img2, int& w, int& h, int& index, int num, bool *isCancel)
{
	double tmp = 10000;
	Mesh *m1,*m2;
}


//图片的切分函数



//形状保持项其实就是网格中所有三角形任意一点减去的向量减去最小
vector<Point2f> f1;
vector<Point2f> f2;
vector<Point2f> points[2];
vector<Point2f> *points1;




void MeshFlowVS::startExe(string videopath)
{
	videopath1 = videopath;
	int localmincount = 0;


	int cutCountBefordetect=6;//3*2

	m_imagePos = 0;
	iter = 0;
	int tt=1;
	Mat	tempFrame;
	VideoCapture vc(videopath);

	Mat gray;
	vector<Point2f> pp1;
	vector<Point2f> pp2;
	Mat graypre;
	vector<Point2f> pref;
	vector<uchar> status;	// 跟踪特征的状态，特征的流发现为1，否则为0
	vector<float> err;
	vector<KeyPoint> tmpkp;

	int t = videopath.find_last_of('/');
	string tmpDir = videopath.substr(0, t + 1);
	videoname = videopath.substr(t + 1, videopath.length() - t - 1 - 4);
	//表示分割后的视频帧保存的目录
	string rootDir = tmpDir + "img/";
	frameNum = 0;
	if (access(rootDir.c_str(), 0) == -1)
	{
		int flag = mkdir(rootDir.c_str());
	}

	imageSavepath = rootDir;
	while (true)
	{
		start = clock();

		vc >> tempFrame;
		
		if (tempFrame.cols == 0 || tempFrame.rows == 0)
		{
			image2Video();
			break;
		}
		frameNum++;
		if (frameV.size()==40)
		{
			vector<Mat>::iterator it = frameV.begin();//当缓冲区中已经装满后，需要删除第一帧再添加
			frameV.erase(it);		//删除第一帧
			frameV.push_back(tempFrame);	//添加一帧
			vector<vector<Point2f>>::iterator it1 = originPathV.begin();	//不但要删除帧，还要对应删除单应矩阵以及，Ct
			originPathV.erase(it1);
			vector<vector<Point2f>>::iterator it2 = optimizePathV.begin();//此时原始路径的个数比帧数小1,上次迭代获得的优化路径要减一个
			optimizePathV.erase(it2);
		}
		else
			frameV.push_back(tempFrame);
		

		cv::cvtColor(tempFrame, gray, CV_BGR2GRAY);
		int startM = clock();
		if (tt == 1)
		{
			////////////
			m_width = frameV[0].cols;//获得视频帧的宽度
			m_height = frameV[0].rows;//获得水平帧的高度
			m_quadWidth = 1.0*m_width / meshcount; //pow(2.0, 4);//计算网格的宽度
			m_quadHeight = 1.0*m_height / meshcount; //pow(2.0, 4);//计算每个网格的高度
			Mesh mmesh(frameV[0].rows, frameV[0].cols, m_quadWidth, m_quadHeight);//初始化并构建网格
			xMat = mmesh.getXMat();//获得网格上所有坐标点的横坐标和纵坐标
			yMat = mmesh.getYMat();
			initMeshFlow(frameV[0].cols, frameV[0].rows);
	
			subImageHeight = m_height / 10;
			subImageWidth = m_width / 10;

			//在检测的时候就地切分
			double cutw = m_width / 3;
			double cuth = m_height / 2;

			

			if (m_width > 1000)
				localmincount = 50;
			else
				localmincount = 10;

			int threshold = 100;
			

			//特征点的提取必须是分块的

			
			//while (true)
			//{
				Mat img1 = gray(Rect(0, 0, cutw, cuth));
				Mat img2 = gray(Rect(cutw, 0, cutw, cuth));
				Mat img3 = gray(Rect(cutw * 2, 0, m_width - cutw * 2, cuth));
				Mat img4 = gray(Rect(0, cuth, cutw,m_height-cuth));
				Mat img5 = gray(Rect(cutw, cuth, cutw, m_height - cuth));
				Mat img6 = gray(Rect(cutw * 2, cuth, m_width - cutw * 2, m_height - cuth));
				
				
				
				vector<KeyPoint> tmpkpp;

				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img1, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if(tmpkpp.size() >=100)
					{
						break;
					}
				//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();

				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img2, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if(tmpkpp.size() >= 100)
					{
						break;
					}
					//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();


				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img3, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if (tmpkpp.size() >= 100)
					{
						break;
					}
					//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();



				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img4, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if (tmpkpp.size() >= 100)
					{
						break;
					}
					//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();


				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img5, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if (tmpkpp.size() >= 100)
					{
						break;
					}
					//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();


				while (true)
				{
					FastFeatureDetector fast(threshold);
					fast.detect(img6, tmpkpp);
					if (tmpkpp.size() < 100)
					{
						threshold -= 10;
					}
					else if (threshold<5)
					{
						break;
					}
					else if (tmpkpp.size() >= 100)
					{
						break;
					}
					//	tmpkpp.clear();
				}
				for (int i = 0; i < tmpkpp.size(); i++)
				{
					tmpkp.push_back(tmpkpp[i]);
				}
				tmpkpp.clear();
		


			/*	fast.detect(gray, tmpkp);
				if (tmpkp.size() < 800)
				{
					threshold -= 10;
				}*/

			//	else
			//		break;
			//}
				
			tt++;
			gray.copyTo(graypre);
			for (int i = 0; i <tmpkp.size(); i++)
			{
				pp1.push_back(tmpkp[i].pt);
			}

			glFeatures = pp1;
			initOriginPath();
			vector<Point2f> optpath = originPathV[originPathV.size() - 1];
			vector<Point2f> originPath = originPathV[originPathV.size() - 1];

			ViewSynthesis(0, optpath, originPath);
			m_currentFrame = frameV[frameV.size() - 1];
			
			updateGL();
			
			continue;
		}


		//kalman filters
		//time update prediction

	


		status.clear();
		err.clear();
		calcOpticalFlowPyrLK(graypre, gray, glFeatures, pp2,status , err);

		vector<Point2f> ppp = glFeatures;
		glFeatures.clear();
		glFeatures2.clear();
		//消除特征点误匹配
		int ptCount = status.size();//
		Mat p1(ptCount, 2, CV_32F);
		Mat p2(ptCount, 2, CV_32F);
		for (int j = 0; j < ptCount; j++)
		{
			p1.at<float>(j, 0) = ppp[j].x;
			p1.at<float>(j, 1) = ppp[j].y;

			p2.at<float>(j, 0) = pp2[j].x;
			p2.at<float>(j, 1) = pp2[j].y;
		}
		Mat m_Fundamental;
		vector<uchar> m_RANSACStatus;
		m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);
		for (int j = 0; j < ptCount; j++)
		{
			if (m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
			{
				status[j] = 0;
			}
		}
		//#pragma omp parallel for
		for (int j = 0; j < pp2.size(); j++)
		{
			//判断特征点坐标是否在图像范围内err[j] > 20 
			if (status[j] == 0 || (pp2[j].x <= 0 || pp2[j].y <= 0 || pp2[j].x >= m_width - 1 || pp2[j].y >= m_height - 1))
			{
				continue;
			}
			//将经过过滤的特征点放入临时特征容器中
			glFeatures2.push_back(pp2[j]);
			glFeatures.push_back(ppp[j]);
		}

		n.features.clear();
		n.features = glFeatures2;
		n.motions.resize(glFeatures2.size());
		m_globalHomography = findHomography(Mat(glFeatures), Mat(glFeatures2));
		homographV.push_back(m_globalHomography);

		//#pragma omp parallel for
		for (int i = 0; i < glFeatures2.size(); i++)
		{
			//加上残差
			n.motions[i] = glFeatures2[i] - glFeatures[i]+ glFeatures2[i] - Trans(m_globalHomography, glFeatures[i]);
		}


		int startM3 = clock();
		
		DistributeMotion2MeshVertexes_MedianFilter();
		//进行第二次中值滤波,结果仍然存储在m_vertexMotions中
		SpatialMedianFilter2();
		
		int endM3 = clock();
		
		
		computeOriginPath();
		
		if (tt == 2)
		{
			vector<Point2f> optpath = originPathV[originPathV.size() - 1];
			vector<Point2f> originPath = originPathV[originPathV.size() - 1];
		
			ViewSynthesis(1, optpath, originPath);
			tt++;
	
		}
		else
		{
	
			Jacobicompute();

			m_ProcessFlag = true;
		}

		m_currentFrame = frameV[frameV.size() - 1];

		//绘制特征点
		//	for (int i = 0; i <glFeatures2.size(); i++)
		//{
			//circle(m_currentFrame,glFeatures2[i],3,Scalar(0,0,255));
		//}



		updateGL();


		localFeatures.clear();
		localFeatures.resize(100);

		//#pragma omp  parallel for
		for (int k = 0; k < glFeatures2.size(); k++)
		{
			int t = getSubIndex(glFeatures2[k]);
			localFeatures[t].push_back(glFeatures2[k]);
		}

		int flag = 0;
		subImages.clear();
		Frame2subFrames(frameV[frameV.size() - 1], subImages);

		//#pragma omp  parallel for

		
		int startM2 = clock();
		for (int k = 0; k < localFeatures.size(); k++)
		{
			if (localFeatures[k].size() < localmincount)
			{
				int threshold = 100;
				while (true)
				{
					tmpkp.clear();
					FastFeatureDetector fast(threshold);
					cvtColor(subImages[k], subgray, CV_BGR2GRAY);
					fast.detect(subgray, tmpkp);
					if (threshold < 10)
						break;

					if (tmpkp.size() < localmincount)
					{
						threshold -= 5;
						
					}
					else
						break;
				}
				

				vector<Point2f> tmp;
				for (int i = 0; i < tmpkp.size(); i++)
				{
					tmp.push_back(tmpkp[i].pt);
				}


				//#pragma omp  parallel for
				for (int j = 0; j < tmp.size(); j++)
				{
					bool flag = true;
					for (int m = 0; m < localFeatures[k].size(); m++)
					{
						//选择与原来由光流法计算出的特征点相近的特征点
						if ((tmp[j].x - localFeatures[k][m].x)*(tmp[j].x - localFeatures[k][m].x) + (tmp[j].y - localFeatures[k][m].y)*(tmp[j].y - localFeatures[k][m].y) < 10)
						{
							flag = false;
							break;
						}
					}

					if (flag == true)
					{
						Point2f p(tmp[j].x + (k % 10)*subImageWidth, tmp[j].y + (k / 10)*subImageHeight);
						glFeatures2.push_back(p);
					}
				}


			}
		}

		subImages.clear();

		gray.copyTo(graypre);
		std::swap(glFeatures, glFeatures2);
		int endM = clock();
		finish = clock();
		duration = (double)(finish - start) / CLOCKS_PER_SEC;
		cout << "time" << duration << endl;
	}



}