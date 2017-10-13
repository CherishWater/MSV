// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 CVEXTENDEDALGORITHM_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// CVEXTENDEDALGORITHM_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
#pragma once
#ifdef CVEXTENDEDALGORITHM_EXPORTS
#define CVEXTENDEDALGORITHM_API __declspec(dllexport)
#else
#define CVEXTENDEDALGORITHM_API __declspec(dllimport)
#endif
#include "opencv.hpp"
#include <vector>
using namespace cv;
using namespace std;

// 此类是从 CvExtendedAlgorithm.dll 导出的

enum CVEXTENDEDALGORITHM_API ROTATERECT_DIRECTION
{
	DONT_CARE,
	DOWN_RIGHT,
	DOWN_LEFT,
	UP_LEFT,
	UP_RIGHT
};

class CVEXTENDEDALGORITHM_API CCvExtendedAlgorithm 
{
public:
	CCvExtendedAlgorithm(void);
private:
	static			RNG		sm_clrRNG;
public:
	//Gamma对比度增强
	//src:原图
	//dst:增强后图像
	//low:
	static void		ConvScale(cv::Mat src, cv::Mat &dst, double low, double high, double bottom, double top, double gamma);
	//获得roi中的子图像
	static void		GetSubMat(cv::Mat src, cv::Mat &dst, cv::Rect roi, int margin = 0, bool bCvtColor = false);
	//huo
	static void		GetSubMat(cv::Mat src, cv::Mat &dst, cv::RotatedRect roi, int margin = 0, bool bCvtColor = false);
	static void		InverseBW(cv::Mat src, cv::Mat &dst);
	static void		ExtractSkeleton(cv::Mat src, cv::Mat &dst, vector<Point> &ends, const int maxIterations = -1);
	static void		SeedFilling(cv::Mat src, vector<vector<Point>> &obArr);
	static void		DrawRotateRect(cv::Mat &src, RotatedRect rect, Scalar color, int thickness = 1, int lineType = 1 , bool byPoint = true);
	static void		DrawIcon(cv::Mat &src, cv::Mat icon, Point ori = Point(0, 0));
	static int		DistanceOf(cv::Point frt, cv::Point snd);
	static Point	CenterOf(cv::Point frt, cv::Point snd);
	static void		ConvertToGray(cv::InputArray src, cv::OutputArray dst);
	static void		ConvertToRGB(cv::InputArray src, cv::OutputArray dst);
	static double	CrossCorrelation(const cv::Mat& src,const cv::Mat &dst);
	//在图像中找出并绘制边缘,并输出边缘数目
	//src:原始图像
	//contours:存储边缘
	static int		LabelAllFoundContours(cv::Mat src,cv::Mat &dst
		, cv::Scalar color, int thickness = 1,cv::Point drawOffset = cv::Point(), cv::Point findOffset = cv::Point()
		, int mode = CV_RETR_CCOMP, int method = CV_CHAIN_APPROX_NONE);
	//计算RotatedRect长轴的绝对角度
	//rot：被计算的RotatedRect
	//pRef:如果不为空则指示了RotatedRect中angle的参考值
	//*pRef == 0：则表示rot的angle表示长轴与x轴的夹角
	//*pRef == 1：则表示rot的angle表示短轴与x轴的夹角
	static double	CalcRotatedRectDegree(const cv::RotatedRect rot, ROTATERECT_DIRECTION dir = DONT_CARE
		,int *pRef = nullptr);
	//计算两点连线的角度
	static double	CalcLineDegree(const Point2f &pt1, const Point2f &pt2);
	//将RotatedRect移动到另一个位置
	//rot:参考的RotatedRect
	//offset:新位置相对rot的偏移
	//angle:新位置相对rot的夹角
	//sz:新位置下的尺寸，若为Size()则新位置下的尺寸不变
	//返回值:移动后的RotatedRect
	static RotatedRect MoveRotate(const cv::RotatedRect& rot, const Point2f &offset
		, const double& angle, const double &angleToCenter, const Size sz = Size(), ROTATERECT_DIRECTION dir = DONT_CARE);
	//比较两幅图像的相似性
	//src:源图像
	//dst:目标图像
	//返回值:返回的双精度浮点数值表示两图的相异程度,返回值越小，说明图像差异越小
	static double	 CompareImage(const cv::Mat &src, const cv::Mat &dst);
	//计算点集质心相对其最小外界矩形中心的方向
	//pointSet:点集
	//返回值:rot质心相对最小外界矩形中心的方向
	static ROTATERECT_DIRECTION CalcRotateDirection(const vector<Point> &pointSet);
	//寻找二值化图像中的连通线
	static int FindContours(const cv::Mat Src, vector<vector<Point>>& contours);
	//生产伪随机颜色
	static cv::Scalar GetRandColor();
	//计算凸包长度
	static double	  HullLength(const vector<Point> &hull);
	//绘制点
	static void		  MarkPoint(Mat & src, Point cen, int siz, char shape = '+', Scalar col = Scalar(0, 255, 0), int nlinewidth = 1);
	// contour 的方向，contour 相距重心较近的外接矩形短边中心与相距较远的外接矩形短边中心的方向

};

