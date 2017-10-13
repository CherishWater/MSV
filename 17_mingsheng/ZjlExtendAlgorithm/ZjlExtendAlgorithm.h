// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 CVEXTENDEDALGORITHM_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// CVEXTENDEDALGORITHM_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
#pragma once
#ifdef ZJLEXTENDALGORITHM_EXPORTS
#define ZJLEXTENDALGORITHM_API __declspec(dllexport)
#else
#define ZJLEXTENDALGORITHM_API __declspec(dllimport)
#endif

#include <opencv2\opencv.hpp>
#include <cv.h>
#include <vector>
using namespace cv;
using namespace std;

class ZJLEXTENDALGORITHM_API CZjlExtendAlgorithm
{
public:
	CZjlExtendAlgorithm();
	~CZjlExtendAlgorithm();



public:
	//图像中心旋转矩阵
	static Mat			getRotationMatrix2DModified(const Point2f& center, double angle, double scale,const Point& subColRow);
	//
	static int			rotateImage(Mat& imageSrc, Mat& imageDst, double angle, const Point& rotateCenter);
	//
	static double		calDistanceTwoPoints(const Point& pt1, const Point& pt2);

	static int			pointCoordinateTrans(const Point& ptOldOrigin,const Point& ptOld,const Point& ptNewOrigin,double angle, Point& ptNew);

	static void			pointLineMove(const Point2f& ptOld, const int nLength, const double &dAngle, Point& ptNew);

	static void			drawArrow(Mat& imSrc, Point pStart, int arrowLen, double arrowAngle, int smallLineLen, int smallLineAngle, Scalar& color, int thickness = 1, int lineType = 8);

};




