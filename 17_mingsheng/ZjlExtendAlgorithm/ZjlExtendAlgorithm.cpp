// ZjlExtendAlgorithm.cpp : 定义 DLL 应用程序的导出函数。
//
//#define ZJLEXTENDALGORITHM_EXPORTS
#include "stdafx.h"
#include "ZjlExtendAlgorithm.h"

CZjlExtendAlgorithm::CZjlExtendAlgorithm()
{

}

CZjlExtendAlgorithm::~CZjlExtendAlgorithm()
{
}

Mat CZjlExtendAlgorithm::getRotationMatrix2DModified(const Point2f& center, double angle, double scale, const Point& subColRow)
{
	angle *= CV_PI / 180;
	double alpha = cos(angle)*scale;
	double beta = sin(angle)*scale;

	Mat M(2, 3, CV_64F);
	double* m = (double*)M.data;

	m[0] = alpha;
	m[1] = beta;
	m[2] = (1 - alpha)*center.x - beta*center.y;
	m[3] = -beta;
	m[4] = alpha;
	m[5] = beta*center.x + (1 - alpha)*center.y;

	m[2] += subColRow.x;
	m[5] += subColRow.y;

	return M;
}

int CZjlExtendAlgorithm::rotateImage(Mat& imageSrc, Mat& imageDst, double angle, const Point& rotateCenter)
{
	int nRet = 0;
	double a = sin(angle*CV_PI / 180.0), b = cos(angle*CV_PI / 180.0);
	
	//旋转后的图像长度，宽度
	int width = imageSrc.cols;
	int height = imageSrc.rows;

	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));

	Point subColRow;
	subColRow.x = (width_rotate - width) / 2;
	subColRow.y = (height_rotate - height) / 2;

	Mat mapMatrix = getRotationMatrix2DModified(rotateCenter, angle, 1.0, subColRow);
	warpAffine(imageSrc, imageDst, mapMatrix, Size(width_rotate, height_rotate));

	return 0;
}

double CZjlExtendAlgorithm::calDistanceTwoPoints(const Point& pt1, const Point& pt2) 
{
	return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
}

int CZjlExtendAlgorithm::pointCoordinateTrans(const Point& ptOldOrigin, const Point& ptOld, const Point& ptNewOrigin,double angle, Point& ptNew)
{
	double a = sin(angle * CV_PI / 180);
	double b = cos(angle * CV_PI / 180);

	int nCols = static_cast<int>(ptOld.x - ptOldOrigin.x +0.5f);
	int nRows = static_cast<int>(-(ptOld.y - ptOldOrigin.y +0.5f));

	ptNew.x = static_cast<int>(ptNewOrigin.x  + nCols*b + nRows*a+0.5f);
	ptNew.y = static_cast<int>(ptNewOrigin.y  - (-nCols*a + nRows*b)+0.5f);
	return 0;
}


//图像坐标系与定位的像素坐标系之间的坐标变换
//y方向坐标相减

void CZjlExtendAlgorithm::pointLineMove(const Point2f& ptOld, const int nLength, const double &dAngle, Point& ptNew)
{
	ptNew.x =static_cast<int>( ptOld.x + nLength*cos(dAngle*CV_PI / 180.0)+0.5f);
	ptNew.y =static_cast<int>( ptOld.y - nLength*sin(dAngle*CV_PI / 180.0)+0.5f);
}


void CZjlExtendAlgorithm::drawArrow(Mat& imSrc, Point pStart, int arrowLen, double arrowAngle, int smallLineLen,
	int smallLineAngle, Scalar& color, int thickness, int lineType)
{
	const double PI = 3.1415926;
	Point pEnd;
	pEnd.x = pStart.x + arrowLen*cos(arrowAngle*PI / 180.0);
	pEnd.y = pStart.y - arrowLen*sin(arrowAngle*PI / 180.0);
	line(imSrc, pStart, pEnd, color, thickness, lineType);

	Point arrow;
	arrow.x = pEnd.x + smallLineLen * cos((arrowAngle + smallLineAngle + 180)*PI / 180.0);
	arrow.y = pEnd.y - smallLineLen * sin((arrowAngle + smallLineAngle + 180)*PI / 180.0);
	line(imSrc, pEnd, arrow, color, thickness, lineType);
	arrow.x = pEnd.x + smallLineLen * cos((arrowAngle - smallLineAngle + 180)*PI / 180.0);
	arrow.y = pEnd.y - smallLineLen * sin((arrowAngle - smallLineAngle + 180)*PI / 180.0);
	line(imSrc, pEnd, arrow, color, thickness, lineType);
}
