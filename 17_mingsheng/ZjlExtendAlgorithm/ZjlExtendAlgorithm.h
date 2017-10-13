// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� CVEXTENDEDALGORITHM_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// CVEXTENDEDALGORITHM_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
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
	//ͼ��������ת����
	static Mat			getRotationMatrix2DModified(const Point2f& center, double angle, double scale,const Point& subColRow);
	//
	static int			rotateImage(Mat& imageSrc, Mat& imageDst, double angle, const Point& rotateCenter);
	//
	static double		calDistanceTwoPoints(const Point& pt1, const Point& pt2);

	static int			pointCoordinateTrans(const Point& ptOldOrigin,const Point& ptOld,const Point& ptNewOrigin,double angle, Point& ptNew);

	static void			pointLineMove(const Point2f& ptOld, const int nLength, const double &dAngle, Point& ptNew);

	static void			drawArrow(Mat& imSrc, Point pStart, int arrowLen, double arrowAngle, int smallLineLen, int smallLineAngle, Scalar& color, int thickness = 1, int lineType = 8);

};




