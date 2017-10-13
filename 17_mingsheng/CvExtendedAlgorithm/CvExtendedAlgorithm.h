// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� CVEXTENDEDALGORITHM_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// CVEXTENDEDALGORITHM_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
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

// �����Ǵ� CvExtendedAlgorithm.dll ������

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
	//Gamma�Աȶ���ǿ
	//src:ԭͼ
	//dst:��ǿ��ͼ��
	//low:
	static void		ConvScale(cv::Mat src, cv::Mat &dst, double low, double high, double bottom, double top, double gamma);
	//���roi�е���ͼ��
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
	//��ͼ�����ҳ������Ʊ�Ե,�������Ե��Ŀ
	//src:ԭʼͼ��
	//contours:�洢��Ե
	static int		LabelAllFoundContours(cv::Mat src,cv::Mat &dst
		, cv::Scalar color, int thickness = 1,cv::Point drawOffset = cv::Point(), cv::Point findOffset = cv::Point()
		, int mode = CV_RETR_CCOMP, int method = CV_CHAIN_APPROX_NONE);
	//����RotatedRect����ľ��ԽǶ�
	//rot���������RotatedRect
	//pRef:�����Ϊ����ָʾ��RotatedRect��angle�Ĳο�ֵ
	//*pRef == 0�����ʾrot��angle��ʾ������x��ļн�
	//*pRef == 1�����ʾrot��angle��ʾ������x��ļн�
	static double	CalcRotatedRectDegree(const cv::RotatedRect rot, ROTATERECT_DIRECTION dir = DONT_CARE
		,int *pRef = nullptr);
	//�����������ߵĽǶ�
	static double	CalcLineDegree(const Point2f &pt1, const Point2f &pt2);
	//��RotatedRect�ƶ�����һ��λ��
	//rot:�ο���RotatedRect
	//offset:��λ�����rot��ƫ��
	//angle:��λ�����rot�ļн�
	//sz:��λ���µĳߴ磬��ΪSize()����λ���µĳߴ粻��
	//����ֵ:�ƶ����RotatedRect
	static RotatedRect MoveRotate(const cv::RotatedRect& rot, const Point2f &offset
		, const double& angle, const double &angleToCenter, const Size sz = Size(), ROTATERECT_DIRECTION dir = DONT_CARE);
	//�Ƚ�����ͼ���������
	//src:Դͼ��
	//dst:Ŀ��ͼ��
	//����ֵ:���ص�˫���ȸ�����ֵ��ʾ��ͼ������̶�,����ֵԽС��˵��ͼ�����ԽС
	static double	 CompareImage(const cv::Mat &src, const cv::Mat &dst);
	//����㼯�����������С���������ĵķ���
	//pointSet:�㼯
	//����ֵ:rot���������С���������ĵķ���
	static ROTATERECT_DIRECTION CalcRotateDirection(const vector<Point> &pointSet);
	//Ѱ�Ҷ�ֵ��ͼ���е���ͨ��
	static int FindContours(const cv::Mat Src, vector<vector<Point>>& contours);
	//����α�����ɫ
	static cv::Scalar GetRandColor();
	//����͹������
	static double	  HullLength(const vector<Point> &hull);
	//���Ƶ�
	static void		  MarkPoint(Mat & src, Point cen, int siz, char shape = '+', Scalar col = Scalar(0, 255, 0), int nlinewidth = 1);
	// contour �ķ���contour ������ĽϽ�����Ӿ��ζ̱�����������Զ����Ӿ��ζ̱����ĵķ���

};

