// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� CVEXTENDEDALGORITHM_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// CVEXTENDEDALGORITHM_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#pragma once
#ifdef ZJLINFUSIONBAGIDENTIFY_EXPORTS
#define ZJLINFUSIONBAGDETECT_API __declspec(dllexport)
#else
#define ZJLINFUSIONBAGDETECT_API __declspec(dllimport)
#endif

#include <opencv2\opencv.hpp>
#include <cv.h>
#include <vector>

#include "ZjlExtendAlgorithm.h"
#include "CvExtendedAlgorithm.h"
using namespace cv;
using namespace std;


#ifndef _T
#define _T __TEXT
#endif
//

//#define IMAGE_DEBUG_SHOW
//#define	TIME_RUN


enum CONTSELECT
{
	ALL_CONFORMITY,
	WHITH_HEIGHT_INCONFORMITY,
	AREA_INCONFORMITY
};


struct BagInfo
{
	Point center;
	double angle;
};


class ZJLINFUSIONBAGDETECT_API ZjlMedicalBagIdentify
{
public:
	ZjlMedicalBagIdentify();
	~ZjlMedicalBagIdentify();

public:

	vector<double> m_contSelFtrVal;
	vector<double> m_contSelFtrTh;
	
	Mat				m_imPreprocess;
	Mat				m_imResult;

	vector<BagInfo> m_vtRotateRect;
	vector<BagInfo> m_vtSendInfo;


	int				m_nDistanceHead2Center;
	int				m_nBagNumber;

	Rect						m_szRoi;
	Point						m_ptRoiLocation;
	Point						m_ptCameraOrigin;
	double						m_dCameraXRatio;
	double						m_dCameraYRatio;



public:

	int imPreprocess(Mat& imSrc);
	int	imDetect(Mat& imSrc);
	int contSelect(vector<vector<Point>>& contVectSrc, vector<vector<Point>>& contVectRemain);
	bool contProcess(const vector<Point>& contSrc);

	void calLocation(Point& ptOld, Point& ptNew);
	void drawMatchResult(Mat& imSrc);
	void calBagInfo(vector<vector<Point>>& contVectSrc);
	void calSendInfo();

};


