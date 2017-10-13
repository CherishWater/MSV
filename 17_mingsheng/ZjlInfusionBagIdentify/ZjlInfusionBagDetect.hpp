// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 CVEXTENDEDALGORITHM_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// CVEXTENDEDALGORITHM_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
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


