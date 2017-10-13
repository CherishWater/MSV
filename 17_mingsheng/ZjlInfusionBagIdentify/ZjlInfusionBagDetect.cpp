// ZjlInfusionBagDetect.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "ZjlInfusionBagDetect.hpp"


ZjlMedicalBagIdentify::ZjlMedicalBagIdentify()
{
	double contArea = 1900;
	double contWidth = 45;
	double contLength = 85;

	m_contSelFtrVal.push_back(contArea);
	m_contSelFtrVal.push_back(contWidth);
	m_contSelFtrVal.push_back(contLength);


	double aTh = 0.6;
	double wTh = 0.6;
	double lTh = 0.6;

	m_contSelFtrTh.push_back(aTh);
	m_contSelFtrTh.push_back(wTh);
	m_contSelFtrTh.push_back(lTh);

	m_nDistanceHead2Center = 170;

	m_szRoi.x = 120;
	m_szRoi.y = 45;
	m_szRoi.width = 500;
	m_szRoi.height = 500;

	m_ptRoiLocation.x = m_szRoi.x;
	m_ptRoiLocation.y = m_szRoi.y;

	m_dCameraXRatio = 0.6061;
	m_dCameraYRatio = 0.6186;

	m_ptCameraOrigin.x = 358;
	m_ptCameraOrigin.y = 286;

}

ZjlMedicalBagIdentify::~ZjlMedicalBagIdentify()
{}

int ZjlMedicalBagIdentify::imDetect(Mat& imSrc)
{
	m_nBagNumber = 0;

	Mat imSrcSub = imSrc(m_szRoi);
	
	imPreprocess(imSrcSub);

	vector<vector<Point>> contImPreprocess;
	findContours(m_imPreprocess, contImPreprocess, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	calBagInfo(contImPreprocess);

	drawMatchResult(imSrcSub);
	calSendInfo();

	m_nBagNumber = m_vtRotateRect.size();

	return 0;
}

int ZjlMedicalBagIdentify::imPreprocess(Mat& imSrc)
{
	int nRet = 0;

	Mat imGray;

	if (imSrc.channels() == 3)
	{
		cvtColor(imSrc, imGray, CV_BGR2GRAY);
	}
	else if (imSrc.channels() == 1)
	{
		imGray = imSrc.clone();
	}


#ifdef IMAGE_DEBUG_SHOW
	imshow("imGray", imGray);
	imwrite("imGray.bmp", imGray);
#endif

	Mat imFliter;
	medianBlur(imGray, imFliter, 3);

	Mat imThreshold;

	//threshold(imFliter, imThreshold, 190, 255, THRESH_BINARY);
	/*----------------------che modify--------------------------------*/
	threshold(imFliter, imThreshold, 190, 255, THRESH_BINARY);

#ifdef IMAGE_DEBUG_SHOW
	imshow("imThreshold", imThreshold);
	imwrite("imThreshold.bmp", imThreshold);
#endif

	Mat imErode;
	//Mat kernelElement = getStructuringElement(MORPH_RECT, Size(5, 5));	
	//morphologyEx(imThreshold, imErode, MORPH_ERODE, kernelElement, Point(-1, -1), 3);
	/*---------------------------che modify---------------------------*/
	Mat kernelElement = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imThreshold, imErode, MORPH_ERODE, kernelElement, Point(-1, -1),2);

#ifdef IMAGE_DEBUG_SHOW
	imshow("imErode", imErode);
	imwrite("imErode.bmp", imErode);
#endif


	vector<vector<Point>> contImErode;
	findContours(imErode, contImErode, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<vector<Point>> contRemain;
	contSelect(contImErode, contRemain);

	Mat imContRemain = Mat::zeros(imErode.rows, imErode.cols, CV_8UC1);
	drawContours(imContRemain, contRemain, -1, CV_RGB(255, 255, 255), -1);

#ifdef IMAGE_DEBUG_SHOW
	imshow("imContRemain", imContRemain);
	imwrite("imContRemain.bmp", imContRemain);
#endif

	Mat imDilate;
	//morphologyEx(imContRemain, imDilate,MORPH_DILATE, kernelElement, Point(-1, -1), 1);
	/*---------------------------che modify---------------------------*/
	morphologyEx(imContRemain, imDilate, MORPH_DILATE, kernelElement, Point(-1, -1), 1);

#ifdef IMAGE_DEBUG_SHOW
	imshow("imDilate", imDilate);
	imwrite("imDilate.bmp", imDilate);
#endif

	m_imPreprocess = imDilate.clone();

	return 0;
}

int ZjlMedicalBagIdentify::contSelect(vector<vector<Point>>& contVectSrc, vector<vector<Point>>& contVectRemain)
{
	int nCont = 0;

	contVectRemain.clear();
	for (auto it = contVectSrc.begin(); it != contVectSrc.end(); ++it)
	{
		if (contProcess(*it))
		{
			contVectRemain.push_back(*it);
			++nCont;
		}
	}
	return nCont;
}

bool ZjlMedicalBagIdentify::contProcess(const vector<Point>& contSrc)
{
	bool bRet = true;
	float area = fabs(contourArea(contSrc));
	RotatedRect rt = minAreaRect(contSrc);

	float wd, le;
	wd = rt.size.width;
	le = rt.size.height;

	if (wd > le)
	{
		float t = wd;
		wd = le;
		le = t;
	}

	vector<double> val;
	val.push_back(area);
	val.push_back(wd);
	val.push_back(le);

	for (size_t i = 0; bRet && i < m_contSelFtrVal.size(); ++i)
	{
		bRet &= (val[i] <= m_contSelFtrVal[i] * (2.0 - m_contSelFtrTh[i])
			&& val[i] >= m_contSelFtrVal[i] * m_contSelFtrTh[i]);
	}

	return bRet;
}

void ZjlMedicalBagIdentify::calLocation(Point& ptOld, Point& ptNew)
{
	ptNew.x = -(ptOld.x - (m_ptCameraOrigin.x-m_ptRoiLocation.x))*m_dCameraXRatio;
	ptNew.y = (ptOld.y - (m_ptCameraOrigin.y-m_ptRoiLocation.y))*m_dCameraXRatio;
}

void ZjlMedicalBagIdentify::drawMatchResult(Mat& imSrc)
{
//	CCvExtendedAlgorithm::ConvertToRGB(imSrc, m_imResult);

	for (auto itor = m_vtRotateRect.begin(); itor < m_vtRotateRect.end(); ++itor)
	{
		CZjlExtendAlgorithm::drawArrow(imSrc, itor->center, 100, itor->angle, 25, 25, Scalar(255, 0, 0), 5);
	}

	m_imResult = imSrc.clone();

#ifdef IMAGE_DEBUG_SHOW
	imshow("m_imResult", m_imResult);
	imwrite("m_imResult.bmp", m_imResult);
	waitKey();
#endif
}


void ZjlMedicalBagIdentify::calBagInfo(vector<vector<Point>>& contVectSrc)
{
	m_vtRotateRect.clear();
	for (auto it = contVectSrc.begin(); it < contVectSrc.end(); ++it)
	{
		BagInfo bagInfo;
		double agl;
		Point pt;
/*
		RotatedRect rt = minAreaRect(*it);

		switch (CCvExtendedAlgorithm::CalcRotateDirection(*it))
		{
		case DOWN_RIGHT:
			agl = 270 - rt.angle;
			break;
		case DOWN_LEFT:
			agl = 180 - rt.angle;
			break;
		case UP_LEFT:
			agl = 270 - rt.angle;
			break;
		case UP_RIGHT:
			agl = 180-rt.angle;
			break;
		case DONT_CARE:
			agl = 90.0+180.0;
			break;
		default:
			break;
		}

//		rt.center.x += 0;
//		rt.center.y += 0;

//		CZjlExtendAlgorithm::pointLineMove(rt.center, m_nDistanceHead2Center, agl + 180, pt);
		pt = rt.center;
*/

		Moments		m = moments(*it);
		double cx = m.m10 / m.m00;
		double cy = m.m01 / m.m00;

		double a = m.m20 / m.m00 - cx*cx;
		double b = 2 * (m.m11 / m.m00 - cx*cy);
		double c = m.m02 / m.m00 - cy*cy;

		pt.x = cx;
		pt.y = cy;

		agl = atan2f(b, (a - c)) / 2.0 * 180 / 3.14;
		agl = agl > 0.0 ? 180 - agl : -agl;
		bagInfo.angle = agl+180;
	
		if (pt.y > m_szRoi.height / 2)
		{
			Point ptCenter;
			CZjlExtendAlgorithm::pointLineMove(pt, m_nDistanceHead2Center, bagInfo.angle + 180, ptCenter);

			bagInfo.center = ptCenter;

			m_vtRotateRect.push_back(bagInfo);
		}
	}

}

void ZjlMedicalBagIdentify::calSendInfo()
{
	BagInfo bagInfo;
	m_vtSendInfo.clear();
	for (auto it = m_vtRotateRect.begin(); it < m_vtRotateRect.end(); ++it)
	{
		bagInfo.angle = it->angle-180;
		calLocation(it->center, bagInfo.center);
		m_vtSendInfo.push_back(bagInfo);
	}
}