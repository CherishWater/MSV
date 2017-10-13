// CvExtendedAlgorithm.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "CvExtendedAlgorithm.h"
#include <stack>
using namespace std;

// 这是已导出类的构造函数。
// 有关类定义的信息，请参阅 CvExtendedAlgorithm.h

RNG CCvExtendedAlgorithm::sm_clrRNG = RNG();

CCvExtendedAlgorithm::CCvExtendedAlgorithm()
{
	CCvExtendedAlgorithm::sm_clrRNG.uniform(0, 255);
	return;
}



void CCvExtendedAlgorithm::ConvScale(cv::Mat src, cv::Mat &dst, double low, double high, double bottom, double top, double gamma)
{
	dst = Mat::zeros(Mat(src).rows, src.cols, src.type());
	if (low < 0.0 || low > 1.0 || high < 0.0 || high >1.0
		|| top < 0.0 || top > 1.0 || bottom < 0.0 || bottom > 1.0 || low > high)
		return;
	double	low2 = low * 255;
	double	high2 = high * 255;
	double	bottom2 = bottom * 255;
	double	top2 = top * 255;
	double	err_in = high2 - low2;
	double  err_out = top2 - bottom2;
	double val;
	uchar* ptrSrc;
	uchar* ptrDst;
	for (int ch = 0; ch < dst.channels(); ch++)
	{
		for (int i = 0; i < src.rows; i++)
		{
			ptrSrc = src.ptr<uchar>(i);
			ptrDst = dst.ptr<uchar>(i);
			for (int j = 0; j < src.cols; j++)
			{
				val = *(ptrSrc+j+ch);
				val = pow((val - low2) / err_in, gamma)*err_out + bottom2;
				val = (val> 254.5) ? 254.5 : (val < 0) ? 0 : val;
				*(ptrDst+j+ch) = static_cast<uchar>(val + 0.5);
			}
		}
	}
}

void CCvExtendedAlgorithm::GetSubMat(cv::Mat src, cv::Mat &dst, Rect roi, int margin, bool bCvtColor)
{
	int		l = (roi.x - margin < 0) ? 0 : roi.x - margin;
	int		r = (roi.x + margin + roi.width > src.cols) ? src.cols : roi.x + margin + roi.width;
	int		t = (roi.y - margin < 0) ? 0 : roi.y - margin;
	int		b = (roi.y + margin + roi.height > src.rows) ? src.rows : roi.y + margin + roi.height;
	dst = src.rowRange(t, b).clone();
	dst = dst.colRange(l, r).clone();
	if (bCvtColor)
	{
		CCvExtendedAlgorithm::ConvertToGray(dst, dst);
	}
}

void CCvExtendedAlgorithm::GetSubMat(cv::Mat src, cv::Mat &dst, cv::RotatedRect roi, int margin, bool bCvtColor)
{
	int		t(0), b(0), l(0), r(0);
	Mat		rotMat;
	//roi大小
	int		halfWidth = static_cast<int>(roi.size.width / 2 + 0.5f);
	int		halfHeight = static_cast<int>(roi.size.height / 2 + 0.5f);
	if (roi.angle == 0.0 || roi.angle == 90.0 || roi.angle == 180.0 || roi.angle == 270.0)
	{
		src.copyTo(rotMat);
		l = (roi.center.x - halfWidth - margin > 0) ? roi.center.x - halfWidth - margin : 0;
		r = (roi.center.x + halfWidth + margin < src.cols) ? roi.center.x + halfWidth + margin : src.cols;
		t = (roi.center.y - halfHeight -margin> 0) ? roi.center.y - halfHeight - margin : 0;
		b = (roi.center.y + halfHeight +margin< src.rows) ? roi.center.y + halfHeight +margin : src.rows;
	}
	else
	{
		//根据Roi的角度旋转图像，转为弧度单位
		double	ang = roi.angle*CV_PI / 180.0;
		//旋转后图像的尺寸，
		Size	rotSize;
		rotSize.height = static_cast<int>(src.rows * abs(cos(ang)) + src.cols * abs(sin(ang)) + 0.5);
		rotSize.width = static_cast<int>(src.rows * abs(sin(ang)) + src.cols * abs(cos(ang)));
		//旋转后的图像
		rotMat = Mat::zeros(rotSize, src.type());
		//旋转后图像的边界宽度
		int		top = abs(static_cast<int>((rotSize.height - src.rows) / 2.0 + 0.5));
		int		left = abs(static_cast<int>((rotSize.width - src.cols) / 2.0 + 0.5));
		//将原始图像复制到旋转后的图像中央位置
		copyMakeBorder(src, rotMat, top, top, left, left, BORDER_CONSTANT);
		//计算旋转矩阵：旋转中心为旋转后图像的中心，旋转角度为roi的角度,缩放比例1（不缩放）
		Mat		rot = getRotationMatrix2D(Point2f(rotSize.width / 2.0f, rotSize.height / 2.0f), roi.angle, 1.0);
		//旋转
		warpAffine(rotMat, rotMat, rot, rotSize);
		//旋转前roi中心的位置
		int		rcx = static_cast<int>(roi.center.x + 0.5) + left;
		int		rcy = static_cast<int>(roi.center.y + 0.5) + top;
		//旋转后roi中心的位置
		Mat		rot2 = getRotationMatrix2D(Point2f(rotSize.width / 2.0f, rotSize.height / 2.0f), -roi.angle, 1.0);
		double	*mptr = (double*)rot2.data;
		int		cx = static_cast<int>(((rcx - mptr[2])*mptr[4] - (rcy - mptr[5])*mptr[1]) / (mptr[0] * mptr[4] - mptr[3] * mptr[1]) + 0.5);
		int		cy = static_cast<int>(((rcx - mptr[2])*mptr[3] - (rcy - mptr[5])*mptr[0]) / (mptr[1] * mptr[3] - mptr[4] * mptr[0]) + 0.5);
		//提取ROI
		t = (cy - halfHeight - margin < 0) ? 0 : cy - halfHeight - margin;
		l = (cx - halfWidth - margin < 0) ? 0 : cx - halfWidth - margin;
		b = (cy + halfHeight + 1 + margin > rotSize.height) ? rotSize.height : cy + halfHeight + 1 + margin;
		r = (cx + halfWidth + 1 + margin > rotSize.width) ? rotSize.width : cx + halfWidth + 1 + margin;
	}
	dst = rotMat.colRange(l, r).clone();
	dst = dst.rowRange(t, b).clone();
	if (bCvtColor)
	{
		if (dst.channels() == 3)
			cvtColor(dst, dst, CV_RGB2GRAY);
		else if (dst.channels() == 4)
			cvtColor(dst, dst, CV_RGBA2GRAY);
	}
}

void CCvExtendedAlgorithm::InverseBW(cv::Mat src, cv::Mat &dst)
{
	CV_DbgAssert(src.type() == CV_8UC1);
	uchar *SrcPtr = nullptr;
	uchar *DstPtr = nullptr;
	dst = Mat::zeros(src.size(), src.type());
	for (int i = 0; i < src.rows; i++)
	{
		SrcPtr = src.ptr<uchar>(i);
		DstPtr = dst.ptr<uchar>(i);
		for (int j = 0; j < src.cols; j++)
		{
			if (SrcPtr[j] == 0)
				DstPtr[j] = 255;
		}
	}
}

void CCvExtendedAlgorithm::ExtractSkeleton(cv::Mat src, cv::Mat &dst, vector<Point> &ends, const int maxIterations)
{
	CV_DbgAssert(src.type() == CV_8UC1);
	dst = src.clone();
	int		count = 0;
	uchar	*ptrSrc = nullptr;
	uchar	*ptrDst = nullptr;
	//////////////////
	// p9 p2 p3
	// p8 p1 p4
	// p7 p6 p5
	uchar	p1, p2, p3, p4, p5, p6, p7, p8, p9, ap;
	vector<pair<int, int>> flag;
	bool normalLoop = true;
	bool lastLoop = false;
	while (normalLoop || lastLoop)
	{
		count++;
		if (maxIterations != -1 && count > maxIterations)
			break;

		for (int i = 0; i < src.rows; i++)
		{
			ptrSrc = src.ptr<uchar>(i);
			ptrDst = dst.ptr<uchar>(i);
			for (int j = 0; j < src.cols; j++)
			{
				p1 = ptrDst[j];
				if (p1 != 255)
					continue;
				p2 = (i == 0) ? 0 : ((ptrDst - src.step)[j] == 255) ? 1 : 0;
				p3 = (i == 0 || j == src.cols - 1) ? 0 : ((ptrDst - src.step)[j + 1] == 255) ? 1 : 0;
				p4 = (j == src.cols - 1) ? 0 : (ptrDst[j + 1] == 255) ? 1 : 0;
				p5 = (i == src.rows - 1 || j == src.cols - 1) ? 0 : ((ptrDst + src.step)[j + 1] == 255) ? 1 : 0;
				p6 = (i == src.rows - 1) ? 0 : ((ptrDst + src.step)[j] == 255) ? 1 : 0;
				p7 = (i == src.rows - 1 || j == 0) ? 0 : ((ptrDst + src.step)[j - 1] == 255) ? 1 : 0;
				p8 = (j == 0) ? 0 : ((ptrDst[j - 1]) == 255) ? 1 : 0;
				p9 = (i == 0 || j == 0) ? 0 : ((ptrDst - src.step)[j - 1] == 255) ? 1 : 0;

				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					ap = 0;
					ap += (p2 == 0 && p3 == 1) ? 1 : 0;
					ap += (p3 == 0 && p4 == 1) ? 1 : 0;
					ap += (p4 == 0 && p5 == 1) ? 1 : 0;
					ap += (p5 == 0 && p6 == 1) ? 1 : 0;
					ap += (p6 == 0 && p7 == 1) ? 1 : 0;
					ap += (p7 == 0 && p8 == 1) ? 1 : 0;
					ap += (p8 == 0 && p9 == 1) ? 1 : 0;
					ap += (p9 == 0 && p2 == 1) ? 1 : 0;
					if (ap == 1 && p2*p4*p6 == 0 && p4*p6*p8 == 0)
					{
						flag.push_back(make_pair(i, j));
					}
				}
				else if (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9 == 1 && lastLoop)
					ends.push_back(Point(j, i));
			}
		}
		for (auto itor = flag.begin(); itor != flag.end(); itor ++)
		{
			*(dst.data + src.step*itor->first + itor->second) = 0;
		}
		if (flag.size() == 0 && normalLoop)
		{
			//break;
			normalLoop = false;
			lastLoop = true;
		}
		else if (flag.size() == 0 && lastLoop)
		{
			break;
		}
		else
			flag.clear();

		for (int i = 0; i < src.rows; i++)
		{
			ptrSrc = src.ptr<uchar>(i);
			ptrDst = dst.ptr<uchar>(i);
			for (int j = 0; j < src.cols; j++)
			{
				p1 = ptrDst[j];
				if (p1 != 255)
					continue;
				p2 = (i == 0) ? 0 : ((ptrDst - src.step)[j] == 255) ? 1 : 0;
				p3 = (i == 0 || j == src.cols - 1) ? 0 : ((ptrDst - src.step)[j + 1] == 255) ? 1 : 0;
				p4 = (j == src.cols - 1) ? 0 : (ptrDst[j + 1] == 255) ? 1 : 0;
				p5 = (i == src.rows - 1 || j == src.cols - 1) ? 0 : ((ptrDst + src.step)[j + 1] == 255) ? 1 : 0;
				p6 = (i == src.rows - 1) ? 0 : ((ptrDst + src.step)[j] == 255) ? 1 : 0;
				p7 = (i == src.rows - 1 || j == 0) ? 0 : ((ptrDst + src.step)[j - 1] == 255) ? 1 : 0;
				p8 = (j == 0) ? 0 : ((ptrDst[j - 1]) == 255) ? 1 : 0;
				p9 = (i == 0 || j == 0) ? 0 : ((ptrDst - src.step)[j - 1] == 255) ? 1 : 0;

				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					ap = 0;
					ap += (p2 == 0 && p3 == 1) ? 1 : 0;
					ap += (p3 == 0 && p4 == 1) ? 1 : 0;
					ap += (p4 == 0 && p5 == 1) ? 1 : 0;
					ap += (p5 == 0 && p6 == 1) ? 1 : 0;
					ap += (p6 == 0 && p7 == 1) ? 1 : 0;
					ap += (p7 == 0 && p8 == 1) ? 1 : 0;
					ap += (p8 == 0 && p9 == 1) ? 1 : 0;
					ap += (p9 == 0 && p2 == 1) ? 1 : 0;
					if (ap == 1 && p2*p4*p8 == 0 && p2*p6*p8 == 0)
					{
						flag.push_back(make_pair(i, j));
					}
				}
				else if (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9 == 1 && lastLoop)
					ends.push_back(Point(j, i));
			}
		}
		for (auto itor = flag.begin(); itor != flag.end(); itor++)
		{
			*(dst.data + src.step*itor->first + itor->second) = 0;
		}
		if (flag.size() == 0 && normalLoop)
		{
			//break;
			normalLoop = false;
			lastLoop = true;
		}
		else if (flag.size() == 0 && lastLoop)
		{
			lastLoop = false;
		}
		else
			flag.clear();
	}
}

void CCvExtendedAlgorithm::SeedFilling(cv::Mat src, vector<vector<Point>> &obArr)
{
	CV_DbgAssert(!src.empty() && src.type() == CV_8UC1);
	Mat		lsrc = src.clone();
	int		step = lsrc.step;
	uchar	*SrcPtr = lsrc.data;
	std::stack<std::pair<int, int>> forSearching;
	std::pair<int, int>	 curSearching;
	int		ix, iy;
	for (int i = 0; i < lsrc.rows; i++)
	{
		for (int j = 0; j < lsrc.cols; j++)
		{

			if (*(SrcPtr+i*step+j) == 255)
			{
				std::vector<Point>				ob;
				forSearching.push(std::pair<int, int>(i, j));     // 像素位置: <i,j>
				ob.push_back(Point(j, i));
				*(SrcPtr + i*step + j) = 128;
				while (!forSearching.empty())
				{
					curSearching = forSearching.top(); //如果与上一行中一个团有重合区域，则将上一行的那个团的标号赋给它
					ix = curSearching.first;
					iy = curSearching.second;
					forSearching.pop();

					//八连通
					if (ix != 0 && iy != 0 && *(SrcPtr + (ix-1)*step + (iy-1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix-1, iy-1));
						ob.push_back(Point(iy-1, ix-1));
						*(SrcPtr + (ix - 1)*step + (iy - 1)) = 128;
					}
					if (ix != 0 && *(SrcPtr + (ix - 1)*step + (iy - 0)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix-1, iy));
						ob.push_back(Point(iy, ix - 1));
						*(SrcPtr + (ix - 1)*step + (iy - 0)) = 128;
					}
					if (ix != 0 && iy != lsrc.cols && *(SrcPtr + (ix - 1)*step + (iy + 1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix-1, iy+1));
						ob.push_back(Point(iy+1, ix-1));
						*(SrcPtr + (ix - 1)*step + (iy + 1)) = 128;
					}
					if (iy != 0 && *(SrcPtr + (ix - 0)*step + (iy - 1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix, iy-1));
						ob.push_back(Point(iy - 1, ix-0));
						*(SrcPtr + (ix - 0)*step + (iy - 1)) = 128;
					}
					if (iy != lsrc.cols && *(SrcPtr + (ix - 0)*step + (iy + 1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix, iy+1));
						ob.push_back(Point(iy+1, ix));
						*(SrcPtr + (ix - 0)*step + (iy + 1)) = 128;
					}
					if (ix != lsrc.rows&&iy != 0 && *(SrcPtr + (ix + 1)*step + (iy - 1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix + 1, iy-1));
						ob.push_back(Point(iy - 1, ix+1));
						*(SrcPtr + (ix + 1)*step + (iy - 1)) = 128;
					}
					if (ix != lsrc.rows && *(SrcPtr + (ix + 1)*step + (iy )) == 255)
					{
						forSearching.push(std::pair<int, int>(ix+1, iy));
						ob.push_back(Point(iy, ix+1));
						*(SrcPtr + (ix + 1)*step + (iy)) = 128;
					}
					if (ix != lsrc.rows && iy != lsrc.cols && *(SrcPtr + (ix + 1)*step + (iy + 1)) == 255)
					{
						forSearching.push(std::pair<int, int>(ix+1, iy+1));
						ob.push_back(Point(iy+1, ix+1));
						*(SrcPtr + (ix + 1)*step + (iy + 1)) = 128;
					}
				}
				obArr.push_back(ob);
			}
		}
	}
	imshow("bin", lsrc);
}

void CCvExtendedAlgorithm::DrawRotateRect(cv::Mat &src, RotatedRect rect, Scalar color, int thickness, int lineType, bool byPoint)
{
	Point2f vertices[4];
	if (byPoint == true)
	{
		rect.points(vertices);
	}
	else
	{
		double	rectAngle = rect.angle*CV_PI / 180.0;
		double  ang =atan(((rect.size.height < rect.size.width) ? rect.size.height / rect.size.width : rect.size.width / rect.size.height));
		double	len = sqrt(rect.size.width*rect.size.width + rect.size.height*rect.size.height)/2.0;
		vertices[0].x = rect.center.x + len*cos(rectAngle+ang);
		vertices[0].y = rect.center.y + len*sin(rectAngle+ang);
		vertices[1].x = rect.center.x + len*cos(rectAngle-ang);
		vertices[1].y = rect.center.y + len*sin(rectAngle-ang);
		vertices[2].x = rect.center.x + len*cos(rectAngle-CV_PI+ang);
		vertices[2].y = rect.center.y + len*sin(rectAngle-CV_PI+ang);
		vertices[3].x = rect.center.x + len*cos(rectAngle + CV_PI - ang);
		vertices[3].y = rect.center.y + len*sin(rectAngle + CV_PI - ang);

		Point temp;
		temp.x = 10 * cos(rectAngle)+rect.center.x;
		temp.y = 10 * sin(rectAngle)+rect.center.y;
		line(src, rect.center, temp, color, 2);

	}
	for (int i = 0; i < 4; i++)
		line(src, vertices[i], vertices[(i + 1) % 4], color, thickness, lineType);

	circle(src, rect.center,3, color);




}

void CCvExtendedAlgorithm::DrawIcon(cv::Mat &src, Mat ico, Point ori)
{
	if (src.type() != ico.type())
	{
		OutputDebugString(L"DrawIcon()：参数ico与参数src类型不一致!\n");
		return;
	}
	else if(!ico.empty())
		ico.copyTo(src(Range(ori.y, ori.y + ico.rows), Range(ori.x, ori.x + ico.cols)));
///	src = ico.clone();
}

int	CCvExtendedAlgorithm::DistanceOf(cv::Point frt, cv::Point snd)
{
	return static_cast<int>(sqrt(pow((double)frt.x - (double)snd.x, 2)+pow((double)frt.y - (double)snd.y,2))+0.5);
}

Point CCvExtendedAlgorithm::CenterOf(cv::Point frt, cv::Point snd)
{
	return Point((frt.x + snd.x) / 2, (frt.y + snd.y) / 2);
}

void CCvExtendedAlgorithm::ConvertToGray(cv::InputArray src, cv::OutputArray dst)
{
	Mat		mSrc = src.getMat();
	dst.create(mSrc.size(), CV_8UC1);
	Mat		mDst = dst.getMat();
	int		c = mSrc.channels();
	if (mSrc.channels() == 1)
			mSrc.copyTo(mDst);
	else if (mSrc.channels() == 3)
		cv::cvtColor(mSrc, mDst, CV_BGR2GRAY);
	else if (mSrc.channels() == 4)
		cv::cvtColor(mSrc, mDst, CV_BGRA2GRAY);
	return;
}

void CCvExtendedAlgorithm::ConvertToRGB(cv::InputArray src, cv::OutputArray dst)
{
	Mat		mSrc = src.getMat();
	dst.create(mSrc.size(), CV_8UC3);
	Mat		mDst = dst.getMat();
	if (mSrc.channels() == 3)
		mDst = mSrc.clone();
	else if (mSrc.channels() == 1)
		cv::cvtColor(mSrc, mDst, CV_GRAY2BGR);
	else if (mSrc.channels() == 4)
		cv::cvtColor(mSrc, mDst, CV_BGRA2BGR);
}

double CCvExtendedAlgorithm::CrossCorrelation(const cv::Mat &src,const cv::Mat &dst)
{
	Mat		re(1, 1, CV_32FC1);
	Mat		gSrc, gDst;
	CCvExtendedAlgorithm::ConvertToGray(src, gSrc);
	CCvExtendedAlgorithm::ConvertToGray(dst, gDst);
	resize(gDst, gDst, gSrc.size());
	matchTemplate(gSrc, gDst, re, CV_TM_CCORR_NORMED);
 	return *(double*)re.data;
}

int CCvExtendedAlgorithm::LabelAllFoundContours(cv::Mat src,
	cv::Mat &dst, cv::Scalar color, int thickness, cv::Point drawOffset, cv::Point findOffset, int mode, int method)
{
	if (src.channels() != 1)
		return -1;
	vector<Vec4i> hierarchy;
	vector<vector<cv::Point>> contours;
	cv::threshold(src, src, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::findContours(src, contours, hierarchy, mode, method, findOffset);
	if (contours.size() > 1)
	{
		int idx = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
			drawContours(dst, contours, idx, color, thickness, 8, hierarchy);
	}


	return contours.size();
}

double CCvExtendedAlgorithm::CalcLineDegree(const Point2f &pt1, const Point2f &pt2)
{
	double		lAngle(0.0f);
	if (pt2.x - pt1.x != 0)
	{

		double cy = static_cast<double>(pt1.y - pt2.y);
		double cx = static_cast<double>(pt1.x - pt2.x);

		lAngle = atan( cy/cx )*180.0 / CV_PI;
		if (lAngle < 0)
			lAngle += 180;
	}
	else
		lAngle = 90.0;
	return lAngle;
}

double CCvExtendedAlgorithm::CalcRotatedRectDegree(const cv::RotatedRect rot, ROTATERECT_DIRECTION dir
	,int *pRef)
{
	Point2f		vertVect[4];
	rot.points(vertVect);
	const	double firstDegree = (vertVect[1].x - vertVect[0].x)*(vertVect[1].x - vertVect[0].x)
		+ (vertVect[1].y - vertVect[0].y)*(vertVect[1].y - vertVect[0].y);
	const	double secondDegree = (vertVect[2].x - vertVect[1].x)*(vertVect[2].x - vertVect[1].x)
		+ (vertVect[2].y - vertVect[1].y)*(vertVect[2].y - vertVect[1].y);

	double		ang(0.0);
	if (firstDegree > secondDegree)
	{
		ang = CalcLineDegree(vertVect[1], vertVect[0]);
		if (pRef != nullptr)
			if (ang > rot.angle - 0.1 && ang < rot.angle)
				*pRef = 0;
			else
				*pRef = 1;
	}
	else
	{
		ang = CalcLineDegree(vertVect[2], vertVect[1]);
		if (pRef != nullptr)
			if (ang > rot.angle - 0.1 && ang < rot.angle)
				*pRef = 1;
			else
				*pRef = 0;
	}

	if (dir == UP_LEFT || dir == UP_RIGHT)
	{
		ang += 180;
	}
	//if (dir == DOWN_LEFT || dir == DOWN_RIGHT)
	//{
	//	ang += 180;
	//}

	return ang;
}

RotatedRect CCvExtendedAlgorithm::MoveRotate(const cv::RotatedRect& rot, const Point2f &offset
	, const double& angle, const double & angleToCenter, const cv::Size sz, ROTATERECT_DIRECTION dir)
{
	RotatedRect		replace(rot);
	int				ref(-1);
//	double			rectAngle = CCvExtendedAlgorithm::CalcRotatedRectDegree(rot, dir, &ref);

	replace.angle = CCvExtendedAlgorithm::CalcRotatedRectDegree(rot, dir) + angle;
	double offsetDistance = sqrtf(pow(offset.x, 2) + pow(offset.y, 2));

	//switch (dir)
	//{
	//case:UP_LEFT

	//	break;
	//case:UP_RIGHT

	//	break;
	//case:DOWN_LEFT
	//	replace.center.x = rot.center.x - offsetDistance*cos((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot) - angleToCenter)*CV_PI / 180.0);
	//	replace.center.y = rot.center.y - offsetDistance*sin((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot) - angleToCenter)*CV_PI / 180.0);
	//	break;
	//case:DOWN_RIGHT
	//	replace.center.x = rot.center.x - offsetDistance*cos((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot) - angleToCenter)*CV_PI / 180.0);
	//	replace.center.y = rot.center.y - offsetDistance*sin((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot) - angleToCenter)*CV_PI / 180.0);
	//	break;
	//default:
	//	break;
	//}

	replace.center.x = rot.center.x - offsetDistance*cos((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot, dir) - angleToCenter)*CV_PI / 180.0);
	replace.center.y = rot.center.y - offsetDistance*sin((CCvExtendedAlgorithm::CalcRotatedRectDegree(rot, dir) - angleToCenter)*CV_PI / 180.0);

	//if (dir == UP_LEFT)
	//{

	//}
	//else if (dir == UP_RIGHT)
	//{
	//	replace.center.x += offset.x;
	//	replace.center.y += offset.y;
	//}
	//else if (dir == DOWN_LEFT)
	//{
	//	replace.center.x += offset.x;
	//	replace.center.y -= offset.y;
	//}
	//if (dir == DOWN_RIGHT)
	//{
	//	replace.center -= offset;
	//}


	if (sz != Size())
		replace.size = sz;
	return replace;
}

double CCvExtendedAlgorithm::CompareImage(const cv::Mat &src, const cv::Mat &dst)
{
	Mat			SrcSimple, DstSimple;
	CCvExtendedAlgorithm::ConvertToGray(src, SrcSimple);
	CCvExtendedAlgorithm::ConvertToGray(dst, DstSimple);
	resize(SrcSimple, SrcSimple, cv::Size(64, 64));
	resize(DstSimple, DstSimple, cv::Size(64, 64));

	//equalizeHist(SrcSimple, SrcSimple);
	//equalizeHist(DstSimple, DstSimple);
	//GaussianBlur(SrcSimple, SrcSimple, Size(5, 5), 1.0);
	//GaussianBlur(DstSimple, DstSimple, Size(5, 5), 1.0);
	double		SrcMean = cv::mean(SrcSimple)[0];
	double		DstMean = cv::mean(DstSimple)[0];
	threshold(SrcSimple, SrcSimple, SrcMean, 255, CV_THRESH_BINARY);
	threshold(DstSimple, DstSimple, DstMean, 255, CV_THRESH_BINARY);
	Mat			Diff;
	cv::absdiff(SrcSimple, DstSimple, Diff);
	Mat			kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(Diff, Diff, MORPH_OPEN, kernel);
	int			nNoneZero = cv::countNonZero(Diff);
//	return		double(nNoneZero) / (64 * 64);
	Mat			tm(1, 1, CV_32FC1);
	matchTemplate(SrcSimple, DstSimple, tm, CV_TM_CCORR_NORMED);
	return	*(double*)tm.data;
}

ROTATERECT_DIRECTION CCvExtendedAlgorithm::CalcRotateDirection(const vector<Point> &pointSet)
{
	RotatedRect rect = minAreaRect(pointSet);
	Moments		m = moments(pointSet);
	float		cx = m.m10 / m.m00;
	float		cy = m.m01 / m.m00;

	Point2f		vertVect[4];
	rect.points(vertVect);
	const	double firstDegree = (vertVect[1].x - vertVect[0].x)*(vertVect[1].x - vertVect[0].x)
		+ (vertVect[1].y - vertVect[0].y)*(vertVect[1].y - vertVect[0].y);
	const	double secondDegree = (vertVect[2].x - vertVect[1].x)*(vertVect[2].x - vertVect[1].x)
		+ (vertVect[2].y - vertVect[1].y)*(vertVect[2].y - vertVect[1].y);
	
	Point2f pt1, pt2,ptNear;

	if (firstDegree <= secondDegree)
	{
		pt1.x = (vertVect[0].x + vertVect[1].x) / 2.0;
		pt1.y = (vertVect[0].y + vertVect[1].y) / 2.0;
		pt2.x = (vertVect[2].x + vertVect[3].x) / 2.0;
		pt2.y = (vertVect[2].y + vertVect[3].y) / 2.0;
	}
	else
	{
		pt1.x = (vertVect[2].x + vertVect[1].x) / 2.0;
		pt1.y = (vertVect[2].y + vertVect[1].y) / 2.0;
		pt2.x = (vertVect[0].x + vertVect[3].x) / 2.0;
		pt2.y = (vertVect[0].y + vertVect[3].y) / 2.0;
	}

	float distance1 = (pt1.x - cx)*(pt1.x - cx) + (pt1.y - cy)*(pt1.y - cy);
	float distance2 = (pt2.x - cx)*(pt2.x - cx) + (pt2.y - cy)*(pt2.y - cy);

	if (distance1 <= distance2)
	{
		ptNear = pt1;
	}
	else
	{
		ptNear = pt2;
	}

	if (ptNear.x > rect.center.x && ptNear.y > rect.center.y)
		return DOWN_RIGHT;
	else if (ptNear.x > rect.center.x && ptNear.y < rect.center.y)
		return UP_RIGHT;
	else if (ptNear.x < rect.center.x && ptNear.y  < rect.center.y)
		return UP_LEFT;
	else if (ptNear.x < rect.center.x && ptNear.y  > rect.center.y)
		return DOWN_LEFT;
	else
		return DONT_CARE;
}

int CCvExtendedAlgorithm::FindContours(const cv::Mat Src, vector<vector<Point>>& contours)
{
	if (Src.channels() > 1)
		return 0;
	uchar	*pData = nullptr;
	for (int i = 0; i < Src.rows; i++)
	{
		pData = (uchar*)Src.ptr<uchar>(i);
		for (int j = 0; j < Src.cols; j++)
		{

		}
	}
	return contours.size();
}

cv::Scalar CCvExtendedAlgorithm::GetRandColor()
{
	return cv::Scalar(uchar(CCvExtendedAlgorithm::sm_clrRNG.next())
		, uchar(CCvExtendedAlgorithm::sm_clrRNG.next())
		, uchar(CCvExtendedAlgorithm::sm_clrRNG.next()));
}

double CCvExtendedAlgorithm::HullLength(const vector<Point>& hull)
{
	double	len(0.0);
	size_t	cnt = hull.size();
	for (int i = 0; i < cnt - 1; i++)
	{
		len += CCvExtendedAlgorithm::DistanceOf(hull[i], hull[i + 1]);
	}
	len += CCvExtendedAlgorithm::DistanceOf(hull[0], hull[cnt - 1]);
	return len;
}

void CCvExtendedAlgorithm::MarkPoint(Mat & src, Point cen, int siz,char shape,Scalar col, int nlinewidth)
{
	switch (shape)
	{
	case 'o':
		circle(src, cen, siz, col, nlinewidth);
		break;
	case '+':
		line(src, Point(cen.x - siz, cen.y), Point(cen.x + siz - 1), col, nlinewidth);
		line(src, Point(cen.x, cen.y - siz), Point(cen.x, cen.y + siz - 1), col, nlinewidth);
		break;
	case 'x':
		line(src, Point(cen.x - siz, cen.y - siz), Point(cen.x + siz, cen.y + siz), col, nlinewidth);
		line(src, Point(cen.x - siz, cen.y + siz), Point(cen.x + siz, cen.y - siz), col, nlinewidth);
		break;
	case '.':
	default:
		circle(src, cen, siz, col, -1);
		break;
		break;
	}
}


