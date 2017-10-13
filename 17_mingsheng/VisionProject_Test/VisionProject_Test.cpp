// VisionProject_Test.cpp : 定义控制台应用程序的入口点。
//


#include "stdafx.h"
#include "ZjlInfusionBagDetect.hpp"



int main()
{
	ZjlMedicalBagIdentify zjlMedicalBagIdentify;
	for (int i = 1; i < 8; i++)
	{



		string imName;
		char ch[8];
		imName.append("");
		_itoa(i, ch, 10);
		imName.append(ch);
		imName.append(".bmp");


		Mat src = imread(imName);

		//imshow("src", src);
		zjlMedicalBagIdentify.imDetect(src);
		Mat result = zjlMedicalBagIdentify.m_imResult;
		imshow("result", result);

		waitKey();
	}
	//system("pause");
	return 0;
}

