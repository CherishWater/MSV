#pragma once


class TDetectData
{

public:
	TDetectData();
	~TDetectData();
	//static int iIndex;
	static CString sendData;
	static bool bTrigger;
	static bool bGrabFinished;
	static HANDLE	eImageProcessed;
public:
};

