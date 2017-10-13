#include "stdafx.h"
#include "DetectData.h"



CString TDetectData::sendData = L"1; 0; 0.0; 0.0; 0.0; $";
bool TDetectData::bTrigger=false;
HANDLE  TDetectData::eImageProcessed = CreateEvent(NULL, true, false, L"");
TDetectData::TDetectData()
{
}


TDetectData::~TDetectData()
{
}
