#pragma once
// CSocketServer command target
//#include "VisionProjectView.h"
#include "afxsock.h"
#include"string.h"



class CSocketServerEx : public CSocket
{
public:
	CSocketServerEx();
	virtual ~CSocketServerEx();
public:
	virtual void OnReceive(int nErrorCode);
	virtual void OnAccept(int nErrorCode);
	virtual void OnClose(int nErrorCode);
public:
	void SendInfo2Client(CString info);
	void SendInfo2Client(LPCTSTR lpBuffer,int nBuffer,int nFlags);
	
private:
	std::string WChar2Ansi(LPCWSTR pwszSrc);
public:
	static CList<CSocketServerEx*, CSocketServerEx*> m_ServerExlist;

};


