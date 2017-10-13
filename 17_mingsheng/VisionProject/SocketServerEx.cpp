// SocketServer.cpp : implementation file
//

#include "stdafx.h"

#include "SocketServerEx.h"

// CSocketServer

CSocketServerEx::CSocketServerEx()
{
	AfxSocketInit();
	m_nTimeOut = 0;
}
CSocketServerEx::~CSocketServerEx()
{
}

CList<CSocketServerEx*, CSocketServerEx*> CSocketServerEx::m_ServerExlist;


string CSocketServerEx::WChar2Ansi(LPCWSTR pwszSrc)
{
	int nLen = WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, NULL, 0, NULL, NULL);
	if (nLen <= 0) return std::string("");
	char* pszDst = new char[nLen];
	if (NULL == pszDst) return std::string("");
	WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, pszDst, nLen, NULL, NULL);
	pszDst[nLen - 1] = 0;
	std::string strTemp(pszDst);
	delete[] pszDst;
	return strTemp;
}

// CSocketServer member functions
void CSocketServerEx::OnAccept(int nErrorCode)
{
	// TODO: Add your specialized code here and/or call the base class
	CSocketServerEx *p = new CSocketServerEx;
	if(!Accept(*p))
	{
		delete p;
		return;
	}
	//
	CString szIP;
	UINT szPort;
	p->GetPeerName(szIP,szPort);
	//connect to robot
	//TShowData::iPort = szPort;
	//TShowData::sConnectIP = WChar2Ansi(szIP.GetBuffer(szIP.GetLength()));
	/*
	tempClientIPAddress.csIP = szIP;
	tempClientIPAddress.iPort = szPort;

	clientInfo.pop_back();
	*/
	//TShowData::bConnectFlag = true;
	//

	if (szIP==L"172.168.0.20")
	{
		m_ServerExlist.AddTail(p);
	}
	else
	{
	}
	
	CSocket::OnAccept(nErrorCode);
}


void CSocketServerEx::SendInfo2Client(LPCTSTR lpBuffer, int nBuffer, int nFlags)
{
	// TODO: Add your specialized code here and/or call the base class
	POSITION pos = m_ServerExlist.GetHeadPosition();
	//CString str=_T("hello");
	while(pos)
	{
		CSocketServerEx* pSock = m_ServerExlist.GetNext(pos);
		pSock->Send(lpBuffer,nBuffer,nFlags);
	}

}
void CSocketServerEx::SendInfo2Client(CString info)
{
	// TODO: Add your specialized code here and/or call the base class
	int nBuffer = info.GetLength();

	//
	int nBytes = WideCharToMultiByte(CP_ACP, 0, info, nBuffer, NULL, 0, NULL, NULL);
	char* pInfo = new char[nBytes + 1];
	memset(pInfo, 0, nBuffer + 1);
	WideCharToMultiByte(CP_OEMCP, 0, info, nBuffer, pInfo, nBytes, NULL, NULL);
	pInfo[nBytes] = 0;
	//
	CString szIP;
	UINT szPort;
 	int nFlags = 0;
	POSITION pos = m_ServerExlist.GetHeadPosition();
	while (pos)
	{
		CSocketServerEx* pSock = m_ServerExlist.GetNext(pos);

		pSock->GetPeerName(szIP, szPort);

		pSock->Send(pInfo, nBuffer, nFlags);
	}

	delete pInfo;
}

void CSocketServerEx::OnReceive(int nErrorCode)
{
	// TODO: Add your specialized code here and/or call the base class
	char s[1024]; CString szIP; UINT szPort;

	GetPeerName(szIP,szPort);
	int nLen = Receive(s,1024);
	s[nLen]=L'\0';
	CString strText(s);
	

	SendInfo2Client(L"run$");

	CSocket::OnReceive(nErrorCode);
}


void CSocketServerEx::OnClose(int nErrorCode)
{
	// TODO: Add your specialized code here and/or call the base class

	POSITION pos = m_ServerExlist.GetHeadPosition();
	while(pos)
	{
		if (m_ServerExlist.GetAt(pos) == this)
		{
			m_ServerExlist.RemoveAt(pos);
			break;
		}
		m_ServerExlist.GetNext(pos);
	}
	delete this;

	
	//
	CSocket::OnClose(nErrorCode);
}
