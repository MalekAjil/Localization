// mfcLocalizationDlg.h : header file
//

#pragma once
#include "afxcmn.h"
#include "afxwin.h"
#include <process.h>
#include <windows.h>


// CmfcLocalizationDlg dialog
class CmfcLocalizationDlg : public CDialogEx
{
// Construction
public:
	CmfcLocalizationDlg(CWnd* pParent = NULL);	// standard constructor
	static DWORD WINAPI StaticThreadStart(void* Param);
	DWORD ThreadStart(void);
	HANDLE startMyThread();
	UINT Move(LPVOID param);
// Dialog Data
	enum { IDD = IDD_MFCLOCALIZATION_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CString Rip;
	CString msg;
	CTabCtrl idc;
	int X1;
	int Y1;
	int X2;
	int Y2;
	int X3;
	int Y3;
	int X4;
	int Y4;
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButtongo();
	afx_msg void OnBnClickedButtonstop();
};
