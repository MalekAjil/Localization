
// mfcLocalizationDlg.cpp : implementation file
//

#include "stdafx.h"
#include "mfcLocalization.h"
#include "mfcLocalizationDlg.h"
#include "afxdialogex.h"

#include "MyRobot.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


MyRobot *rb;
ArRobot *amigo;
HANDLE	threadHandle;
CWinThread* pThread;
int moveThread;
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CmfcLocalizationDlg dialog

CmfcLocalizationDlg::CmfcLocalizationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CmfcLocalizationDlg::IDD, pParent)
	, Rip(_T("localhost"))	
	, msg(_T(""))
	, X1(1500)
	, Y1(0)
	, X2(1500)
	, Y2(1000)
	, X3(0)
	, Y3(1000)
	, X4(0)
	, Y4(0)
	//, myIp(2130706433)	
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CmfcLocalizationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_CBString(pDX, IDC_COMBO1, Rip);
	DDX_Text(pDX, IDC_EDIT3, msg);
	DDX_Text(pDX, IDC_EDIT1, X1);
	DDX_Text(pDX, IDC_EDIT2, Y1);
	DDX_Text(pDX, IDC_EDIT4, X2);
	DDX_Text(pDX, IDC_EDIT5, Y2);
	DDX_Text(pDX, IDC_EDIT8, X3);
	DDX_Text(pDX, IDC_EDIT6, Y3);
	DDX_Text(pDX, IDC_EDIT9, X4);
	DDX_Text(pDX, IDC_EDIT7, Y4);
}

BEGIN_MESSAGE_MAP(CmfcLocalizationDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()	
	ON_CBN_SELCHANGE(IDC_COMBO1, &CmfcLocalizationDlg::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(IDC_BUTTONgo, &CmfcLocalizationDlg::OnBnClickedButtongo)
	ON_BN_CLICKED(IDC_BUTTONstop, &CmfcLocalizationDlg::OnBnClickedButtonstop)
END_MESSAGE_MAP()


// CmfcLocalizationDlg message handlers

BOOL CmfcLocalizationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	moveThread = 0; //initialize at 0 to indicate no thread is running
	amigo = new ArRobot();
	rb = new MyRobot(amigo);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CmfcLocalizationDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CmfcLocalizationDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
		HBRUSH hh;
		hh = CreateSolidBrush(RGB(255, 0, 0));
		FillRect(NULL, CRect(x / 2, y / 2, 20, 20), hh);
		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CmfcLocalizationDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CmfcLocalizationDlg::OnCbnSelchangeCombo1()
{/*
 //localhost(127.0.0.1)==> myIp = 2130706433;
 //Robot(169.254.2.115)==> myIp = 2851996275;
	UpdateData();
	if (Rip == "localhost")
	{
		myIp = 2130706433;
	}
	else
	{
		myIp = 2851996275;
	}	
	UpdateData(false);*/
}

UINT CmfcLocalizationDlg::Move(LPVOID param)
{
	int ip = 0;
	if (Rip == "robot")
	{
		ip = 1;
	}

	if (!rb->connect(ip))
	{
		UpdateData();
		msg = " connected";
		UpdateData(false);

		rb->GoTo(X1, Y1, 1);
		rb->GoTo(X2, Y2, 2);
		rb->GoTo(X3, Y3, 3);
		rb->GoTo(X4, Y4, 4);
		rb->CompletePrint();
		//rb->printFooter();
		rb->disconnect();

		UpdateData();
		msg = " stopped";
		UpdateData(false);
	}
	else
	{
		UpdateData();
		msg = "not connected";
		UpdateData(false);
	}
	return 0;
}

DWORD WINAPI CmfcLocalizationDlg::StaticThreadStart(void* Param)
{
	CmfcLocalizationDlg* This = (CmfcLocalizationDlg*)Param;
	return This->ThreadStart();
}

DWORD CmfcLocalizationDlg::ThreadStart(void)
{
		rb->GoTo(X1, Y1, 1);
		rb->GoTo(X2, Y2, 2);
		rb->GoTo(X3, Y3, 3);
		rb->GoTo(X4, Y4, 4);
		//rb->CompletePrint();
		rb->disconnect();
		
	return 0;
}

HANDLE CmfcLocalizationDlg::startMyThread()
{
	DWORD ThreadID;
	threadHandle = CreateThread(NULL, 0, StaticThreadStart, (void*) this, 0, &ThreadID);
	
	UpdateData();
	msg = " stopped";
	UpdateData(false);
	return threadHandle;
}

void CmfcLocalizationDlg::OnBnClickedButtongo()
{
	// TODO: Add your control notification handler code here
	/*
	HWND hWnd = GetSafeHwnd();
	if (moveThread == 0)
	{
	pThread = AfxBeginThread(Move, hWnd, THREAD_PRIORITY_NORMAL);
	moveThread = 1; //1 = thread running
	}
	else if (moveThread == 2)
	{
	pThread->ResumeThread();
	moveThread = 1; //1 = thread running
	}
	else
	{
	pThread->SuspendThread();
	moveThread = 2; //2 = thread suspended
	rb->stop();
	}
	*/
	int ip = 0;
	if (Rip == "robot")
	{
		ip = 1;
	}
	if (!rb->connect(ip))
	{
		UpdateData();
		msg = " connected";
		UpdateData(false);

		startMyThread();

		UpdateData();
		msg = " Running ";
		UpdateData(false);

	}
	else
	{
		UpdateData();
		msg = "not connected";
		UpdateData(false);
	}
}


void CmfcLocalizationDlg::OnBnClickedButtonstop()
{
	// TODO: Add your control notification handler code here
	/*
	UpdateData();
	//	pThread->Delete();
	moveThread = 0;
	rb->stop();
	UpdateData(false);*/

	//rb->CompletePrint();
	if (rb->is_Connected())	rb->disconnect();

	UpdateData();
	msg = " stopped";
	UpdateData(false);
}
