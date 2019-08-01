//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <vector>
#include <cstdlib>
#include <d2d1.h>
#include <Dwrite.h>
#include <DirectXMath.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <fstream>


using namespace std;

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

//imi::GestureStatus *aGestureStatus[6];
//imi::ges GesArr;
imi::skel SkelArr_B;
imi::xyskel xySkelArr;

//Global
extern string IP;
extern bool flag_skel; extern bool flag_ges;
extern float SkeletonDistance;

ofstream myfile;
extern TCHAR szBuf;
// <summary>
// Entry point for the application
// </summary>
// <param name="hInstance">handle to the application instance</param>
// <param name="hPrevInstance">always 0</param>
// <param name="lpCmdLine">command line arguments</param>
// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
// <returns>status</returns>
//int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
//{
//    UNREFERENCED_PARAMETER(hPrevInstance);
//    UNREFERENCED_PARAMETER(lpCmdLine);
//
//    CBodyBasics application;
//    application.Run(hInstance, nCmdShow);
//}

/// <summary>
/// Constructor
/// </summary>
/*
CBodyBasics::Wave::Wave()
{
	result = 0;
	frameCount = 0;
	currentSegment = 0;
	position = 0;
	temp = 0, temp1 = 0;

}

CBodyBasics::L_Wave::L_Wave()
{
	L_result = 0;
	L_frameCount = 0;
	L_currentSegment = 0;
	L_position = 0;
	L_temp = 0, L_temp1 = 0;

}
*/
CBodyBasics::CBodyBasics() :
m_hWnd(NULL),
m_nStartTime(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0),
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pBodyFrameReader(NULL),
m_pD2DFactory(NULL),
m_pRenderTarget(NULL),
m_pBrushJointTracked(NULL),
m_pBrushJointInferred(NULL),
m_pBrushBoneTracked(NULL),
m_pBrushBoneInferred(NULL),
m_pBrushHandClosed(NULL),
m_pBrushHandOpen(NULL),
m_pBrushHandLasso(NULL)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	//ges_success_id = 0;
	//wave_users.reserve(6);

	//L_ges_success_id = 0;
	//L_wave_users.reserve(6);

	SkelArr_B.reserve(6);
	//GesArr.reserve(6);
	xySkelArr.reserve(6);

}


/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
	DiscardDirect2DResources();

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);

//	while (wave_users.size() > 0)//delete users that are gone
	//{
	//	Wave *temp = wave_users.back();
	//	wave_users.pop_back();
	//	delete temp;
	//}
	myfile.close();
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
	MSG       msg = { 0 };
	WNDCLASS  wc;

	//i2p connection

	m_client = new ProtectedClient<imi::UserTrackingServiceNewKinectClient>(IP, imi::g_Inputs_constants.DEFAULT_USERTRACKING_KINECT2_SERVICE_PORT);

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP),
		NULL,
		(DLGPROC)CBodyBasics::MessageRouter,
		reinterpret_cast<LPARAM>(this));


	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		//Sleep(50);
		Update();

		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{
	if (!m_pBodyFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);


	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;

		hr = pBodyFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr))
		{
			ProcessBody(nTime, BODY_COUNT, ppBodies);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{

	CBodyBasics* pThis = NULL;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<CBodyBasics*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
	{
						  // Bind application window handle
						  m_hWnd = hWnd;

						  // Init Direct2D
						  D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

						  // Get and initialize the default Kinect sensor
						  InitializeDefaultSensor();
	}
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		break;

	case WM_DESTROY:
		// Quit the main message pump
		ExitProcess(0);
		PostQuitMessage(0);
		break;
	}

	return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		SafeRelease(pBodyFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		SetStatusMessage(L"No ready Kinect found!", 10000, true);
		return E_FAIL;
	}

	return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
	//myfile.open("output.txt");
	//myfile << "Writing this to a file.\n";

	//OutputDebugStringW(L"My output string \n ");

	imi::Skeleton *aSkeleton[6];

	//int res[4];

	if (m_hWnd)
	{
		HRESULT hr = EnsureDirect2DResources();

		if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
		{
			m_pRenderTarget->BeginDraw();
			m_pRenderTarget->Clear();

			RECT rct;
			GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
			int width = rct.right;
			int height = rct.bottom;

			for (int i = 0; i < nBodyCount; ++i)
			{
				aSkeleton[i] = new imi::Skeleton();

				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count];
						D2D1_POINT_2F jointPoints[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;



						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);

						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							//JointType_SpineBase//
							aSkeleton[i]->spinebase.x = joints[0].Position.X; aSkeleton[i]->spinebase.y = joints[0].Position.Y; aSkeleton[i]->spinebase.z = joints[0].Position.Z;
							//JointType_SpineMid//
							aSkeleton[i]->spinemid.x = joints[1].Position.X; aSkeleton[i]->spinemid.y = joints[1].Position.Y; aSkeleton[i]->spinemid.z = joints[1].Position.Z;
							//JointType_Neck//
							aSkeleton[i]->neck.x = joints[2].Position.X; aSkeleton[i]->neck.y = joints[2].Position.Y; aSkeleton[i]->neck.z = joints[2].Position.Z;
							//JointType_Head//
							aSkeleton[i]->head.x = joints[3].Position.X; aSkeleton[i]->head.y = joints[3].Position.Y; aSkeleton[i]->head.z = joints[3].Position.Z;
							//JointType_ShoulderLeft//
							aSkeleton[i]->shoulderl.x = joints[4].Position.X; aSkeleton[i]->shoulderl.y = joints[4].Position.Y; aSkeleton[i]->shoulderl.z = joints[4].Position.Z;
							//JointType_ElbowLeft//
							aSkeleton[i]->elbowl.x = joints[5].Position.X; aSkeleton[i]->elbowl.y = joints[5].Position.Y; aSkeleton[i]->elbowl.z = joints[5].Position.Z;
							//JointType_WristLeft
							aSkeleton[i]->wristl.x = joints[6].Position.X; aSkeleton[i]->wristl.y = joints[6].Position.Y; aSkeleton[i]->wristl.z = joints[6].Position.Z;
							//JointType_HandLeft
							aSkeleton[i]->handl.x = joints[7].Position.X; aSkeleton[i]->handl.y = joints[7].Position.Y; aSkeleton[i]->handl.z = joints[7].Position.Z;
							//JointType_ShoulderRight
							aSkeleton[i]->shoulderr.x = joints[8].Position.X; aSkeleton[i]->shoulderr.y = joints[8].Position.Y; aSkeleton[i]->shoulderr.z = joints[8].Position.Z;
							//JointType_ElbowRight
							aSkeleton[i]->elbowr.x = joints[9].Position.X; aSkeleton[i]->elbowr.y = joints[9].Position.Y; aSkeleton[i]->elbowr.z = joints[9].Position.Z;
							//JointType_WristRight
							aSkeleton[i]->wristr.x = joints[10].Position.X; aSkeleton[i]->wristr.y = joints[10].Position.Y; aSkeleton[i]->wristr.z = joints[10].Position.Z;
							//JointType_HandRight
							aSkeleton[i]->handr.x = joints[11].Position.X; aSkeleton[i]->handr.y = joints[11].Position.Y; aSkeleton[i]->handr.z = joints[11].Position.Z;
							//JointType_HipLeft
							aSkeleton[i]->hipl.x = joints[12].Position.X;	aSkeleton[i]->hipl.y = joints[12].Position.Y; aSkeleton[i]->hipl.z = joints[12].Position.Z;
							//JointType_KneeLeft
							aSkeleton[i]->kneel.x = joints[13].Position.X; aSkeleton[i]->kneel.y = joints[13].Position.Y; aSkeleton[i]->kneel.z = joints[13].Position.Z;
							//JointType_AnkleLeft
							aSkeleton[i]->anklel.x = joints[14].Position.X; aSkeleton[i]->anklel.y = joints[14].Position.Y; aSkeleton[i]->anklel.z = joints[14].Position.Z;
							//JointType_FootLeft
							aSkeleton[i]->footl.x = joints[15].Position.X; aSkeleton[i]->footl.y = joints[15].Position.Y; aSkeleton[i]->footl.z = joints[15].Position.Z;
							//JointType_HipRight
							aSkeleton[i]->hipr.x = joints[16].Position.X; aSkeleton[i]->hipr.y = joints[16].Position.Y; aSkeleton[i]->hipr.z = joints[16].Position.Z;
							//JointType_KneeRight	= 17,
							aSkeleton[i]->kneer.x = joints[17].Position.X; aSkeleton[i]->kneer.y = joints[17].Position.Y; aSkeleton[i]->kneer.z = joints[17].Position.Z;
							//JointType_AnkleRight	= 18,
							aSkeleton[i]->ankler.x = joints[18].Position.X; aSkeleton[i]->ankler.y = joints[18].Position.Y; aSkeleton[i]->ankler.z = joints[18].Position.Z;
							//JointType_FootRight	= 19,
							aSkeleton[i]->footr.x = joints[19].Position.X; aSkeleton[i]->footr.y = joints[19].Position.Y; aSkeleton[i]->footr.z = joints[19].Position.Z;
							//JointType_SpineShoulder	= 20,
							aSkeleton[i]->spines.x = joints[20].Position.X; aSkeleton[i]->spines.y = joints[20].Position.Y; aSkeleton[i]->spines.z = joints[20].Position.Z;
							//JointType_HandTipLeft	= 21,
							aSkeleton[i]->handtipl.x = joints[21].Position.X; aSkeleton[i]->handtipl.y = joints[21].Position.Y; aSkeleton[i]->handtipl.z = joints[21].Position.Z;
							//JointType_ThumbLeft	= 22,
							aSkeleton[i]->thumbl.x = joints[22].Position.X; aSkeleton[i]->thumbl.y = joints[22].Position.Y; aSkeleton[i]->thumbl.z = joints[22].Position.Z;
							//JointType_HandTipRight	= 23,
							aSkeleton[i]->handtipr.x = joints[23].Position.X; aSkeleton[i]->handtipr.y = joints[23].Position.Y; aSkeleton[i]->handtipr.z = joints[23].Position.Z;
							//JointType_ThumbRight	= 24,
							aSkeleton[i]->thumbr.x = joints[24].Position.X; aSkeleton[i]->thumbr.y = joints[24].Position.Y; aSkeleton[i]->thumbr.z = joints[24].Position.Z;

							if (rightHandState == 0)
							{
								aSkeleton[i]->rhandstate = imi::HandPossibilities::Unknown;
							}
							else if (rightHandState == 1)
							{
								aSkeleton[i]->rhandstate = imi::HandPossibilities::NotTracked;
							}
							else if (rightHandState == 2)
							{
								aSkeleton[i]->rhandstate = imi::HandPossibilities::Open;
							}
							else if (rightHandState == 3)
							{
								aSkeleton[i]->rhandstate = imi::HandPossibilities::Closed;
							}
							else if (rightHandState == 4)
							{
								aSkeleton[i]->rhandstate = imi::HandPossibilities::Lasso;
							}

							//Left
							if (leftHandState == 0)
							{
								aSkeleton[i]->lhandstate = imi::HandPossibilities::Unknown;
							}
							else if (leftHandState == 1)
							{
								aSkeleton[i]->lhandstate = imi::HandPossibilities::NotTracked;
							}
							else if (leftHandState == 2)
							{
								aSkeleton[i]->lhandstate = imi::HandPossibilities::Open;
							}
							else if (leftHandState == 3)
							{
								aSkeleton[i]->lhandstate = imi::HandPossibilities::Closed;
							}
							else if (leftHandState == 4)
							{
								aSkeleton[i]->lhandstate = imi::HandPossibilities::Lasso;
							}

							//Store in array for Body Basics

							if (aSkeleton[i]->head.z <= SkeletonDistance)
							{
								SkelArr_B.push_back(*aSkeleton[i]);
							}

							//Wave

							for (int j = 0; j < _countof(joints); ++j)
							{
								jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
							}

							DrawBody(joints, jointPoints);

							DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
							DrawHand(rightHandState, jointPoints[JointType_HandRight]);

						}
					}
				}
			}
			//Release//
			for (int r = 0; r < 6; r++)
			{
				delete aSkeleton[r];
			}
			//i2p//
			try
			{
				if (flag_skel == true)
				{
					m_client->getClient()->MultiUserSkeletonChanged("KinectV2", imi::getTimeStamp(), SkelArr_B, xySkelArr);
				}
				if (flag_ges == true)
				{
					//WaveHandGesture();
					//L_WaveHandGesture();
					//m_client->getClient()->gestureStart("KinectV2", imi::getTimeStamp(), GesArr);
				}
				//GesArr.clear();
				SkelArr_B.clear();
			}
			catch (apache::thrift::TException &tx)
			{
				m_client->ensureConnection();
				ExitProcess(0);

			}

			//End of i2p//


			hr = m_pRenderTarget->EndDraw();

			// Device lost, need to recreate the render target
			// We'll dispose it now and retry drawing
			if (D2DERR_RECREATE_TARGET == hr)
			{
				hr = S_OK;
				DiscardDirect2DResources();
			}
		}

		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}
}











/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
	DWORD now = GetTickCount();

	if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
	{
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
	HRESULT hr = S_OK;

	if (m_pD2DFactory && !m_pRenderTarget)
	{
		RECT rc;
		GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);

		int width = rc.right - rc.left;
		int height = rc.bottom - rc.top;
		D2D1_SIZE_U size = D2D1::SizeU(width, height);
		D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
		rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
		rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

		// Create a Hwnd render target, in order to render to the window set in initialize
		hr = m_pD2DFactory->CreateHwndRenderTarget(
			rtProps,
			D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
			&m_pRenderTarget
			);

		if (FAILED(hr))
		{
			SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
			return hr;
		}

		// light green
		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
		m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
	}

	return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
	SafeRelease(m_pRenderTarget);

	SafeRelease(m_pBrushJointTracked);
	SafeRelease(m_pBrushJointInferred);
	SafeRelease(m_pBrushBoneTracked);
	SafeRelease(m_pBrushBoneInferred);

	SafeRelease(m_pBrushHandClosed);
	SafeRelease(m_pBrushHandOpen);
	SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

	return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
	// Draw the bones

	// Torso
	DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
	DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
	DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
	DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
	DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{
		D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
		}
	}
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
	}
	else
	{
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
	}
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
	D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

	switch (handState)
	{
	case HandState_Closed:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
		break;

	case HandState_Open:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
		break;

	case HandState_Lasso:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
		break;
	}
}


////Draw id:
//for(int i = 0; i<SkelArr.size() ; i++)
//{
//std::wstring user_id = L"";
//user_id += std::to_wstring(i);

//DepthSpacePoint depthPoint = {0};
//m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[3].Position, &depthPoint);

//float sPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
//float sPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

//ID2D1SolidColorBrush  *m_whiteBrush, *m_pYellowBrush, *m_redBrush , *m_greenBrush, *m_blueBrush, *m_pinkBrush;

//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White, 1.0f), &m_whiteBrush );
//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pYellowBrush );
//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 1.0f), &m_redBrush );
//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_greenBrush );
//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 1.0f), &m_blueBrush );
//hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::HotPink, 1.0f), &m_pinkBrush );


//D2D1_ELLIPSE ellipse = D2D1::Ellipse(D2D1::Point2F(sPointX, sPointY), 30.0F, 30.0F);
//if(i==0)
//{m_pRenderTarget->FillEllipse(ellipse, m_whiteBrush);}
//else if(i==1)
//{m_pRenderTarget->FillEllipse(ellipse, m_pYellowBrush);}
//else if(i==2)
//{m_pRenderTarget->FillEllipse(ellipse, m_redBrush);}
//else if(i==3)
//{m_pRenderTarget->FillEllipse(ellipse, m_greenBrush);}
//else if(i==4)
//{m_pRenderTarget->FillEllipse(ellipse, m_blueBrush);}
//else if(i==5)
//{m_pRenderTarget->FillEllipse(ellipse, m_pinkBrush);}
//
//}//Draw id: