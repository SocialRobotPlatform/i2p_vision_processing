

//------------------------------------------------------------------------------
// <copyright file="FaceBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "FaceBasics.h"
//#include "C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc\Kinect.Face.h"
#include <vector>
#include <cstdlib>
#include <iostream>     
#include <fstream>
#include <string>
#include <windows.h>
#include "Facebook_server.skeleton.cpp"
#include "NewSpeechRecognitionService_server.cpp"
using namespace std;
//rubha-handshake
double w1, w2, w3, w4, w5, w6;
//double x_Min = -0.14, x_Max = 0.15, y_Min = -0.58, y_Max = -0.42, z_Max = 1.40, z_Min = 1.00;
double x_Min = -0.14, x_Max = 0.15, y_Min = -0.58, y_Max = -0.35, z_Max = 1.40, z_Min = 1.00;
//ruba- angle
D2D1_POINT_2F pJoints[JointType_Count];
double gr, angle = 1.73, b;
//angle = tan((180 - param) / 2);
//ruba- angle
// fACE DLG//
static const int        nIDEvent = 1;
wstring file_no;
void Write2file();
HWND hWndApp;
//

Mat dst;
//************BODY*******************//
imi::GestureStatus *aGestureStatus[6];
imi::ges GesArr;
imi::skel SkelArr;
//imi::xyskel xySkelArr;


imi::GestureHandShake *aGestureHandShake[6];
imi::gesHandShake gesShakeArr;

//Global
extern string IP;
extern bool flag_skel; extern bool flag_ges;
extern float SkeletonDistance;

imi::Skeleton *aSkeleton[6];
imi::xySkeleton *axySkeleton[6];
//************BODY*******************//

extern string IP;
imi::face faceArr;
Mat hsvImg;
Mat grayImg;
int histSize; double* newUserTemplate;
std::string name, name_arr[21];
extern HWND FacehWndApp; extern HINSTANCE hInst;

// face property text layout offset in X axis
static const float c_FaceTextLayoutOffsetX = -0.1f;

// face property text layout offset in Y axis
static const float c_FaceTextLayoutOffsetY = -0.125f;

// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>

/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
//int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
//{
//    UNREFERENCED_PARAMETER(hPrevInstance);
//    UNREFERENCED_PARAMETER(lpCmdLine);
//
//    CFaceBasics application;
//    application.Run(hInstance, nCmdShow);
//}

/// <summary>
/// Constructor
/// </summary>
//************BODY*******************//
CFaceBasics::Hand_Shake::Hand_Shake()
{
	shake_temp = 0;
	last_position = 0;
}

CFaceBasics::Wave::Wave()
{
	result = 0;
	frameCount = 0;
	currentSegment = 0;
	position = 0;
	temp = 0, temp1 = 0;

}

CFaceBasics::L_Wave::L_Wave()
{
	L_result = 0;
	L_frameCount = 0;
	L_currentSegment = 0;
	L_position = 0;
	L_temp = 0, L_temp1 = 0;

}

//************BODY*******************//
CFaceBasics::CFaceBasics() :
m_hWnd(NULL),
m_nStartTime(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0),
m_pKinectSensor(nullptr),
m_pCoordinateMapper(nullptr),
m_pColorFrameReader(nullptr),
m_pD2DFactory(nullptr),
m_pDrawDataStreams(nullptr),
m_pColorRGBX(nullptr),
m_pBodyFrameReader(nullptr)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	for (int i = 0; i <256; i++)
	{
		m_bKeyDown[i] = FALSE;
		m_bKeyPressed[i] = FALSE;
	}
	m_IsStartRecord = FALSE;
	m_Slow = FALSE;
	newUserTemplateNum = 0;
	newUserTemplate = new double[256 * NUM_OF_PATCH*NUM_OF_PATCH*NUM_OF_USER_TEMPLATE];
	for (int i = 0; i<256 * NUM_OF_PATCH*NUM_OF_PATCH*NUM_OF_USER_TEMPLATE; i++)
		newUserTemplate[i] = 0;

	frame_count = 0;
	face_frame_count = 0;

	//************BODY*******************//
	hand_shake_users.reserve(6);

	ges_success_id = 0;
	wave_users.reserve(6);

	L_ges_success_id = 0;
	L_wave_users.reserve(6);

	SkelArr.reserve(6);
	GesArr.reserve(6);

	//xySkelArr.reserve(6);

	//************BODY*******************//
}


/// <summary>
/// Destructor
/// </summary>
CFaceBasics::~CFaceBasics()
{
	// clean up Direct2D renderer
	if (m_pDrawDataStreams)
	{
		delete m_pDrawDataStreams;
		m_pDrawDataStreams = nullptr;
	}

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = nullptr;
	}

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);

	// done with face sources and readers
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
	//************BODY*******************//
	while (wave_users.size() > 0)//delete users that are gone
	{
		Wave *temp = wave_users.back();
		wave_users.pop_back();
		delete temp;
	}

	while (hand_shake_users.size() > 0)//delete users that are gone
	{
		Hand_Shake *temp = hand_shake_users.back();
		hand_shake_users.pop_back();
		delete temp;
	}
	//************BODY*******************//
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CFaceBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
	m_client = new ProtectedClient<imi::UserTrackingServiceNewKinectClient>(IP, imi::g_Inputs_constants.DEFAULT_USERTRACKING_KINECT2_SERVICE_PORT);
	m_client_facebook = new ProtectedClient<imi::FacebookClient>(IP, imi::g_SocialNetwork_constants.DEFAULT_FACEBOOK_SERVICE_PORT); //14000
	m_client_facebook1 = new ProtectedClient<imi::FacebookClient>(IP, imi::g_SocialNetwork_constants.DEFAULT_GOOGLE_SERVICE_PORT); //14001


	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP1));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"FaceBasicsAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP1),
		NULL,
		(DLGPROC)CFaceBasics::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		//Sleep(10);
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
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CFaceBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	CFaceBasics* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<CFaceBasics*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<CFaceBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
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
LRESULT CALLBACK CFaceBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
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

						  // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
						  // We'll use this to draw the data we receive from the Kinect to the screen
						  m_pDrawDataStreams = new ImageRenderer();
						  HRESULT hr = m_pDrawDataStreams->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
						  if (FAILED(hr))
						  {
							  SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
						  }

						  // Get and initialize the default Kinect sensor
						  InitializeDefaultSensor();
						  InitializeFaceRecognitionModule();

						  ShowWindow((GetDlgItem(m_hWnd, IDC_STATIC4)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_EDIT1)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_BUTTON1)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_BUTTON3)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_EDIT2)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_STATIC_NAME)), SW_HIDE);
						  ShowWindow((GetDlgItem(m_hWnd, IDC_STATIC_GENDER)), SW_HIDE);
						//  SetTimer(m_hWnd, nIDEvent, 500, NULL);
	}
		break;

		//case WM_KEYDOWN:
		//	if(!m_bKeyPressed[wParam])
		//	{
		//		m_bKeyDown[wParam] = true;
		//		m_bKeyPressed[wParam] = true;
		//	}
		//	else
		//		m_bKeyDown[wParam] = false;
		//	break;
		//case WM_KEYUP:
		//	m_bKeyDown[wParam] = false;
		//	m_bKeyPressed[wParam] = false;
		//	break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		ExitProcess(0);
		PostQuitMessage(0);
		break;

	case WM_DESTROY:
		// Quit the main message pump
		ExitProcess(0);
		PostQuitMessage(0);
		break;

	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDC_BUTTON1:

			Write2file();
			break;
		case IDC_BUTTON3:
			SetWindowText(GetDlgItem(hWndApp, IDC_EDIT1), _T(""));
			SetWindowText(GetDlgItem(hWndApp, IDC_EDIT2), _T(""));
			ShowWindow((GetDlgItem(hWndApp, IDC_STATIC4)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_EDIT1)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON1)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON3)), SW_HIDE);
			ShowWindow((GetDlgItem(m_hWnd, IDC_EDIT2)), SW_HIDE);
			ShowWindow((GetDlgItem(m_hWnd, IDC_STATIC_NAME)), SW_HIDE);
			ShowWindow((GetDlgItem(m_hWnd, IDC_STATIC_GENDER)), SW_HIDE);
		}
		break;

	case WM_TIMER:
		if (nIDEvent == 1)
		{
			static BOOL t = TRUE;
			if (t = !t)
				ShowWindow((GetDlgItem(hWnd, IDC_STATIC4)), SW_SHOW);
			else
				ShowWindow((GetDlgItem(hWnd, IDC_STATIC4)), SW_HIDE);
			//IDC_STATIC3.ShowWindow((t = !t) ? SW_SHOW : SW_HIDE);
			//ShowWindow(FacehWndApp, SW_SHOWDEFAULT);
		}
		break;
	}

	return FALSE;
}

void Write2file()
{
	//Read name from edit box//
	TCHAR szBuffer_name[200]; bool isnum = false;
	GetWindowText(GetDlgItem(hWndApp, IDC_EDIT1), szBuffer_name, 200);
	wstring temp_name_name = wstring(szBuffer_name);
	//Read gender from edit box2//
	TCHAR szBuffer_gender[20]; 
	GetWindowText(GetDlgItem(hWndApp, IDC_EDIT2), szBuffer_gender, 20);
	wstring temp_name_gender = wstring(szBuffer_gender);
	//add name+gender
	wstring f_info = temp_name_name + L" , " + temp_name_gender;

	if (temp_name_name.size() == 0 || temp_name_gender.size() == 0)
	{
		MessageBox(hWndApp, _T("Please enter the name and gender"), _T("Help"), MB_OK);
		SetFocus(hWndApp);
	}


	else
	{


		for (int i = 0; i < temp_name_name.size(); i++)
		{
			if (isalpha(szBuffer_name[i]))
				isnum = false;
			else
			{
				isnum = true;
				break;
			}
		}

		if (isnum == true)
		{
			MessageBox(hWndApp, _T("Please enter a valid name"), _T("Help"), MB_OK);
			SetWindowText(GetDlgItem(hWndApp, IDC_EDIT1), _T(""));
			SetFocus(hWndApp);
		}

		if (isnum == false)
		{

			//Read name file and copy contents//
			std::string name1, name_arr1[21], ss;
			ifstream file1("face_recognition_names.txt");
			for (int i = 0; i < NUM_OF_REGISTER_USER + 1; ++i)
			{
				getline(file1, name1);
				name_arr1[i] = name1;
			}
			std::string str1(CW2A(f_info.c_str()));
			int file_number = std::stoi(file_no);
			name_arr1[file_number] = str1;

			//Increment the file_no inside the txt file//
			if (file_number < NUM_OF_REGISTER_USER - 1)
			{
				file_number++;
			}
			else if (file_number == (NUM_OF_REGISTER_USER - 1))
			{
				file_number = 1;
			}
			name_arr1[NUM_OF_REGISTER_USER] = std::to_string(file_number);

			//Save new name into file//
			std::ofstream os("face_recognition_names.txt");
			for (int i = 0; i < NUM_OF_REGISTER_USER + 1; i++)
			{
				os << name_arr1[i];
				os << endl;
			}
			os.close();

			//ShowWindow(hWndApp, SW_HIDE);
			SetWindowText(GetDlgItem(hWndApp, IDC_EDIT1), _T(""));
			SetWindowText(GetDlgItem(hWndApp, IDC_EDIT2), _T(""));
			ShowWindow((GetDlgItem(hWndApp, IDC_STATIC_NAME)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_STATIC_GENDER)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_EDIT2)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_STATIC4)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_EDIT1)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON1)), SW_HIDE);
			ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON3)), SW_HIDE);
			//SetFocus(hWndApp);
		}
	}

}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>S_OK on success else the failure code</returns>
HRESULT CFaceBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize Kinect and get color, body and face readers
		IColorFrameSource* pColorFrameSource = nullptr;
		IBodyFrameSource* pBodyFrameSource = nullptr;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}
		}

		SafeRelease(pColorFrameSource);
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
/// Main processing function
/// </summary>
void CFaceBasics::Update()
{
	if (GetAsyncKeyState(VK_F1))
	{
		m_Slow = true; m_IsStartRecord = true;
	}



	if (!m_pColorFrameReader || !m_pBodyFrameReader)
	{
		return;
	}

	IColorFrame* pColorFrame = nullptr;
	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = nullptr;
		int nWidth = 0;
		int nHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = nullptr;

		hr = pColorFrame->get_RelativeTime(&nTime);


		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			frame_count++;
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);

			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr) && (frame_count == 30)) //30 slow down recording aryel
		{

			Mat colorFrame(nHeight, nWidth, CV_8UC4, reinterpret_cast<void*>(pBuffer));
			cvtColor(colorFrame, hsvImg, CV_BGR2HSV);
			cvtColor(colorFrame, grayImg, CV_BGR2GRAY);//for face recognition
			frame_count = 0;
			cvtColor(hsvImg,colorFrame, CV_HSV2RGB);

			//Size size1(500, 500);
			//resize(colorFrame, dst, size1);
			

			//Rubha-Facebook
			std::vector<string> result;
			m_client_facebook->getClient()->takePicture(result);

			if (result.at(0) == "YES")
			{
				imwrite("Image.jpg", colorFrame);
				//imshow("RGB", colorFrame);
				//waitKey(0);

				//imshow("RGB", dst);
				//waitKey(0);

				std::streampos size;
				char * memblock;
				std::string *picture, *pic2;

				std::ifstream file("Image.jpg", std::ios::in | std::ios::binary | std::ios::ate);
				if (file.is_open())
				{
					size = file.tellg();
					memblock = new char[size];
					file.seekg(0, std::ios::beg);
					file.read(memblock, size);
					file.close();
					picture = new std::string();// (memblock);
					std::cout << "Size is: " << size << " string size is: " << picture->size() << "\n";

					for (int i = 0; i < size; i++)
						*picture = *picture + memblock[i];
					//aFacebook->getClient()->postMessageAndPicture("test picture", *picture);
					m_client_facebook1->getClient()->postMessageAndPicture("IMI", *picture);
					delete[] memblock;
					delete picture;
				}
				else std::cout << "Unable to open file";
				//string matAsString(colorFrame.begin<unsigned char>(), colorFrame.end<unsigned char>());
				//m_client_facebook1->getClient()->postMessageAndPicture("hi", ColorBuf);
			}
		}

		if (SUCCEEDED(hr))
		{
			DrawStreams(nTime, pBuffer, nWidth, nHeight);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
}

/// <summary>
/// Renders the color and face streams
/// </summary>
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
void CFaceBasics::DrawStreams(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	if (m_hWnd)
	{
		HRESULT hr;
		hr = m_pDrawDataStreams->BeginDrawing();

		if (SUCCEEDED(hr))
		{
			// Make sure we've received valid color data
			if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
			{
				// Draw the data with Direct2D
				hr = m_pDrawDataStreams->DrawBackground(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));
			}
			else
			{
				// Recieved invalid data, stop drawing
				hr = E_INVALIDARG;
			}

			if (SUCCEEDED(hr))
			{
				// begin processing the face frames
				time(&currentTime);
				double seconds = difftime(currentTime, previousTime);
				//if (seconds > 1) //Jianfeng
				if (m_Slow == FALSE)
				{
					ProcessFaces();
					time(&previousTime);
				}
				if (m_Slow == TRUE)
				{
					if (seconds > 1)
					{
						ProcessFaces();
						time(&previousTime);
					}
				}
			}

			m_pDrawDataStreams->EndDrawing();
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
		//StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));
		//StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" X = %0.3f    Y = %0.3f", pJoints[3].x, pJoints[3].y);
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" Wrist_X = %0.3f Wrist_Y = %0.3f Wrist_Z= %0.3f ", w1, w2, w3);
		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}
}

/// <summary>
/// Processes new face frames
/// </summary>
void CFaceBasics::ProcessFaces()
{

	imi::Facefeatures *aFacefeatures[6];

	faceArr.reserve(100);

	HRESULT hr;
	IBody* ppBodies[BODY_COUNT] = { 0 };
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies));

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		aFacefeatures[iFace] = new imi::Facefeatures();
		aSkeleton[iFace] = new imi::Skeleton();
		axySkeleton[iFace] = new imi::xySkeleton();

		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			////Jianfeng
			//// get_ColorFrameReference
			//IColorFrameReference *colorFrameReference;
			//hr = pFaceFrame->get_ColorFrameReference(&colorFrameReference);

			////Gets the actual color frame from the reference
			//IColorFrame *colorFrame;
			//if (SUCCEEDED(hr))
			//{
			//	hr = colorFrameReference->AcquireFrame(&colorFrame);
			//}

			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				RectI faceBox = { 0 };
				PointF facePoints[FacePointType::FacePointType_Count];
				Vector4 faceRotation;
				DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
				D2D1_POINT_2F faceTextLayout;

				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
						//******BODY***********//
						IBody* pBody = ppBodies[iFace];
						if (pBody != nullptr)
						{
							BOOLEAN bTracked = false;
							hr = pBody->get_IsTracked(&bTracked);

							if (SUCCEEDED(hr) && bTracked)
							{
								Joint joints[JointType_Count];
								/*******BODY**********/
								HandState leftHandState = HandState_Unknown;
								HandState rightHandState = HandState_Unknown;

								pBody->get_HandLeftState(&leftHandState);
								pBody->get_HandRightState(&rightHandState);
								/*******BODY**********/

								hr = pBody->GetJoints(_countof(joints), joints);
								if (SUCCEEDED(hr))
								{
									/*******BODY**********/
									aSkeleton[iFace]->spinebase.x = joints[0].Position.X; aSkeleton[iFace]->spinebase.y = joints[0].Position.Y; aSkeleton[iFace]->spinebase.z = joints[0].Position.Z;
									//JointType_SpineMid//
									aSkeleton[iFace]->spinemid.x = joints[1].Position.X; aSkeleton[iFace]->spinemid.y = joints[1].Position.Y; aSkeleton[iFace]->spinemid.z = joints[1].Position.Z;
									//JointType_Neck//
									aSkeleton[iFace]->neck.x = joints[2].Position.X; aSkeleton[iFace]->neck.y = joints[2].Position.Y; aSkeleton[iFace]->neck.z = joints[2].Position.Z;
									//JointType_Head//
									aSkeleton[iFace]->head.x = joints[3].Position.X; aSkeleton[iFace]->head.y = joints[3].Position.Y; aSkeleton[iFace]->head.z = joints[3].Position.Z;
									//JointType_ShoulderLeft//
									aSkeleton[iFace]->shoulderl.x = joints[4].Position.X; aSkeleton[iFace]->shoulderl.y = joints[4].Position.Y; aSkeleton[iFace]->shoulderl.z = joints[4].Position.Z;
									//JointType_ElbowLeft//
									aSkeleton[iFace]->elbowl.x = joints[5].Position.X; aSkeleton[iFace]->elbowl.y = joints[5].Position.Y; aSkeleton[iFace]->elbowl.z = joints[5].Position.Z;
									//JointType_WristLeft
									aSkeleton[iFace]->wristl.x = joints[6].Position.X; aSkeleton[iFace]->wristl.y = joints[6].Position.Y; aSkeleton[iFace]->wristl.z = joints[6].Position.Z;
									//JointType_HandLeft
									aSkeleton[iFace]->handl.x = joints[7].Position.X; aSkeleton[iFace]->handl.y = joints[7].Position.Y; aSkeleton[iFace]->handl.z = joints[7].Position.Z;
									//JointType_ShoulderRight
									aSkeleton[iFace]->shoulderr.x = joints[8].Position.X; aSkeleton[iFace]->shoulderr.y = joints[8].Position.Y; aSkeleton[iFace]->shoulderr.z = joints[8].Position.Z;
									//JointType_ElbowRight
									aSkeleton[iFace]->elbowr.x = joints[9].Position.X; aSkeleton[iFace]->elbowr.y = joints[9].Position.Y; aSkeleton[iFace]->elbowr.z = joints[9].Position.Z;
									//JointType_WristRight
									aSkeleton[iFace]->wristr.x = joints[10].Position.X; aSkeleton[iFace]->wristr.y = joints[10].Position.Y; aSkeleton[iFace]->wristr.z = joints[10].Position.Z;
									//JointType_HandRight
									aSkeleton[iFace]->handr.x = joints[11].Position.X; aSkeleton[iFace]->handr.y = joints[11].Position.Y; aSkeleton[iFace]->handr.z = joints[11].Position.Z;
									//JointType_HipLeft
									aSkeleton[iFace]->hipl.x = joints[12].Position.X;	aSkeleton[iFace]->hipl.y = joints[12].Position.Y; aSkeleton[iFace]->hipl.z = joints[12].Position.Z;
									//JointType_KneeLeft
									aSkeleton[iFace]->kneel.x = joints[13].Position.X; aSkeleton[iFace]->kneel.y = joints[13].Position.Y; aSkeleton[iFace]->kneel.z = joints[13].Position.Z;
									//JointType_AnkleLeft
									aSkeleton[iFace]->anklel.x = joints[14].Position.X; aSkeleton[iFace]->anklel.y = joints[14].Position.Y; aSkeleton[iFace]->anklel.z = joints[14].Position.Z;
									//JointType_FootLeft
									aSkeleton[iFace]->footl.x = joints[15].Position.X; aSkeleton[iFace]->footl.y = joints[15].Position.Y; aSkeleton[iFace]->footl.z = joints[15].Position.Z;
									//JointType_HipRight
									aSkeleton[iFace]->hipr.x = joints[16].Position.X; aSkeleton[iFace]->hipr.y = joints[16].Position.Y; aSkeleton[iFace]->hipr.z = joints[16].Position.Z;
									//JointType_KneeRight	= 17,
									aSkeleton[iFace]->kneer.x = joints[17].Position.X; aSkeleton[iFace]->kneer.y = joints[17].Position.Y; aSkeleton[iFace]->kneer.z = joints[17].Position.Z;
									//JointType_AnkleRight	= 18,
									aSkeleton[iFace]->ankler.x = joints[18].Position.X; aSkeleton[iFace]->ankler.y = joints[18].Position.Y; aSkeleton[iFace]->ankler.z = joints[18].Position.Z;
									//JointType_FootRight	= 19,
									aSkeleton[iFace]->footr.x = joints[19].Position.X; aSkeleton[iFace]->footr.y = joints[19].Position.Y; aSkeleton[iFace]->footr.z = joints[19].Position.Z;
									//JointType_SpineShoulder	= 20,
									aSkeleton[iFace]->spines.x = joints[20].Position.X; aSkeleton[iFace]->spines.y = joints[20].Position.Y; aSkeleton[iFace]->spines.z = joints[20].Position.Z;
									//JointType_HandTipLeft	= 21,
									aSkeleton[iFace]->handtipl.x = joints[21].Position.X; aSkeleton[iFace]->handtipl.y = joints[21].Position.Y; aSkeleton[iFace]->handtipl.z = joints[21].Position.Z;
									//JointType_ThumbLeft	= 22,
									aSkeleton[iFace]->thumbl.x = joints[22].Position.X; aSkeleton[iFace]->thumbl.y = joints[22].Position.Y; aSkeleton[iFace]->thumbl.z = joints[22].Position.Z;
									//JointType_HandTipRight	= 23,
									aSkeleton[iFace]->handtipr.x = joints[23].Position.X; aSkeleton[iFace]->handtipr.y = joints[23].Position.Y; aSkeleton[iFace]->handtipr.z = joints[23].Position.Z;
									//JointType_ThumbRight	= 24,
									aSkeleton[iFace]->thumbr.x = joints[24].Position.X; aSkeleton[iFace]->thumbr.y = joints[24].Position.Y; aSkeleton[iFace]->thumbr.z = joints[24].Position.Z;

									w1 = joints[10].Position.X; w2 = joints[10].Position.Y; w3 = joints[10].Position.Z;

									if (rightHandState == 0)
									{
										aSkeleton[iFace]->rhandstate = imi::HandPossibilities::Unknown;
									}
									else if (rightHandState == 1)
									{
										aSkeleton[iFace]->rhandstate = imi::HandPossibilities::NotTracked;
									}
									else if (rightHandState == 2)
									{
										aSkeleton[iFace]->rhandstate = imi::HandPossibilities::Open;
									}
									else if (rightHandState == 3)
									{
										aSkeleton[iFace]->rhandstate = imi::HandPossibilities::Closed;
									}
									else if (rightHandState == 4)
									{
										aSkeleton[iFace]->rhandstate = imi::HandPossibilities::Lasso;
									}

									//Left
									if (leftHandState == 0)
									{
										aSkeleton[iFace]->lhandstate = imi::HandPossibilities::Unknown;
									}
									else if (leftHandState == 1)
									{
										aSkeleton[iFace]->lhandstate = imi::HandPossibilities::NotTracked;
									}
									else if (leftHandState == 2)
									{
										aSkeleton[iFace]->lhandstate = imi::HandPossibilities::Open;
									}
									else if (leftHandState == 3)
									{
										aSkeleton[iFace]->lhandstate = imi::HandPossibilities::Closed;
									}
									else if (leftHandState == 4)
									{
										aSkeleton[iFace]->lhandstate = imi::HandPossibilities::Lasso;
									}

									//Store in array for Body Basics

								//	if (aSkeleton[iFace]->head.z <= SkeletonDistance)
									//{

									//}
								}
							}
						}

						/*******RRR**********/
						


						for (int i = 0; i <JointType_Count; i++)
						{
							pJoints[i].x = 0;
							pJoints[i].y = 0;
						}
						GetJointsInColorSpace(ppBodies[iFace], pJoints);
						/////////////////////////////
						axySkeleton[iFace]->spinebase.x = pJoints[0].x; axySkeleton[iFace]->spinebase.y = pJoints[0].y;
						//JointType_SpineMid//
						axySkeleton[iFace]->spinemid.x = pJoints[1].x; axySkeleton[iFace]->spinemid.y = pJoints[1].y;
						//JointType_Neck//
						axySkeleton[iFace]->neck.x = pJoints[2].x; axySkeleton[iFace]->neck.y = pJoints[2].y;
						//JointType_Head//
						axySkeleton[iFace]->head.x = pJoints[3].x; axySkeleton[iFace]->head.y = pJoints[3].y;
						//JointType_ShoulderLeft//
						axySkeleton[iFace]->shoulderl.x = pJoints[4].x; axySkeleton[iFace]->shoulderl.y = pJoints[4].y;
						//JointType_ElbowLeft//
						axySkeleton[iFace]->elbowl.x = pJoints[5].x; axySkeleton[iFace]->elbowl.y = pJoints[5].y;
						//JointType_WristLeft
						axySkeleton[iFace]->wristl.x = pJoints[6].x; axySkeleton[iFace]->wristl.y = pJoints[6].y;
						//JointType_HandLeft
						axySkeleton[iFace]->handl.x = pJoints[7].x; axySkeleton[iFace]->handl.y = pJoints[7].y;
						//JointType_ShoulderRight
						axySkeleton[iFace]->shoulderr.x = pJoints[8].x; axySkeleton[iFace]->shoulderr.y = pJoints[8].y;
						//JointType_ElbowRight
						axySkeleton[iFace]->elbowr.x = pJoints[9].x; axySkeleton[iFace]->elbowr.y = pJoints[9].y;
						//JointType_WristRight
						axySkeleton[iFace]->wristr.x = pJoints[10].x; axySkeleton[iFace]->wristr.y = pJoints[10].y;
						//JointType_HandRight
						axySkeleton[iFace]->handr.x = pJoints[11].x; axySkeleton[iFace]->handr.y = pJoints[11].y;
						//JointType_HipLeft
						axySkeleton[iFace]->hipl.x = pJoints[12].x;	axySkeleton[iFace]->hipl.y = pJoints[12].y;
						//JointType_KneeLeft
						axySkeleton[iFace]->kneel.x = pJoints[13].x; axySkeleton[iFace]->kneel.y = pJoints[13].y;
						//JointType_AnkleLeft
						axySkeleton[iFace]->anklel.x = pJoints[14].x; axySkeleton[iFace]->anklel.y = pJoints[14].y;
						//JointType_FootLeft
						axySkeleton[iFace]->footl.x = pJoints[15].x; axySkeleton[iFace]->footl.y = pJoints[15].y;
						//JointType_HipRight
						axySkeleton[iFace]->hipr.x = pJoints[16].x; axySkeleton[iFace]->hipr.y = pJoints[16].y;
						//JointType_KneeRight	= 17,
						axySkeleton[iFace]->kneer.x = pJoints[17].x; axySkeleton[iFace]->kneer.y = pJoints[17].y;
						//JointType_AnkleRight	= 18,
						axySkeleton[iFace]->ankler.x = pJoints[18].x; axySkeleton[iFace]->ankler.y = pJoints[18].y;
						//JointType_FootRight	= 19,
						axySkeleton[iFace]->footr.x = pJoints[19].x; axySkeleton[iFace]->footr.y = pJoints[19].y;
						//JointType_SpineShoulder	= 20,
						axySkeleton[iFace]->spines.x = pJoints[20].x; axySkeleton[iFace]->spines.y = pJoints[20].y;
						//JointType_HandTipLeft	= 21,
						axySkeleton[iFace]->handtipl.x = pJoints[21].x; axySkeleton[iFace]->handtipl.y = pJoints[21].y;
						//JointType_ThumbLeft	= 22,
						axySkeleton[iFace]->thumbl.x = pJoints[22].x; axySkeleton[iFace]->thumbl.y = pJoints[22].y;
						//JointType_HandTipRight	= 23,
						axySkeleton[iFace]->handtipr.x = pJoints[23].x; axySkeleton[iFace]->handtipr.y = pJoints[23].y;
						//JointType_ThumbRight	= 24,
						axySkeleton[iFace]->thumbr.x = pJoints[24].x; axySkeleton[iFace]->thumbr.y = pJoints[24].y;


						//////////////////////////////

						ID2D1SolidColorBrush  *m_brush;
						m_pDrawDataStreams->m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 1.0f), &m_brush);
						for (int i = 0; i<JointType_Count; i++)
						{
							D2D1_RECT_F rect = D2D1::RectF(pJoints[i].x - 30, pJoints[i].y - 30, pJoints[i].x + 30.0, pJoints[i].y + 30.0);
							//m_pDrawDataStreams->m_pRenderTarget->FillRectangle(rect, m_brush);

							D2D1_ELLIPSE ellipse = D2D1::Ellipse(D2D1::Point2F(pJoints[i].x, pJoints[i].y), 1.0f, 1.0f);
							m_pDrawDataStreams->m_pRenderTarget->DrawEllipse(ellipse, m_brush, 20.0f);

						}

						/*******RRR**********/
						//******BODY***********//

					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);

						//eNGAGED//
						int prperty = faceProperties[1];
						if (prperty == 0)
						{
							aFacefeatures[iFace]->engaged = imi::FacePossibilities::UNKNOWN;
						}
						else if (prperty == 1){
							aFacefeatures[iFace]->engaged = imi::FacePossibilities::NO;
						}
						else if (prperty == 2){
							aFacefeatures[iFace]->engaged = imi::FacePossibilities::MAYBE;
						}
						else if (prperty == 3){
							aFacefeatures[iFace]->engaged = imi::FacePossibilities::YES;
						}
						//happy//
						prperty = faceProperties[0];
						if (prperty == 0)
						{
							aFacefeatures[iFace]->happy = imi::FacePossibilities::UNKNOWN;
						}
						else if (prperty == 1){
							aFacefeatures[iFace]->happy = imi::FacePossibilities::NO;
						}
						else if (prperty == 2){
							aFacefeatures[iFace]->happy = imi::FacePossibilities::MAYBE;
						}
						else if (prperty == 3){
							aFacefeatures[iFace]->happy = imi::FacePossibilities::YES;
						}


						// cloth color spine joints//
						CameraSpacePoint head;
						head = GetFaceTextPositionInColorSpace_1(ppBodies[iFace], &faceTextLayout);

						/*
						D2D1_POINT_2F pJoints[JointType_Count];
						for (int i = 0; i <JointType_Count; i++)
						{
							pJoints[i].x = 0;
							pJoints[i].y = 0;
						}
						GetJointsInColorSpace(ppBodies[iFace], pJoints);
						*/
						int clothColor = getClothColor(hsvImg, pJoints);
						aFacefeatures[iFace]->color = imi::ColorPossibilities::BLACK;
						//cloth color//
						if (clothColor == 0)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::RED;
						}
						else if (clothColor == 1)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::ORANGE;
						}
						else if (clothColor == 2)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::YELLOW;
						}
						else if (clothColor == 3)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::GREEN;
						}
						else if (clothColor == 4)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::CYAN;
						}
						else if (clothColor == 5)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::BLUE;
						}
						else if (clothColor == 6)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::VIOLET;
						}
						else if (clothColor == 7)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::MAGENTA;
						}
						else if (clothColor == 8)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::WHITE;
						}
						else if (clothColor == 9)
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::BLACK;
						}
						else
						{
							aFacefeatures[iFace]->color = imi::ColorPossibilities::UNKNOWN;
						}


						// face recognition
						//time(&currentTime);
						//double seconds = difftime(currentTime,previousTime);
						//if (seconds > 1)

						//Jianfeng
						int pitch, yaw, roll;
						ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
						if ((pitch >= -10) && (pitch <= 10) && (yaw >= -10) && (yaw <= 10) && (roll >= -10) && (roll <= 10)) //aryel
						{
							//aFacefeatures[iFace]->Face_rec_ID = FaceVerification(facePoints[0], facePoints[1]);
							int face_id = FaceVerification(facePoints[0], facePoints[1]); // core function, inputs: eye cords // aryel
							//Read file

 							std::ifstream file2("face_recognition_names.txt");
							

							for (int i = 0; i < NUM_OF_REGISTER_USER + 1; ++i)
							{
								std::getline(file2, name);
								name_arr[i] = name;
							}
	
							aFacefeatures[iFace]->Face_rec_ID = name_arr[face_id];

							//time(&previousTime);
						}

						//aFacefeatures[i]->user_name = string(m_WatchList[i].user_name);

						//Push into faceArray - Distance//
						if (head.Z <= SkeletonDistance && pJoints[3].x != 0.0)
						{
							
							gr = pJoints[3].y / pJoints[3].x;
							//if (gr > -angle && gr < angle)
							{
								faceArr.push_back(*aFacefeatures[iFace]);
								SkelArr.push_back(*aSkeleton[iFace]);
								//xySkelArr.push_back(*axySkeleton[iFace]);
							}
						}
					}

					if (SUCCEEDED(hr))
					{
						hr = GetFaceTextPositionInColorSpace(ppBodies[iFace], &faceTextLayout);

					}

					if (SUCCEEDED(hr))
					{
						// draw face frame results
						m_pDrawDataStreams->DrawFaceFrameResults(iFace, &faceBox, facePoints, &faceRotation, faceProperties, &faceTextLayout);

					}
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);

	}

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);

		}
	}
	//Draw id:
	//std::string result;
	//result = m_client_facebook()->takePicture();
	//i2p
	//Release//


	for (int rr = 0; rr < 6; rr++)
	{
		delete aFacefeatures[rr];
		delete aSkeleton[rr];
		delete axySkeleton[rr];

	}
	//End of i2p//

	try
	{
		if (flag_skel == true)
		{
			//m_client->getClient()->MultiUserSkeletonChanged("KinectV2", imi::getTimeStamp(), SkelArr, xySkelArr);
			m_client->getClient()->MultiUserFaceFeatures("KinectV2", imi::getTimeStamp(), faceArr);
		}
		//if (flag_ges == true)
		{
			//if (pJoints[3].x != 0.0)
			{

				WaveHandGesture();
				L_WaveHandGesture();
				HandShakingGesture();
			}
			m_client->getClient()->gestureStart("KinectV2", imi::getTimeStamp(), GesArr);
			m_client->getClient()->gestureHandsShakingStart("KinectV2", imi::getTimeStamp(), gesShakeArr);
			//m_client->getClient()->gestureStart("KinectV2", imi::getTimeStamp(), GesArr);
		}
		GesArr.clear();
		SkelArr.clear();
		gesShakeArr.clear();
		faceArr.clear();
		//xySkelArr.clear();
	}
	catch (apache::thrift::TException &tx)
	{
		m_client->ensureConnection();
		ExitProcess(0);

	}

}
void CFaceBasics::HandShakingGesture()
{

	for (int k = 0; k < SkelArr.size(); k++)
	{

		aGestureHandShake[k] = new imi::GestureHandShake();

		while (hand_shake_users.size()<SkelArr.size())
		{
			hand_shake_users.push_back(new Hand_Shake());
		}

		while (hand_shake_users.size()>SkelArr.size())
		{
			Hand_Shake *temp = hand_shake_users.back();
			hand_shake_users.pop_back();
			delete temp;
		}
		//Height = -50(-0.5) && (SkelArr.at(k).wristr.y <= -0.5) && (SkelArr.at(k).wristr.y <= -0.5) // (SkelArr.at(k).wristr.x >= -0.2 && SkelArr.at(k).wristr.x <= 0.2)
		//Distance near robot and distance spine-wrist
		//if ((SkelArr.at(k).head.z <= 1.6) && (SkelArr.at(k).spinemid.z - SkelArr.at(k).wristr.z) > 0.3 && (SkelArr.at(k).head.z != 0.0) && (SkelArr.at(k).elbowr.y > SkelArr.at(k).wristr.y))
		if ((SkelArr.at(k).head.z <= 1.6) && (SkelArr.at(k).head.z != 0.0))
		{
			if ((SkelArr.at(k).wristr.x <= x_Max && SkelArr.at(k).wristr.x >= x_Min))
			{
				if ((SkelArr.at(k).wristr.y <= y_Max && SkelArr.at(k).wristr.y >= y_Min))
				{
					if (SkelArr.at(k).wristr.z <= z_Max && SkelArr.at(k).wristr.z >= z_Min)
					{
						hand_shake_users.at(k)->shake_temp = hand_shake_users.at(k)->shake_temp + 1;

						if ((hand_shake_users.at(k)->shake_temp >= 5))
						{
							hand_shake_users.at(k)->shake_temp = 0;
							aGestureHandShake[k]->Right_HandShake = imi::GesPossibilities::YES;
						}
					}

				}
			}
			/*
			hand_shake_users.at(k)->shake_temp = hand_shake_users.at(k)->shake_temp + 1;
			if ((hand_shake_users.at(k)->shake_temp >= 5))
			{
			hand_shake_users.at(k)->shake_temp = 0;
			aGestureHandShake[k]->Right_HandShake = imi::GesPossibilities::YES;
			}*/
			//
		}// - how to find the points are inside the rectangle - 
		else
		{
			hand_shake_users.at(k)->shake_temp = 0;
			aGestureHandShake[k]->Right_HandShake = imi::GesPossibilities::NO;

		}
		gesShakeArr.push_back(*aGestureHandShake[k]);
	}
}

void CFaceBasics::WaveHandGesture()
{

	for (int k = 0; k < SkelArr.size(); k++)
	{
		aGestureStatus[k] = new imi::GestureStatus();

		while (wave_users.size()<SkelArr.size())
		{
			wave_users.push_back(new Wave());
		}

		while (wave_users.size()>SkelArr.size())
		{
			Wave *temp = wave_users.back();
			wave_users.pop_back();
			delete temp;
		}

		wave_users.at(k)->res[0] = WaveRightSegment1(SkelArr.at(k), wave_users.at(k)->position, k);
		wave_users.at(k)->res[1] = WaveRightSegment2(SkelArr.at(k), wave_users.at(k)->position, k);
		wave_users.at(k)->res[2] = WaveRightSegment1(SkelArr.at(k), wave_users.at(k)->position, k);
		wave_users.at(k)->res[3] = WaveRightSegment2(SkelArr.at(k), wave_users.at(k)->position, k);


		wave_users.at(k)->result = wave_users.at(k)->res[wave_users.at(k)->currentSegment];

		if (wave_users.at(k)->result == 1)
		{
			if (wave_users.at(k)->currentSegment + 1 < 3)
			{
				wave_users.at(k)->currentSegment++;
				wave_users.at(k)->frameCount = 0;
				aGestureStatus[k]->Right_wavehand = imi::GesPossibilities::NO;
			}
			else


			{
				aGestureStatus[k]->Right_wavehand = imi::GesPossibilities::YES;
				wave_users.at(k)->frameCount = 0; wave_users.at(k)->currentSegment = 0;
			}
		}
		else if (wave_users.at(k)->result == 0 || wave_users.at(k)->frameCount == 50)
		{
			wave_users.at(k)->frameCount = 0; wave_users.at(k)->currentSegment = 0;
			aGestureStatus[k]->Right_wavehand = imi::GesPossibilities::NO;
		}
		else
		{
			wave_users.at(k)->frameCount++;
			aGestureStatus[k]->Right_wavehand = imi::GesPossibilities::NO;
		}
		//GesArr.push_back(*aGestureStatus[k]);
		wave_users.at(k)->position = SkelArr.at(k).handr.x;

		//To Check//
		//swprintf(szBuf, L"User ID: %d \n", k);
		//OutputDebugString(szBuf);
		//swprintf(szBuf, L"Result: %d \n", wave_users.at(k)->result);
		//OutputDebugString(szBuf);
		//swprintf(szBuf, L"currentSegment: %d \n", wave_users.at(k)->currentSegment);
		//OutputDebugString(szBuf);
		//swprintf(szBuf, L"Status WaveHand: %d \n", aGestureStatus[k]->wavehand);
		//OutputDebugString(szBuf);
		//swprintf(szBuf, L"******\n");
		//OutputDebugString(szBuf);
	}

}

void CFaceBasics::L_WaveHandGesture()
{
	//imi::GestureStatus *aGestureStatus[6];

	for (int L_k = 0; L_k < SkelArr.size(); L_k++)
	{
		//aGestureStatus[L_k] = new imi::GestureStatus();

		while (L_wave_users.size()<SkelArr.size())
		{
			L_wave_users.push_back(new L_Wave());
		}

		while (L_wave_users.size()>SkelArr.size())
		{
			L_Wave *L_temp = L_wave_users.back();
			L_wave_users.pop_back();
			delete L_temp;
		}

		L_wave_users.at(L_k)->L_res[0] = L_WaveLeftSegment1(SkelArr.at(L_k), L_wave_users.at(L_k)->L_position, L_k);
		L_wave_users.at(L_k)->L_res[1] = L_WaveLeftSegment2(SkelArr.at(L_k), L_wave_users.at(L_k)->L_position, L_k);
		L_wave_users.at(L_k)->L_res[2] = L_WaveLeftSegment1(SkelArr.at(L_k), L_wave_users.at(L_k)->L_position, L_k);
		L_wave_users.at(L_k)->L_res[3] = L_WaveLeftSegment2(SkelArr.at(L_k), L_wave_users.at(L_k)->L_position, L_k);


		L_wave_users.at(L_k)->L_result = L_wave_users.at(L_k)->L_res[L_wave_users.at(L_k)->L_currentSegment];

		if (L_wave_users.at(L_k)->L_result == 1)
		{
			if (L_wave_users.at(L_k)->L_currentSegment + 1 < 3)
			{
				L_wave_users.at(L_k)->L_currentSegment++;
				L_wave_users.at(L_k)->L_frameCount = 0;
				aGestureStatus[L_k]->Left_wavehand = imi::GesPossibilities::NO;
			}
			else
			{
				aGestureStatus[L_k]->Left_wavehand = imi::GesPossibilities::YES;
				L_wave_users.at(L_k)->L_frameCount = 0; L_wave_users.at(L_k)->L_currentSegment = 0;
			}
		}
		else if (L_wave_users.at(L_k)->L_result == 0 || L_wave_users.at(L_k)->L_frameCount == 50)
		{
			L_wave_users.at(L_k)->L_frameCount = 0; L_wave_users.at(L_k)->L_currentSegment = 0;
			aGestureStatus[L_k]->Left_wavehand = imi::GesPossibilities::NO;
		}
		else
		{
			L_wave_users.at(L_k)->L_frameCount++;
			aGestureStatus[L_k]->Left_wavehand = imi::GesPossibilities::NO;
		}
		GesArr.push_back(*aGestureStatus[L_k]);
		L_wave_users.at(L_k)->L_position = SkelArr.at(L_k).handl.x;

	}

}


int CFaceBasics::WaveRightSegment1(imi::Skeleton ges_skel1, float ges_skel2, int k)
{
	//if ((ges_skel1.handr.y > ges_skel1.elbowr.y) && (ges_skel1.handr.y > ges_skel1.spinebase.y) && (ges_skel1.rhandstate == 2))
	if ((ges_skel1.handr.y > ges_skel1.elbowr.y) && (ges_skel1.handr.y > ges_skel1.spinebase.y) && (ges_skel1.rhandstate == 2))
	{
		wave_users.at(k)->temp1 = wave_users.at(k)->temp1 + abs(ges_skel2 - ges_skel1.handr.x);

		if ((ges_skel1.handr.x > ges_skel2) && (wave_users.at(k)->temp1 >= 0.03))
		{
			wave_users.at(k)->temp1 = 0;
			return ges_success_id = 1;
		}
		return ges_success_id = 2;
	}
	wave_users.at(k)->temp1 = 0;
	return ges_success_id = 0;

}

int CFaceBasics::WaveRightSegment2(imi::Skeleton ges_skel1, float ges_skel2, int k)
{


	if ((ges_skel1.handr.y > ges_skel1.elbowr.y) && (ges_skel1.handr.y > ges_skel1.spinebase.y) && (ges_skel1.rhandstate == 2))
	{
		wave_users.at(k)->temp = wave_users.at(k)->temp + abs(ges_skel2 - ges_skel1.handr.x);

		if ((ges_skel1.handr.x < ges_skel2) && (wave_users.at(k)->temp >= 0.03))
		{
			wave_users.at(k)->temp = 0;
			return ges_success_id = 1;
		}
		return ges_success_id = 2;
	}
	wave_users.at(k)->temp = 0;
	return ges_success_id = 0;

}
int CFaceBasics::L_WaveLeftSegment1(imi::Skeleton ges_skel1, float ges_skel2, int k)
{
	if ((ges_skel1.handl.y > ges_skel1.elbowl.y) && (ges_skel1.handl.y > ges_skel1.spinebase.y) && (ges_skel1.lhandstate == 2))
	{
		L_wave_users.at(k)->L_temp1 = L_wave_users.at(k)->L_temp1 + abs(ges_skel2 - ges_skel1.handl.x);

		if ((ges_skel1.handl.x > ges_skel2) && (L_wave_users.at(k)->L_temp1 >= 0.03))
		{
			L_wave_users.at(k)->L_temp1 = 0;
			return L_ges_success_id = 1;
		}
		return L_ges_success_id = 2;
	}
	L_wave_users.at(k)->L_temp1 = 0;
	return L_ges_success_id = 0;

}

int CFaceBasics::L_WaveLeftSegment2(imi::Skeleton ges_skel1, float ges_skel2, int k)
{


	if ((ges_skel1.handl.y > ges_skel1.elbowl.y) && (ges_skel1.handl.y > ges_skel1.spinebase.y) && (ges_skel1.lhandstate == 2))
	{
		L_wave_users.at(k)->L_temp = L_wave_users.at(k)->L_temp + abs(ges_skel2 - ges_skel1.handl.x);

		if ((ges_skel1.handl.x < ges_skel2) && (L_wave_users.at(k)->L_temp >= 0.03))
		{
			L_wave_users.at(k)->L_temp = 0;
			return L_ges_success_id = 1;
		}
		return L_ges_success_id = 2;
	}
	L_wave_users.at(k)->L_temp = 0;
	return L_ges_success_id = 0;

}

/// <summary>
/// Computes the face result text position by adding an offset to the corresponding 
/// body's head joint in camera space and then by projecting it to screen space
/// </summary>
/// <param name="pBody">pointer to the body data</param>
/// <param name="pFaceTextLayout">pointer to the text layout position in screen space</param>
/// <returns>indicates success or failure</returns>
HRESULT CFaceBasics::GetFaceTextPositionInColorSpace(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout)
{
	HRESULT hr = E_FAIL;

	if (pBody != nullptr)
	{
		BOOLEAN bTracked = false;
		hr = pBody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			hr = pBody->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr))
			{
				CameraSpacePoint headJoint = joints[JointType_Head].Position;
				CameraSpacePoint textPoint =
				{
					headJoint.X + c_FaceTextLayoutOffsetX,
					headJoint.Y + c_FaceTextLayoutOffsetY,
					headJoint.Z
				};

				ColorSpacePoint colorPoint = { 0 };
				hr = m_pCoordinateMapper->MapCameraPointToColorSpace(textPoint, &colorPoint);

				if (SUCCEEDED(hr))
				{
					pFaceTextLayout->x = colorPoint.X;
					pFaceTextLayout->y = colorPoint.Y;
				}
			}
		}
	}

	return hr;
}

HRESULT CFaceBasics::GetJointsInColorSpace(IBody* pBody, D2D1_POINT_2F* pJoint)
{
	HRESULT hr = E_FAIL;

	if (pBody != nullptr)
	{
		BOOLEAN bTracked = false;
		hr = pBody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			hr = pBody->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr))
			{
				for (int i = 0; i<JointType_Count; i++)
				{
					CameraSpacePoint headJoint = joints[i].Position;

					ColorSpacePoint colorPoint = { 0 };
					hr = m_pCoordinateMapper->MapCameraPointToColorSpace(headJoint, &colorPoint);

					if (SUCCEEDED(hr))
					{
						pJoint[i].x = colorPoint.X;
						pJoint[i].y = colorPoint.Y;
					}
				}
			}
		}
	}

	return hr;
}

CameraSpacePoint CFaceBasics::GetFaceTextPositionInColorSpace_1(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout)
{
	CameraSpacePoint headJoint;
	HRESULT hr = E_FAIL;

	if (pBody != nullptr)
	{
		BOOLEAN bTracked = false;
		hr = pBody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			/*******BODY**********/
			HandState leftHandState = HandState_Unknown;
			HandState rightHandState = HandState_Unknown;

			pBody->get_HandLeftState(&leftHandState);
			pBody->get_HandRightState(&rightHandState);
			/*******BODY**********/

			hr = pBody->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr))
			{
			
				headJoint = joints[JointType_Head].Position;
				CameraSpacePoint textPoint =
				{
					headJoint.X + c_FaceTextLayoutOffsetX,
					headJoint.Y + c_FaceTextLayoutOffsetY,
					headJoint.Z
				};


				ColorSpacePoint colorPoint = { 0 };
				hr = m_pCoordinateMapper->MapCameraPointToColorSpace(textPoint, &colorPoint);

				if (SUCCEEDED(hr))
				{
					pFaceTextLayout->x = colorPoint.X;
					pFaceTextLayout->y = colorPoint.Y;
				}
			}
		}
	}
	return headJoint;
}

/// <summary>
/// Updates body data
/// </summary>
/// <param name="ppBodies">pointer to the body data storage</param>
/// <returns>indicates success or failure</returns>
HRESULT CFaceBasics::UpdateBodyData(IBody** ppBodies)
{
	HRESULT hr = E_FAIL;

	if (m_pBodyFrameReader != nullptr)
	{
		IBodyFrame* pBodyFrame = nullptr;
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		}
		SafeRelease(pBodyFrame);
	}

	return hr;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
/// <returns>success or failure</returns>
bool CFaceBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, ULONGLONG nShowTimeMsec, bool bForce)
{
	ULONGLONG now = GetTickCount64();

	if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
	{
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}

int CFaceBasics::getClothColor(Mat hsvImg, D2D1_POINT_2F* pJoints)
{
	// image patch of the same size 15 x 15
	int halfw = ceil((pJoints[JointType_ShoulderRight].x - pJoints[JointType_ShoulderLeft].x) / 2);
	int halfh = ceil(pJoints[JointType_SpineMid].y - pJoints[JointType_SpineShoulder].y);
	int roiWidth = halfw * 2 + 1;
	int roiHeight = halfh * 2 + 1;

	int CENTRERED = 1;

	// 6 bins
	double* hist = new double[10];
	//int hist[256*20*2];
	for (int i = 0; i<10; i++)
		hist[i] = 0;

	// 4 extract color histogram
	// 3.1 get image patch
	int roiX = pJoints[JointType_SpineMid].x - halfw;
	if (roiX < 0)
	{
		roiX = 0;
		CENTRERED = 0;
	}

	if (pJoints[JointType_SpineMid].x + halfw > hsvImg.cols)
	{
		roiX = hsvImg.cols - roiWidth;
		CENTRERED = 0;
	}
	int roiY = pJoints[JointType_SpineMid].y - halfh;
	if (roiY < 0)
	{
		roiY = 0;
		CENTRERED = 0;
	}
	if (pJoints[JointType_SpineMid].y + halfh > hsvImg.rows)
	{
		roiY = hsvImg.rows - roiHeight;
		CENTRERED = 0;
	}

	int max_bin = -1;
	int bin_max = 0;
	double max_bin_ratio;
	int j = 0;
	int k = 0;
	double meanV = 0;
	try
	{
		//crop image patch
		Mat cropedImageHSV = hsvImg(Rect(roiX, roiY, roiWidth, roiHeight));

		// show image patch;
		//Mat cropedImage = hsvImg(Rect(roiX,roiY,roiWidth,roiHeight));
		//imshow("crop image",cropedImage);
		//waitKey(0);

		// one 256 histogram for each channel
		// get H;
		//int* H = new int[roiWidth*roiHeight];
		//int* S = new int[roiWidth*roiHeight];
		for (j = 0; j<cropedImageHSV.rows; j++)
		{
			for (k = 0; k<cropedImageHSV.cols; k++)
			{
				int H = 2 * cropedImageHSV.at<Vec3b>(j, k)[0];
				int V = cropedImageHSV.at<Vec3b>(j, k)[2];
				meanV += V;

				//color histogram accumulation
				double offsetL = 5; // remove intersection region, aryel 
				double offsetU = 5;
				//first is H
				if ((H >= 0) && (H < 22.5 - offsetU)) //red 0 
					hist[0]++;
				else if ((H>22.5 + offsetL) && (H < 37.5 - offsetU)) //orange 1 
					hist[1]++;
				else if ((H>37.5 + offsetL) && (H < 82.5 - offsetU)) //yellow 2 
					hist[2]++;
				else if ((H>82.5 + offsetL) && (H < 142.5 - offsetU)) //green 3 
					hist[3]++;
				else if ((H> 142.5 + offsetL) && (H < 202.5 - offsetU)) //cyan 4
					hist[4]++;
				else if ((H>202.5 + 5 + offsetL) && (H < 262.5 - 5 - offsetU)) //blue 5
					hist[5]++;
				else if ((H>262.5 + offsetL) && (H < 277.5 - offsetU)) //violet 6
					hist[6]++;
				else if ((H>277.5 + offsetL) && (H < 337.5 - offsetU)) //magenta 7
					hist[7]++;
				else if ((H>337.5 + offsetL) && (H < 360)) //red 0
					hist[0]++;
			}
		}

		// check max bin
		for (int i = 0; i<8; i++)
		{
			if (hist[i]>bin_max)
			{
				max_bin = i;
				bin_max = hist[i];
			}
		}

		// if less than 50%, then it is mixed color
		max_bin_ratio = double(bin_max) / double(cropedImageHSV.cols) / double(cropedImageHSV.rows);
		meanV = meanV / double(cropedImageHSV.cols) / double(cropedImageHSV.rows);
		if (max_bin_ratio<0.5)//50% of pixels need to be same color ARYEL 
			max_bin = -1;
		else if (meanV >230) // aryel
			max_bin = 8; //white
		else if (meanV < 20)
			max_bin = 9; //black
	}
	catch (exception ex)
	{
	}
	return max_bin;
}

// face recognition
int CFaceBasics::FaceVerification(PointF EyeLeft, PointF EyeRight)
{
	double minDist = 20; // aryel threshold for face verification, minimin score.
	//double minDist = 10000;//Changed by Aryel
	double minDistArray[currentTrainedUser];

	BYTE* CorpFaceImg = new BYTE[FACE_VERIFY_HEIGHT * FACE_VERIFY_WIDTH];
	if (grayImg.data == NULL)
		return 0;
	////Image Rotation, corp, and scaling
	if (!GeoNormal(grayImg.data, EyeLeft, EyeRight, CorpFaceImg)) // upright position
	{
		delete[] CorpFaceImg;
		//delete [] Image;
		return false;
	}

	//cv::Mat tempMat1 = cv::Mat(FACE_VERIFY_HEIGHT, FACE_VERIFY_WIDTH, CV_8UC1, (void*)CorpFaceImg);
	//imshow("fd", tempMat1);
	//waitKey(0);

	//int t0 = clock();
	BYTE *LBPMap = new BYTE[(FACE_VERIFY_HEIGHT - LBP_R * 2)*(FACE_VERIFY_WIDTH - LBP_R * 2)];
	calLBPImg(CorpFaceImg, LBPMap); // extract LBP features

	//cv::Mat tempMat2= cv::Mat(FACE_VERIFY_HEIGHT-LBP_R*2,FACE_VERIFY_WIDTH-LBP_R*2,CV_8UC1,(void*)LBPMap);
	//imshow("fd", tempMat2);
	//waitKey(0);


	//int t1 = clock();
	//		
	double* hist = new double[256 * NUM_OF_PATCH*NUM_OF_PATCH];
	calHistForWholeImage(LBPMap, hist); // calculate the LBP histogram, final feature vector
	delete[] LBPMap;


	int ppl_detect = 0;
	histSize = 256 * NUM_OF_PATCH*NUM_OF_PATCH;
	double weight[49] = { 2, 1, 1, 1, 1, 1, 2,
		2, 4, 4, 1, 4, 4, 2,
		1, 1, 1, 0, 1, 1, 1,
		0, 1, 1, 0, 1, 1, 0,
		0, 1, 1, 1, 1, 1, 0,
		0, 1, 1, 2, 1, 1, 0,
		0, 1, 1, 1, 1, 1, 0 };


	//Record the templates
	// need to find a way to get keyboard input
	//VK_F1 VK_CONTROL
	//m_IsStartRecord = false;

	if (m_IsStartRecord == true)
	{
		m_IsStartRecord = true;
		if (m_IsStartRecord)
		{
			//imshow("RGB", dst);
			//waitKey(0);

			//FaceDlg..//	record face
			SetTimer(hWndApp, 1, 500, NULL);
			ShowWindow((GetDlgItem(hWndApp, IDC_STATIC4)), SW_SHOW);
			//ShowWindow((GetDlgItem(hWndApp, IDC_STATIC4)), SW_HIDE);
			//ShowWindow((GetDlgItem(hWndApp, IDC_EDIT1)), SW_HIDE);
			//ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON1)), SW_HIDE);
			//ShowWindow((GetDlgItem(hWndApp, IDC_STATIC3)), SW_SHOW);
			//ShowWindow(FacehWndApp, SW_SHOWDEFAULT);

			histSize = 256 * NUM_OF_PATCH*NUM_OF_PATCH;
			for (int i = 0; i < histSize; i++)
				newUserTemplate[histSize*newUserTemplateNum + i] = hist[i];


			//int t3 = clock();
			newUserTemplateNum++;
			if (newUserTemplateNum == NUM_OF_USER_TEMPLATE) // more template, better performance, aryel
			{
				newUserTemplateNum = 0;
				m_IsStartRecord = false;
				m_Slow = false;
				//Convert string to wstring
				std::wstringstream ws;
				ws << name_arr[NUM_OF_REGISTER_USER].c_str();
				file_no = ws.str();

				wstring filename = L"\\face_recognition_data\\" + file_no;

				DWORD cwdsz = GetCurrentDirectory(0, 0); // determine size needed
				wchar_t *cwd = new wchar_t[300];
				GetCurrentDirectory(cwdsz, cwd);
				wstring filenamefull = wstring(cwd);
				filenamefull.append(filename);

				FILE *fid;
				_wfopen_s(&fid, filenamefull.c_str(), L"wb");
				fwrite((double *)newUserTemplate, sizeof(double), NUM_OF_USER_TEMPLATE*histSize, fid);
				fclose(fid);


				//ShowWindow((GetDlgItem(hWndApp, IDC_STATIC1)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_STATIC4)), SW_HIDE);
				ShowWindow((GetDlgItem(hWndApp, IDC_EDIT1)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON1)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_BUTTON3)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_EDIT2)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_STATIC_NAME)), SW_SHOW);
				ShowWindow((GetDlgItem(hWndApp, IDC_STATIC_GENDER)), SW_SHOW);
				KillTimer(hWndApp, 1);

				InitializeFaceRecognitionModule();

				//newUserTemplateNum = 0;
				//m_IsStartRecord = false;
				//exit(0);

			}
		}

	}
	else // recognition
	{
		ppl_detect = 0;

		for (int ppl = 0; ppl < currentTrainedUser; ppl++)
		{
			minDistArray[ppl] = 30;
			//matrix multiplication
			for (int i = 0; i < NUM_OF_USER_TEMPLATE; i++)
			{
				//WatchList[ppl].verify_face[histSize*i]

				double chi2dist = 0;
				for (int j = 0; j < NUM_OF_PATCH*NUM_OF_PATCH; j++)
				{
					double chi2dist_P = 0;
					//double hh[256];
					for (int k = 0; k < 256; k++)
					{
						chi2dist_P += (hist[j * 256 + k] - m_WatchList[ppl].verify_face[i*histSize + j * 256 + k])*(hist[j * 256 + k] - m_WatchList[ppl].verify_face[i*histSize + j * 256 + k]) / (hist[j * 256 + k] + m_WatchList[ppl].verify_face[i*histSize + j * 256 + k] + 1e-100);
					}
					chi2dist += chi2dist_P*weight[j];
				}

				if (chi2dist < minDistArray[ppl])
				{
					minDistArray[ppl] = chi2dist;
				}

				if (chi2dist < minDist)
				{
					minDist = chi2dist;
					cout << minDist << "\n";
					ppl_detect = ppl;
				}
			}
		}
		delete[] hist;

		if (minDist < 0) // chekc wehter the template is properly recorded. aryel
			return 0;

		return ppl_detect;
	}


	return 0;

}

BOOL CFaceBasics::GeoNormal(BYTE* Image, PointF EyeLeft, PointF EyeRight, BYTE* CorpFaceImg)
{
	double lx = EyeLeft.X;
	double ly = EyeLeft.Y;
	double rx = EyeRight.X;
	double ry = EyeRight.Y;

	double EyeDistance = sqrt((lx - rx)*(lx - rx) + (ly - ry)*(ly - ry));
	if (EyeDistance < 0.0000001)
		return false;

	double sinRatio = (ry - ly) / EyeDistance;
	double cosRatio = (rx - lx) / EyeDistance;

	//below is calculate the starting and ending coordinates for corping and rotation
	double Win_L_Edge_x = lx - (rx - lx) * CORP_EYE_L_EDGE / CORP_EYE_DISTANCE;
	double Win_L_Edge_y = ly - (ry - ly) * CORP_EYE_L_EDGE / CORP_EYE_DISTANCE;

	double Win_Start_x = Win_L_Edge_x + CORP_EYE_T_EDGE * EyeDistance * sinRatio / CORP_EYE_DISTANCE;
	double Win_Start_y = Win_L_Edge_y - CORP_EYE_T_EDGE * EyeDistance * cosRatio / CORP_EYE_DISTANCE;

	if (Win_Start_y < 0) Win_Start_y = 0;

	//below is for scaling
	double ScaleFactor = CORP_EYE_DISTANCE / EyeDistance;

	for (int i = 0; i < FACE_VERIFY_WIDTH; i++)
	{
		for (int j = 0; j < FACE_VERIFY_HEIGHT; j++)
		{
			double tempPos_y = Win_Start_y + ((double)i * sinRatio + (double)j * cosRatio) / ScaleFactor;
			double tempPos_x = Win_Start_x + ((double)i * cosRatio - (double)j * sinRatio) / ScaleFactor;

			if ((tempPos_x<0) || (tempPos_x>cColorWidth) || (tempPos_y<0) || (tempPos_y>cColorHeight))
				return false;

			CorpFaceImg[j*FACE_VERIFY_WIDTH + i] = Image[round(tempPos_y)*cColorWidth + round(tempPos_x)];
		}
	}

	return true;

}

int CFaceBasics::power(int a, int b)
{
	int R = 1;
	for (int i = 0; i<b; i++)
		R *= a;

	return R;
}

int CFaceBasics::round(double floatNumber)
{
	int INTNumber = int(floatNumber);
	if (floatNumber < INTNumber + 0.5)
		return INTNumber;
	else return (INTNumber + 1);
}

BOOL CFaceBasics::calHistForWholeImage(BYTE* LBP_image, double* hist)
{
	for (int i = 0; i<256 * NUM_OF_PATCH*NUM_OF_PATCH; i++)
		hist[i] = 0;

	double ix = FACE_VERIFY_WIDTH - LBP_R * 2;
	double iy = FACE_VERIFY_HEIGHT - LBP_R * 2;
	int sx = round(ix / double(NUM_OF_PATCH));
	int sy = round(iy / double(NUM_OF_PATCH));
	int cc = 0;

	for (int i = 0; i<NUM_OF_PATCH; i++)
	{
		int y_start = round(iy / double(NUM_OF_PATCH)*double(i));
		if (y_start + sy > iy)
			y_start = int(iy - sy + 1);
		for (int j = 0; j<NUM_OF_PATCH; j++)
		{
			int x_start = round(ix / double(NUM_OF_PATCH)*double(j));
			if (x_start + sx >  ix)
				x_start = int(ix - sx + 1);

			for (int ii = 0; ii<sx; ii++)
			{
				for (int jj = 0; jj<sy; jj++)
				{
					int x_pos = x_start + ii;
					int y_pos = y_start + jj;
					hist[cc * 256 + LBP_image[x_pos*int(iy) + y_pos]]++;
				}
			}

			for (int ii = 0; ii<256; ii++)
				hist[cc * 256 + ii] /= sx*sy;
			cc++;
		}
	}
	return true;
}

//**********************//
// calculate LBP image
//**********************//
BOOL CFaceBasics::calLBPImg(BYTE* Image, BYTE* LBPMap)
{
	double offset_x[8] = { 2, sqrt(2.0f), 0, -sqrt(2.0f), -2, -sqrt(2.0f), 0, sqrt(2.0f) };
	double offset_y[8] = { 0, -sqrt(2.0f), -2, -sqrt(2.0f), 0, sqrt(2.0f), 2, sqrt(2.0f) };
	int hb = LBP_R;
	int dy = FACE_VERIFY_HEIGHT - LBP_R * 2;
	int dx = FACE_VERIFY_WIDTH - LBP_R * 2;

	for (int i = 0; i<dx*dy; i++)
		LBPMap[i] = 0;

	int LBP_bit;
	for (int i = 0; i<dy; i++)
	{
		for (int j = 0; j<dx; j++)
		{
			for (int k = 0; k<LBP_P; k++)
			{
				double xx = offset_x[k] + hb;
				double yy = offset_y[k] + hb;
				int rx = round(xx);
				int ry = round(yy);

				if ((abs(xx - rx) < 0.000001) && (abs(yy - ry) < 0.000001))
				{
					LBP_bit = Image[(rx + i)*FACE_VERIFY_WIDTH + ry + j] >= Image[(i + hb)*FACE_VERIFY_WIDTH + j + hb];
				}
				else
				{
					double fx = floor(xx);
					double fy = floor(yy);
					double cx = ceil(xx);
					double cy = ceil(yy);
					double tx = xx - fx;
					double ty = yy - fy;
					double w1 = (1 - tx) * (1 - ty);
					double w2 = tx  * (1 - ty);
					double w3 = (1 - tx) *      ty;
					double w4 = tx  *      ty;
					double N = 0;
					N += w1*double(Image[int((fx + i)*FACE_VERIFY_WIDTH + fy + j)]);
					N += w2*double(Image[int((cx + i)*FACE_VERIFY_WIDTH + fy + j)]);
					N += w3*double(Image[int((fx + i)*FACE_VERIFY_WIDTH + cy + j)]);
					N += w4*double(Image[int((cx + i)*FACE_VERIFY_WIDTH + cy + j)]);
					int tt = (abs(N - double(Image[(i + hb)*FACE_VERIFY_WIDTH + j + hb]))<0.000001);
					LBP_bit = N >= Image[(i + hb)*FACE_VERIFY_WIDTH + j + hb];
					LBP_bit = LBP_bit | tt;
				}
				LBPMap[i*dx + j] += LBP_bit*power(2, k);
			}
		}
	}
	return true;
}

void CFaceBasics::InitializeFaceRecognitionModule() // load the templates, aryel
{
	for (int i = 0; i<NUM_OF_REGISTER_USER; i++)
	{
		DWORD cwdsz = GetCurrentDirectory(0, 0); // determine size needed
		wchar_t *cwd = new wchar_t[300];
		GetCurrentDirectory(cwdsz, cwd);
		wstring CurrentFolder = wstring(cwd);
		histSize = 256 * NUM_OF_PATCH*NUM_OF_PATCH*NUM_OF_USER_TEMPLATE;
		//read in verify_face
		m_WatchList[i].verify_face = new double[histSize]; // check the tempalte - verify_face, aryel
		wchar_t FileName[300];
		wsprintf(FileName, L"\\face_recognition_data\\%d", i);
		CurrentFolder.append(FileName);

		FILE* fid;
		_wfopen_s(&fid, CurrentFolder.c_str(), L"rb");
		if (fid == NULL)
		{
			cout << "Not able to open file\n!";
		}
		else
		{
			fread(m_WatchList[i].verify_face, sizeof(double), histSize, fid);
			fclose(fid);
		}

		//read in user name;
		//m_WatchList[i].user_name = new wchar_t[100];
	}

	//m_WatchList[0].user_name = L"";
	//m_WatchList[1].user_name = L"Rubha";
	//m_WatchList[2].user_name = L"Fuwen";
	//m_WatchList[3].user_name = L"Srikanth";
	//m_WatchList[4].user_name = L"Jianfeng";


	time(&previousTime);
}

void CFaceBasics::ReadNames()
{

}

void CFaceBasics::ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees		
	double dPitch, dYaw, dRoll;
	dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
	dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;

	// clamp rotation values in degrees to a specified range of values to control the refresh rate
	double increment = 5;
	*pPitch = static_cast<int>(floor((dPitch + increment / 2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pYaw = static_cast<int>(floor((dYaw + increment / 2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pRoll = static_cast<int>(floor((dRoll + increment / 2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}