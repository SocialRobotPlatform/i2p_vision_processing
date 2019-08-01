//------------------------------------------------------------------------------
// <copyright file="FaceBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include "FaceDlg.h"
//#include "afxwin.h"

#include <iostream>
#include <string>

#include "ThriftTools.hpp"
#include "Facebook.h"
#include "UserTrackingServiceNewKinect.h"
#include "ProtectedClient.h"
#include "Inputs_constants.h"
#include "SocialNetwork_constants.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
using namespace cv;
//Global//
extern float SkeletonDistance;
//Jianfeng//


#define NUM_OF_PATCH 7
#define LBP_P 8
#define LBP_R 2
#define NUM_OF_USER_TEMPLATE 10
#define NUM_OF_REGISTER_USER 20
#define currentTrainedUser 20

#define CORP_EYE_DISTANCE 70.0f
#define CORP_EYE_T_EDGE 46.0f
//#define CORP_EYE_B_EDGE 68.0f
#define CORP_EYE_L_EDGE 30.0f
//#define CORP_EYE_R_EDGE 19.0f
//#define CORP_WIN_HEIGHT 96.0f
//#define CORP_WIN_WIDTH 82.0f

#define CORP_EYE_DISTANCE_ED 43.0f
#define CORP_EYE_T_EDGE_ED 28.0f
#define CORP_EYE_L_EDGE_ED 19.0f

#define FACE_VERIFY_HEIGHT 150
#define FACE_VERIFY_WIDTH 130


#define M_PI       3.14159265358979323846

struct UserInfo
{
	double* verify_face;
	const wchar_t* user_name;
	//double threshold;
};

class CFaceBasics
{
    static const int       cColorWidth  = 1920;
    static const int       cColorHeight = 1080;

	struct Hand_Shake
	{
		Hand_Shake();
		float last_position;
		float shake_temp;
	};

	std::vector<Hand_Shake *> hand_shake_users;

	struct Wave
	{
		Wave();
		int result;
		int frameCount;
		int currentSegment;
		int res[4];
		float position;
		float temp, temp1;
	};

	std::vector<Wave *> wave_users;

	struct L_Wave
	{
		L_Wave();
		int L_result;
		int L_frameCount;
		int L_currentSegment;
		int L_res[4];
		float L_position;
		float L_temp, L_temp1;
	};

	std::vector<L_Wave *>L_wave_users;

public:
	void HandShakingGesture();

	void WaveHandGesture();
	int WaveRightSegment1(imi::Skeleton ges_skel1, float ges_skel2, int k);
	int WaveRightSegment2(imi::Skeleton ges_skel1, float ges_skel2, int k);
	int ges_success_id;

	void L_WaveHandGesture();
	int L_WaveLeftSegment1(imi::Skeleton L_ges_skel1, float L_ges_skel2, int L_k);
	int L_WaveLeftSegment2(imi::Skeleton L_ges_skel1, float L_ges_skel2, int L_k);
	int L_ges_success_id;



    /// <summary>
    /// Constructor
    /// </summary>
    CFaceBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CFaceBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK  MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK       DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                    Run(HINSTANCE hInstance, int nCmdShow);

private:
    /// <summary>
    /// Main processing function
    /// </summary>
    void                   Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success else the failure code</returns>
    HRESULT                InitializeDefaultSensor();

    /// <summary>
    /// Renders the color and face streams
    /// </summary>			
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    void                   DrawStreams(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);

    /// <summary>
    /// Processes new face frames
    /// </summary>
    void                   ProcessFaces();

    /// <summary>
    /// Computes the face result text layout position by adding an offset to the corresponding 
    /// body's head joint in camera space and then by projecting it to screen space
    /// </summary>
    /// <param name="pBody">pointer to the body data</param>
    /// <param name="pFaceTextLayout">pointer to the text layout position in screen space</param>
    /// <returns>indicates success or failure</returns>
    HRESULT                GetFaceTextPositionInColorSpace(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout);
	CameraSpacePoint GetFaceTextPositionInColorSpace_1(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout);
	HRESULT GetJointsInColorSpace(IBody* pBody, D2D1_POINT_2F* pJoint);

	int CFaceBasics::getClothColor(Mat hsvImg, D2D1_POINT_2F* pJoints);
	int CFaceBasics::FaceVerification(PointF EyeLeft, PointF EyeRight);
	BOOL CFaceBasics::GeoNormal(BYTE* Image,PointF EyeLeft, PointF EyeRight,BYTE* CorpFaceImg);
	int CFaceBasics::power(int a, int b);
	int CFaceBasics::round(double floatNumber);
	BOOL CFaceBasics::calHistForWholeImage(BYTE* LBP_image, double* hist);
	BOOL CFaceBasics::calLBPImg(BYTE* Image,BYTE* LBPMap);
	void CFaceBasics::InitializeFaceRecognitionModule();
	void CFaceBasics::ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll);


    /// <summary>
    /// Updates body data
    /// </summary>
    /// <param name="ppBodies">pointer to the body data storage</param>
    /// <returns>indicates success or failure</returns>
    HRESULT                UpdateBodyData(IBody** ppBodies);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    /// <returns>success or failure</returns>
    bool                   SetStatusMessage(_In_z_ WCHAR* szMessage, ULONGLONG nShowTimeMsec, bool bForce);

    HWND                   m_hWnd;
    INT64                  m_nStartTime;
    INT64                  m_nLastCounter;
    double                 m_fFreq;
    ULONGLONG              m_nNextStatusTime;
    DWORD                  m_nFramesSinceUpdate;

	//face recognition	
	uint8_t m_bKeyDown[256];
	uint8_t m_bKeyPressed[256];
	BOOL m_IsStartRecord, m_Slow;
	int newUserTemplateNum;
	
	UserInfo m_WatchList[NUM_OF_REGISTER_USER];
	time_t currentTime;
	time_t previousTime;

	int frame_count;
	void ReadNames();
	int face_frame_count;
	// Current Kinect
    IKinectSensor*         m_pKinectSensor;

    // Coordinate mapper
    ICoordinateMapper*     m_pCoordinateMapper;

    // Color reader
    IColorFrameReader*     m_pColorFrameReader;

    // Body reader
    IBodyFrameReader*      m_pBodyFrameReader;

    // Face sources
    IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

    // Face readers
    IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];

    // Direct2D
    ImageRenderer*         m_pDrawDataStreams;
    ID2D1Factory*          m_pD2DFactory;
    RGBQUAD*               m_pColorRGBX;

	ProtectedClient<imi::UserTrackingServiceNewKinectClient> *m_client;
	ProtectedClient<imi::FacebookClient> *m_client_facebook;
	ProtectedClient<imi::FacebookClient> *m_client_facebook1;

//	ProtectedClient<imi::UserTrackingServiceNewKinectClient> *m_client;
};

