#include "stdafx.h"
#include "IisuKinectReader.h"
#include <highgui.h>
#include <sys/timeb.h>
#include <iostream>
#include "Global.h"
#include "ImageProcessor.h"
using namespace std;

KinectBody::KinectBody(cv::Point ptJoints[JointType_Count], cv::Vec3f vJoints[JointType_Count], 
	TrackingState nStates[JointType_Count], HandState ls, HandState rs)
{
	memcpy(Joints2D, ptJoints, sizeof(cv::Point) * JointType_Count);
	memcpy(Joints3D, vJoints, sizeof(cv::Vec3f) * JointType_Count);
	memcpy(JointStates, nStates, sizeof(TrackingState)* JointType_Count);

	LHand2D = Joints2D[JointType_HandLeft];
	LWrist2D = Joints2D[JointType_WristLeft];
	LElbow2D = Joints2D[JointType_ElbowLeft];
	RHand2D = Joints2D[JointType_HandRight];
	RWrist2D = Joints2D[JointType_WristRight];
	RElbow2D = Joints2D[JointType_ElbowRight];

	LHand3D = Joints3D[JointType_HandLeft];
	LWrist3D = Joints3D[JointType_WristLeft];
	LElbow3D = Joints3D[JointType_ElbowLeft];
	RHand3D = Joints3D[JointType_HandRight];
	RWrist3D = Joints3D[JointType_WristRight];
	RElbow3D = Joints3D[JointType_ElbowRight];

	LHandState = ls;
	RHandState = rs;
	IsMainBody = false;
}

void IisuKinectReader::Init()
{
	m_pKinectSensor = NULL;

	m_sOrgColor = cv::Size(1920, 1080);
	m_sOrgDepth = cv::Size(512, 424);
	m_fMinDepth = 0.3;
	m_fMaxDepth = 2.0;

	for (int i = 0; i < 4; i++)
		m_hFrameEvents[i] = NULL;
}

bool IisuKinectReader::Open(void)
{
	m_nFrmNum = 0;

	HRESULT hr = Nui_Init();
	if (FAILED(hr))
	{
		printf("Fail to open the camera!\n");
		return false;
	}
	return true;
}

void IisuKinectReader::Close(void)
{
	Nui_Clear();
}

HRESULT IisuKinectReader::Nui_Init()
{
	// init the sensor body
	HRESULT hr;
	if (m_pKinectSensor == NULL)
	{
		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
			return hr;
	}

	// init the event handles
	for (int i = 0; i < 4; i++)
		m_hFrameEvents[i] = (WAITABLE_HANDLE)CreateEvent(NULL, FALSE, FALSE, NULL);


	// open depth, color, body index and skeleton readers
	IDepthFrameSource* pDepthFrameSource = NULL;
	IColorFrameSource* pColorFrameSource = NULL;
	IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;
	IBodyFrameSource* pBodyFrameSource = NULL;
	hr = m_pKinectSensor->Open();

	if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	if (SUCCEEDED(hr))	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	if (SUCCEEDED(hr))	hr = m_pDepthFrameReader->SubscribeFrameArrived(&m_hFrameEvents[0]);

	if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	if (SUCCEEDED(hr))	hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	if (SUCCEEDED(hr))	hr = m_pColorFrameReader->SubscribeFrameArrived(&m_hFrameEvents[1]);

	if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
	if (SUCCEEDED(hr))	hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexReader);
	if (SUCCEEDED(hr))	hr = m_pBodyIndexReader->SubscribeFrameArrived(&m_hFrameEvents[2]);

	if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
	if (SUCCEEDED(hr))	hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	if (SUCCEEDED(hr))	hr = m_pBodyFrameReader->SubscribeFrameArrived(&m_hFrameEvents[3]);
	
	SafeRelease(pDepthFrameSource);
	SafeRelease(pColorFrameSource);
	SafeRelease(pBodyIndexFrameSource);
	SafeRelease(pBodyFrameSource);

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << endl;
		return E_FAIL;
	}

	return hr;
}

void IisuKinectReader::Nui_Clear()
{
	// close the event handles
	m_pDepthFrameReader->UnsubscribeFrameArrived(m_hFrameEvents[0]);
	m_pColorFrameReader->UnsubscribeFrameArrived(m_hFrameEvents[1]);
	m_pBodyIndexReader->UnsubscribeFrameArrived(m_hFrameEvents[2]);
	m_pBodyFrameReader->UnsubscribeFrameArrived(m_hFrameEvents[3]);
	for (int i = 0; i < 4; i++)
	{
		CloseHandle((HANDLE)m_hFrameEvents[i]);
		m_hFrameEvents[i] = NULL;
	}

	// done with body frame reader
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

bool IisuKinectReader::Update(void)
{
	HANDLE handles[4];
	for (int i = 0; i < 4; i++)
		handles[i] = reinterpret_cast<HANDLE>(m_hFrameEvents[i]);
	
	int nEventIdx = WaitForMultipleObjects(4, handles, TRUE, 5);
	if (nEventIdx != WAIT_FAILED && nEventIdx != WAIT_TIMEOUT)
	{
	/* !! very important note here, the following commented code generally cannot
		read all data correctly. By debuging the Kinect sample code, we find that
		FAILED(hr) is true for almost all times, and they adopt the strategy to always 
		query whether the data is ready. I think there are some bits to indicate whether
		the different frames are ready, which are constantly updated and overwritten. 
		You can only get the data at the samll instant that it is set to be ready, which 
		means you have to constantly query. Also consider the synchronization problem
		between different frames, i.e. depth, color, label and body, you can hardly 
		succeed with the following commented code to retrieve the data */
	/*	bool bDepth = Nui_NextDepthFrame();
		bool bColor = Nui_NextColorFrame();
		bool bLabel = Nui_NextLabelFrame();
		bool bBody = Nui_NextBodyFrame();*/

		// constant query mode, which works quite well
		while (!Nui_NextDepthFrame());
		while (!Nui_NextColorFrame());
		while (!Nui_NextLabelFrame());
		while (!Nui_NextBodyFrame());
		m_nFrmNum++;

		Nui_DrawFrame();
		ShowDepth();

		// determine the main body for interaction
		int nMainIdx = Nui_FindMainBody(m_vecKinectBodies);
		if (nMainIdx == -1)
			return false;
		else
		{			
			m_mtxHandMask.create(m_mtxDepthImg.size(), CV_32SC1);
			bool bRefine = Nui_FindHandMask(m_mtxDepthImg, m_vecKinectBodies[nMainIdx], m_mtxHandMask, m_bHandActive);
			return bRefine;
		}
	}
	else
		return false;
}

bool IisuKinectReader::Nui_NextColorFrame(void)
{
	if (!m_pColorFrameReader)
		return false;

	IColorFrame* pColorFrame = NULL;
	IFrameDescription* pFrameDescription = NULL;
	int nWidth, nHeight;


	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))	hr = pColorFrame->get_FrameDescription(&pFrameDescription);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Width(&nWidth);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Height(&nHeight);

	// make sure we've received valid data and update the color frame
	if (SUCCEEDED(hr) && (nWidth == m_sOrgColor.width) && (nHeight == m_sOrgColor.height))
	{
		static cv::Mat mtxColor(m_sOrgColor, CV_8UC4);
		UINT nBufferSize = m_sOrgColor.width * m_sOrgColor.height * sizeof(RGBQUAD);
		hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(mtxColor.data), ColorImageFormat_Bgra);
		cv::resize(mtxColor, m_mtxColorImg, m_mtxColorImg.size());

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
	if (FAILED(hr))
		return false;

	return true;
}

bool IisuKinectReader::Nui_NextDepthFrame(void)
{
	if (!m_pDepthFrameReader)
		return false;

	IDepthFrame* pDepthFrame = NULL;
	IFrameDescription* pFrameDescription = NULL;
	USHORT nDepthMinReliableDistance = 0;
	USHORT nDepthMaxReliableDistance = USHRT_MAX;
	int nWidth, nHeight;
	UINT nBufferSize = 0;
	UINT16 *pBuffer = NULL;
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))	hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Width(&nWidth);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Height(&nHeight);
	if (SUCCEEDED(hr))	hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

	// filtering the depth map with predefined or reliable distances
	if (SUCCEEDED(hr))	hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
	if (SUCCEEDED(hr))	hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);

	// make sure we've received valid data and update the depth frame
	if (SUCCEEDED(hr) && pBuffer && (nWidth == m_sOrgDepth.width) && (nHeight == m_sOrgDepth.height))
	{
	//	m_fMinDepth = nDepthMinReliableDistance / 1000.0f;
	//	m_fMaxDepth = nDepthMaxReliableDistance / 1000.0f;

		static cv::Mat mtxDepth(m_sOrgDepth, CV_32FC1);
		static cv::Mat mtxPseudoDepth(m_sOrgDepth, CV_8UC3);
		float* ptr = (float*)mtxDepth.data;
		for (int i = 0; i < nWidth * nHeight; i++)
		{
			float fDepth = (*(pBuffer + i)) / 1000.0f;
			if ((fDepth >= m_fMinDepth) && (fDepth <= m_fMaxDepth))
				ptr[i] = fDepth;
			else
				ptr[i] = 0.0;
		}
		cv::resize(mtxDepth, m_mtxDepthImg, m_mtxDepthImg.size());

		SafeRelease(pFrameDescription);
	}
	SafeRelease(pDepthFrame);

	if (FAILED(hr))
		return false;
	
	return true;
}

bool IisuKinectReader::Nui_NextLabelFrame(void)
{
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	IFrameDescription* pFrameDescription = NULL;
	int nWidth, nHeight;
	UINT nBufferSize = 0;
	BYTE *pBuffer = NULL;

	HRESULT hr = m_pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);

	if (SUCCEEDED(hr))	hr = pBodyIndexFrame->get_FrameDescription(&pFrameDescription);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Width(&nWidth);
	if (SUCCEEDED(hr))	hr = pFrameDescription->get_Height(&nHeight);
	if (SUCCEEDED(hr))	hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);	
	
	// make sure we've received valid data and update the body index frame
	if (SUCCEEDED(hr) && pBuffer && (nWidth == m_sOrgDepth.width) && (nHeight == m_sOrgDepth.height))
	{
		static cv::Mat mtxLabels(m_sOrgDepth, CV_32SC1);
		static cv::Mat mtxLabelColors(m_sOrgDepth, CV_8UC3);
		for (int i = 0; i < nWidth * nHeight; i++)
		{
			unsigned char R, G, B;
			GetDistinctColor(pBuffer[i], R, G, B);

			if (pBuffer[i] != 0xff)
			{
				*((int*)mtxLabels.data + i) = pBuffer[i];
				*((cv::Vec3b*)mtxLabelColors.data + i) = cv::Vec3b(B, G, R);
			}
			else
			{
				*((int*)mtxLabels.data + i) = 0;
				*((cv::Vec3b*)mtxLabelColors.data + i) = cv::Vec3b(0, 0, 0);
			}
		}

		//!! be very careful when resizing the images of integer type, bug of opencv
		cv::resize(mtxLabels, mtxLabels, mtxLabels.size(), 0, 0, cv::INTER_NEAREST);
		cv::imshow("Label", mtxLabelColors);

		SafeRelease(pFrameDescription);
	}
	SafeRelease(pBodyIndexFrame);

	if (FAILED(hr))
		return false;

	return true;
}


bool IisuKinectReader::Nui_NextBodyFrame(void)
{
	if (!m_pBodyFrameReader)
		return false;

	IBodyFrame* pBodyFrame = NULL;
	IBody* ppBodies[BODY_COUNT] = { 0 };

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hr))	
	{
		hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);

		if (SUCCEEDED(hr))
		{
			m_vecKinectBodies.clear();

			for (int i = 0; i < BODY_COUNT; ++i)
			{
				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);					
					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;

						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);
						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							cv::Point ptJoints[JointType_Count];
							cv::Vec3f vJoints[JointType_Count];
							TrackingState	nStates[JointType_Count];
							for (int j = 0; j < _countof(joints); ++j)
							{
								vJoints[joints[j].JointType] = cv::Vec3f(joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z);
								ptJoints[joints[j].JointType] = ProjectToPixel(vJoints[joints[j].JointType]);
								nStates[joints[j].JointType] = joints[j].TrackingState;
							}
							KinectBody kbTmp(ptJoints, vJoints, nStates, leftHandState, rightHandState);
							pBody->get_TrackingId(&(kbTmp.BodyIndex));
							m_vecKinectBodies.push_back(kbTmp);
						}
					}
				}
			}
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);

	if (FAILED(hr))
		return false;

	return true;
}

int IisuKinectReader::Nui_FindMainBody(std::vector<KinectBody>& vecKinectBodies)
{
	static UINT64 nTrackingID = -1;

	// reinitialize the tracking id if nobody is detected
	if (vecKinectBodies.size() == 0)
	{
		nTrackingID = -1;

		return -1;
	}
	else
	{
		// initialize the tracking id if anybody is detected
		if (nTrackingID == -1)
		{
			// find the one nearest to the camera
			double fMinDist = 1e10;
			int nMainIdx = -1;
			for (int i = 0; i < vecKinectBodies.size(); i++)
			{
				vecKinectBodies[i].IsMainBody = false;
				double fDist = cv::norm(vecKinectBodies[i].Joints3D[JointType_SpineMid]);
				if (fDist < fMinDist)
				{
					fMinDist = fDist;
					nMainIdx = i;
				}
			}
			vecKinectBodies[nMainIdx].IsMainBody = true;
			nTrackingID = vecKinectBodies[nMainIdx].BodyIndex;

			return nMainIdx;
		}
		else
		{
			// track the body with the same tracking id
			int nMainIdx = -1;
			for (int i = 0; i < vecKinectBodies.size(); i++)
			{
				if (vecKinectBodies[i].BodyIndex == nTrackingID)
				{
					vecKinectBodies[i].IsMainBody = true;
					nMainIdx = i;
				}
				else
					vecKinectBodies[i].IsMainBody = false;
			}

			return nMainIdx;
		}
	}
}

void IisuKinectReader::Nui_DrawFrame(void)
{
	cv::Mat mtxRGB(m_mtxColorImg.clone());
	for (int i = 0; i < m_vecKinectBodies.size(); i++)
		Nui_DrawBody(mtxRGB, m_vecKinectBodies[i]);

	cv::imshow("co", mtxRGB);
}

void IisuKinectReader::Nui_DrawBody(cv::Mat &mtxRGBCanvas, KinectBody& body)
{
	// Torso
	Nui_DrawBone(mtxRGBCanvas, body, JointType_Head, JointType_Neck);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_Neck, JointType_SpineShoulder);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineShoulder, JointType_SpineMid);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineMid, JointType_SpineBase);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineShoulder, JointType_ShoulderRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineShoulder, JointType_ShoulderLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineBase, JointType_HipRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	Nui_DrawBone(mtxRGBCanvas, body, JointType_ShoulderRight, JointType_ElbowRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_ElbowRight, JointType_WristRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_WristRight, JointType_HandRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_HandRight, JointType_HandTipRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	Nui_DrawBone(mtxRGBCanvas, body, JointType_ShoulderLeft, JointType_ElbowLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_ElbowLeft, JointType_WristLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_WristLeft, JointType_HandLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_HandLeft, JointType_HandTipLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	Nui_DrawBone(mtxRGBCanvas, body, JointType_HipRight, JointType_KneeRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_KneeRight, JointType_AnkleRight);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	Nui_DrawBone(mtxRGBCanvas, body, JointType_HipLeft, JointType_KneeLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_KneeLeft, JointType_AnkleLeft);
	Nui_DrawBone(mtxRGBCanvas, body, JointType_AnkleLeft, JointType_FootLeft);

	// draw the joints
	for (int i = 0; i < JointType_Count; i++)
	{
		cv::Point pt0 = Nui_ProjectDepthToColorPixel(body.Joints3D[i]);
		if (pt0.x != -std::numeric_limits<int>::infinity() && pt0.y != -std::numeric_limits<int>::infinity())
		{
			cv::Point ptRGB(1.0 * pt0.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * pt0.y / m_sOrgColor.height * mtxRGBCanvas.rows);
			cv::circle(mtxRGBCanvas, ptRGB, 6, cv::Scalar(0, 255, 0), -1);
		}
	}

	// highlight the middle spine to determine whether or not activate gesture recognition
	cv::Point ptSignal = Nui_ProjectDepthToColorPixel(body.Joints3D[JointType_SpineMid]);
	if (ptSignal.x != -std::numeric_limits<int>::infinity() && ptSignal.y != -std::numeric_limits<int>::infinity())
	{
		cv::Point ptRGB(1.0 * ptSignal.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * ptSignal.y / m_sOrgColor.height * mtxRGBCanvas.rows);
		cv::circle(mtxRGBCanvas, ptRGB, 8, cv::Scalar(0, 0, 255), -1);
	}

	// draw the left hand
	cv::Point ptLHand = Nui_ProjectDepthToColorPixel(body.LHand3D);
	if (ptLHand.x != -std::numeric_limits<int>::infinity() && ptLHand.y != -std::numeric_limits<int>::infinity())
	{
		cv::Point ptLHandRGB(1.0 * ptLHand.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * ptLHand.y / m_sOrgColor.height * mtxRGBCanvas.rows);
		switch (body.LHandState)
		{
		case HandState_Closed:
			cv::circle(mtxRGBCanvas, ptLHandRGB, 15, cv::Scalar(255, 0, 0), -1);
			break;
		case HandState_Open:
			cv::circle(mtxRGBCanvas, ptLHandRGB, 15, cv::Scalar(0, 255, 0), -1);
			break;
		case HandState_Lasso:
			cv::circle(mtxRGBCanvas, ptLHandRGB, 15, cv::Scalar(0, 0, 255), -1);
			break;
		}
	}

	// draw the right hand
	cv::Point ptRHand = Nui_ProjectDepthToColorPixel(body.RHand3D);
	if (ptRHand.x != -std::numeric_limits<int>::infinity() && ptRHand.y != -std::numeric_limits<int>::infinity())
	{
		cv::Point ptRHandRGB(1.0 * ptRHand.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * ptRHand.y / m_sOrgColor.height * mtxRGBCanvas.rows);
		switch (body.RHandState)
		{
		case HandState_Closed:
			cv::circle(mtxRGBCanvas, ptRHandRGB, 15, cv::Scalar(255, 0, 0), -1);
			break;
		case HandState_Open:
			cv::circle(mtxRGBCanvas, ptRHandRGB, 15, cv::Scalar(0, 255, 0), -1);
			break;
		case HandState_Lasso:
			cv::circle(mtxRGBCanvas, ptRHandRGB, 15, cv::Scalar(0, 0, 255), -1);
			break;
		}
	}
}

void IisuKinectReader::Nui_DrawBone(cv::Mat &mtxRGBCanvas, const KinectBody& body, JointType joint0, JointType joint1)
{
	TrackingState joint0State = body.JointStates[joint0];
	TrackingState joint1State = body.JointStates[joint1];

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
	cv::Point pt0 = Nui_ProjectDepthToColorPixel(body.Joints3D[joint0]);
	cv::Point pt1 = Nui_ProjectDepthToColorPixel(body.Joints3D[joint1]);
	if (pt0.x != -std::numeric_limits<int>::infinity() && pt0.y != -std::numeric_limits<int>::infinity()
		&& pt1.x != -std::numeric_limits<int>::infinity() && pt1.y != -std::numeric_limits<int>::infinity())
	{
		cv::Point ptRGB0(1.0 * pt0.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * pt0.y / m_sOrgColor.height * mtxRGBCanvas.rows);
		cv::Point ptRGB1(1.0 * pt1.x / m_sOrgColor.width * mtxRGBCanvas.cols, 1.0 * pt1.y / m_sOrgColor.height * mtxRGBCanvas.rows);
		if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
		{
			if (body.IsMainBody == true)
				cv::line(mtxRGBCanvas, ptRGB0, ptRGB1, cv::Scalar(0, 0, 255), 4);
			else
				cv::line(mtxRGBCanvas, ptRGB0, ptRGB1, cv::Scalar(255, 0, 0), 4);
		}
		else
		{
			if (body.IsMainBody == true)
				cv::line(mtxRGBCanvas, ptRGB0, ptRGB1, cv::Scalar(0, 0, 255), 2);
			else
				cv::line(mtxRGBCanvas, ptRGB0, ptRGB1, cv::Scalar(255, 0, 0), 2);
		}
	}
}

cv::Point IisuKinectReader::Nui_ProjectDepthToColorPixel(cv::Vec3f v)
{
	CameraSpacePoint v0;
	ColorSpacePoint pt0;
	v0.X = v[0];
	v0.Y = v[1];
	v0.Z = v[2];
	m_pCoordinateMapper->MapCameraPointToColorSpace(v0, &pt0);

	if (pt0.X < 0 || pt0.Y < 0 || pt0.X >= 1920 || pt0.Y >= 1080)
		pt0 = pt0;

	if (pt0.X != -std::numeric_limits<float>::infinity() && pt0.Y != -std::numeric_limits<float>::infinity())
		return cv::Point(pt0.X, pt0.Y);
	else
		return cv::Point(-std::numeric_limits<int>::infinity(), -std::numeric_limits<int>::infinity());
}

// calculate 3D point projected on depth image plane
cv::Point IisuKinectReader::ProjectToPixel(cv::Vec3f v)
{
	DepthSpacePoint depthPoint = { 0 };
	CameraSpacePoint vCamPoint;
	vCamPoint.X = v[0];
	vCamPoint.Y = v[1];
	vCamPoint.Z = v[2];
	m_pCoordinateMapper->MapCameraPointToDepthSpace(vCamPoint, &depthPoint);

//	CameraIntrinsics pCamParams;
//	m_pCoordinateMapper->GetDepthCameraIntrinsics(&pCamParams);

	int nPointX = (depthPoint.X * m_sFrame.width) / m_sOrgDepth.width;
	int nPointY = (depthPoint.Y * m_sFrame.height) / m_sOrgDepth.height;

	return cv::Point(nPointX, nPointY);
}

cv::Vec3f IisuKinectReader::BackProjectPixel(cv::Point xp, double z)
{
	DepthSpacePoint depthPoint;
	CameraSpacePoint vCamPoint;
	depthPoint.X = xp.x;
	depthPoint.Y = xp.y;
	m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, z * 1000, &vCamPoint);
	
	return cv::Vec3f(vCamPoint.X, vCamPoint.Y, vCamPoint.Z);
}

bool IisuKinectReader::Nui_FindHandMask(cv::Mat &mtxDepthImg, const KinectBody& mainbody, cv::Mat &mtxMask, bool bActive[2])
{
	// threshold the input depth image	
	mtxMask.setTo(cv::Scalar(255));
	int xmin, xmax, ymin, ymax;
	xmin = ymin = _INT_MAX;
	xmax = ymax = _INT_MIN;
	for (int i = 0; i < mtxDepthImg.rows; i++)
	{
		for (int j = 0; j < mtxDepthImg.cols; j++)
		{
			double fValue = mtxDepthImg.at<float>(i, j);
			if (fValue >= m_fMinDepth && fValue <= m_fMaxDepth)
			{
				xmin = j < xmin ? j : xmin;
				xmax = j > xmax ? j : xmax;
				ymin = i < ymin ? i : ymin;
				ymax = i > ymax ? i : ymax;
				mtxMask.at<int>(i, j) = 0;
			}
		}
	}

	cv::Point2f ptPalmCenter[2] = { mainbody.Joints2D[JointType_HandLeft], mainbody.Joints2D[JointType_HandRight] };
	cv::Vec3f vPalmCenter[2] = { mainbody.Joints3D[JointType_HandLeft], mainbody.Joints3D[JointType_HandRight] };
	cv::Point2f ptArmCenter[2] = { mainbody.Joints2D[JointType_WristLeft], mainbody.Joints2D[JointType_WristRight] };

	// determine the active hands and whether further refinement is needed
	bool bRefine[2] = { false, false };
	double d0x[2], d0y[2];
	for (int i = 0; i < 2; i++)
	{
	//	if (ptPalmCenter[i].y > ptArmCenter[i].y + 10)
		if (ptPalmCenter[i].y > mainbody.Joints2D[JointType_SpineMid].y)
			bActive[i] = false;
		else
		{
			bActive[i] = true;

			// if the distance between the palm center and arm center is too small, no refinement 
			// is needed as the arm region are actually not included in the foreground
			cv::Vec2f vDist(ptArmCenter[i].x - ptPalmCenter[i].x, ptArmCenter[i].y - ptPalmCenter[i].y);
			d0x[i] = ptArmCenter[i].x - ptPalmCenter[i].x;
			d0y[i] = ptArmCenter[i].y - ptPalmCenter[i].y;
			if (sqrt(cv::norm(vDist)) > 1.0)
				bRefine[i] = true;
		}
	}
	if (!bActive[0] && !bActive[1])
		return false;

	// apply the hand joint positions to refine the depth image
	for (int i = ymin; i < ymax + 1; i++)
	{
		for (int j = xmin; j < xmax + 1; j++)
		{
			double fDepth = mtxDepthImg.at<float>(i, j);
			if (fDepth != 0.0)
			{
				double fMinDist = 1e10;
				int nIndex = 0;
				double dcx[2], dcy[2];
				bool bInlier = false;
				for (int k = 0; k < 2; k++)
				{
					if (bActive[k] && fDepth - vPalmCenter[k][2] < 0.04 && fDepth - vPalmCenter[k][2] > -0.09)
					{
						bInlier = true;
						dcx[k] = j - ptPalmCenter[k].x;
						dcy[k] = i - ptPalmCenter[k].y;
						double fDist = sqrtf(dcx[k] * dcx[k] + dcy[k] * dcy[k]);
						if (fDist < fMinDist)
						{
							fMinDist = fDist;
							nIndex = k;
						}
					}
				}
				if (bInlier)
				{
					mtxMask.at<int>(i, j) = nIndex;

					double fThreshold = 30 * 0.95 / vPalmCenter[nIndex][2];
					if (fMinDist > fThreshold * 1.5)
					{
						mtxMask.at<int>(i, j) = 255;
					}
			/*		else if (bRefine[nIndex] && d0x[nIndex] * dcx[nIndex] + d0y[nIndex] * dcy[nIndex] > 0)
					{
						if (fMinDist > fThreshold * 1.3)
						{
							mtxMask.at<int>(i, j) = 255;
						}
					}*/
				}
				else
				{
					mtxMask.at<int>(i, j) = 255;
				}
			}
		}
	}
	return true;
}