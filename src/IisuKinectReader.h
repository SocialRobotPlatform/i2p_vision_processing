#pragma once

#define _WINSOCKAPI_ //before windows.h conflicts with winsock2
#include <Windows.h>
#include <Kinect.h>
#include <cv.h>
#include "IisuReader.h"

class KinectBody
{
public:
	KinectBody(void) : IsMainBody(false){}
	KinectBody(cv::Point ptJoints[JointType_Count], cv::Vec3f vJoints[JointType_Count],
		TrackingState nStates[JointType_Count], HandState ls, HandState rs);

public:
	cv::Point	Joints2D[JointType_Count];		// note the joint array is indexed by joint type
	cv::Vec3f	Joints3D[JointType_Count];		// that is, use it like Joints2D[Joint_Type], rather than Joints2D[i]
	cv::Point	LHand2D, LWrist2D, LElbow2D;
	cv::Point	RHand2D, RWrist2D, RElbow2D;
	cv::Vec3f	LHand3D, LWrist3D, LElbow3D;
	cv::Vec3f	RHand3D, RWrist3D, RElbow3D;
	HandState	LHandState, RHandState;
	bool		IsMainBody;
	UINT64		BodyIndex;
	TrackingState	JointStates[JointType_Count];
};

class IisuKinectReader : public IisuReader
{
public:
	IisuKinectReader(void) { Init(); }
	~IisuKinectReader(void) { Clear(); }

public:	
	void		Init(void);
	bool		Open(void);
	bool		Update(void);
	void		Close(void);
	void		GetHandInfo(cv::Mat &mtxDepthImg, cv::Mat &mtxMask, bool bActive[2])
	{
		m_mtxDepthImg.copyTo(mtxDepthImg);
		m_mtxHandMask.copyTo(mtxMask);
		bActive[0] = m_bHandActive[0];
		bActive[1] = m_bHandActive[1];
	}
	
private:
	HRESULT		Nui_Init(void);
	void		Nui_Clear(void);
	bool		Nui_NextColorFrame(void);
	bool		Nui_NextDepthFrame(void);
	bool		Nui_NextBodyFrame(void);
	bool		Nui_NextLabelFrame(void);
	void		Nui_DrawFrame(void);
	void		Nui_DrawBody(cv::Mat &mtxRGBCanvas, KinectBody& body);
	void		Nui_DrawBone(cv::Mat &mtxRGBCanvas, const KinectBody& body, JointType joint0, JointType joint1);
	cv::Point	Nui_ProjectDepthToColorPixel(cv::Vec3f v);
	int			Nui_FindMainBody(std::vector<KinectBody>& vecKinectBodies);
	bool		Nui_FindHandMask(cv::Mat &mtxDepthImg, const KinectBody& mainbody, cv::Mat &mtxMask, bool bActive[2]);


public:
	virtual cv::Point		ProjectToPixel(cv::Vec3f v);
	virtual cv::Vec3f		BackProjectPixel(cv::Point xp, double z);
	
private:
	// it is possible to use IMultiSourceFrameReader instead of all the separate frame readers
	// however, we find the frame rate of IMultiSourceFrameReader is much lower in run-time

	IKinectSensor*			m_pKinectSensor;
	IDepthFrameReader*      m_pDepthFrameReader;
	IColorFrameReader*      m_pColorFrameReader;
	IBodyFrameReader*       m_pBodyFrameReader;
	IBodyIndexFrameReader*	m_pBodyIndexReader;
	ICoordinateMapper*      m_pCoordinateMapper;
	WAITABLE_HANDLE			m_hFrameEvents[4];

private:
	std::vector<KinectBody>	m_vecKinectBodies;
	cv::Mat					m_mtxHandMask;
	bool					m_bHandActive[2];

	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};

