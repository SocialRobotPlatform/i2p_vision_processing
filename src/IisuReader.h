#pragma once
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

class IisuReader
{
public:
	IisuReader(void)	{ Init(); }
	virtual~IisuReader(void) { Clear(); }

public:	
	virtual void	Init(void) {}
	virtual void	Clear(void) {}
	virtual bool	Open(void) { return true; }
	virtual void	Close(void) {}
	virtual bool	Update(void) { return true; }
	void			ViewVideo(void);
	void			ShowDepth(void);
	virtual void	SetSize(cv::Size sFrame);
	void			SetProjParams(double fc0[2], double cc0[2], double kc0[5], double alpha_c0);
	void			SetDepthRange(double min, double max) { m_fMinDepth = min; m_fMaxDepth = max; }
	
public:
	int				GetFrameNum(void) { return m_nFrmNum; }
	cv::Mat&		GetDepthPoints(void) { return m_mtxDepthPoints; }
	cv::Mat&		GetColorImg(void) { return m_mtxColorImg; }
	cv::Mat&		GetLabels(void) { return m_mtxLabels; }
	cv::Mat&		GetPseudoDepth(void) { return m_mtxPseudoDepth; }

public:
	virtual cv::Point	ProjectToPixel(cv::Vec3f v);
	virtual cv::Vec3f	BackProjectPixel(cv::Point xp, double z);

protected:
	cv::Mat		m_mtxDepthImg;
	cv::Mat		m_mtxColorImg;
	cv::Mat		m_mtxDepthPoints;
	cv::Mat		m_mtxPseudoDepth;
	cv::Mat		m_mtxLabels;				// 0 for left hand, 1 for right hand, 255 for background

protected:
	cv::Size	m_sFrame;					// refers to depth frames
	cv::Size	m_sOrgColor, m_sOrgDepth;
	int			m_nFrmNum;
	double		m_fMinDepth, m_fMaxDepth;

private:		// the camera projection parameters
	double		fc[2];
	double		cc[2];
	double		kc[5];
	double		alpha_c;
};