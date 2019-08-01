#include <iostream>
#include <sys/timeb.h>
#include "IisuReader.h"
#include "ImageProcessor.h"
using namespace std;

void IisuReader::SetSize(cv::Size sFrame)
{ 
	m_sFrame = sFrame; 

	// allocate the memory for images
	m_mtxColorImg.create(cv::Size(960, 540), CV_8UC4);
	m_mtxDepthImg.create(m_sFrame, CV_32FC1);
	m_mtxDepthPoints.create(m_sFrame, CV_32FC3);
	m_mtxPseudoDepth.create(m_sFrame, CV_8UC3);
	m_mtxLabels.create(m_sFrame, CV_32SC1);
}

void IisuReader::SetProjParams(double fc0[2], double cc0[2], 
	double kc0[5], double alpha_c0)
{
	memcpy(fc, fc0, sizeof(double) * 2);
	memcpy(cc, cc0, sizeof(double) * 2);
	memcpy(kc, kc0, sizeof(double) * 5);
	alpha_c = alpha_c0;
}

cv::Vec3f IisuReader::BackProjectPixel(cv::Point xp, double z)
{
	// perform back linear projection
	cv::Vec2f xd((xp.x - cc[0]) / fc[0], (xp.y - cc[1]) / fc[1]);
	xd[0] = xd[0] - alpha_c * xd[1];

	// check whether to compensate for lens distortion
	bool bComp = false;
	for (int i = 0; i < 5; i++)
	{
		if (kc[i] != 0)
		{
			bComp = true;
			break;
		}
	}

	if (bComp == false)
		return cv::Vec3f(xd[0] * z, xd[1] * z, z);
	else
	{
		cv::Vec2f xn = xd;
		for (int i = 0; i < 10; i++)
		{
			double r_2 = xn[0] * xn[0] + xn[1] * xn[1];
			double k_radial =  1 + kc[0] * r_2 + kc[1] * r_2 * r_2 + kc[4] * r_2 * r_2 * r_2;
			cv::Vec2f delta(2 * kc[2] * xn[0] * xn[1] + kc[3] * (r_2 + 2 * xn[0] * xn[0]),
				kc[2] * (r_2 + 2 * xn[1] * xn[1])+2 * kc[3] * xn[0] * xn[1]);
			xn[0] = (xd[0] - delta[0]) / k_radial;
			xn[1] = (xd[1] - delta[1]) / k_radial;
		}
		return cv::Vec3f(xn[0] * z, xn[1] * z, z);
	}
}

cv::Point IisuReader::ProjectToPixel(cv::Vec3f v)
{
	if (v[2] == 0.0)
		return cv::Point(0, 0);

	cv::Vec2f xn(v[0] / v[2], v[1] / v[2]);
	double r_2 = xn[0] * xn[0] + xn[1] * xn[1];
	double k_radial =  1 + kc[0] * r_2 + kc[1] * r_2 * r_2 + kc[4] * r_2 * r_2 * r_2;
	cv::Vec2f delta(2 * kc[2] * xn[0] * xn[1] + kc[3] * (r_2 + 2 * xn[0] * xn[0]),
		kc[2] * (r_2 + 2 * xn[1] * xn[1])+2 * kc[3] * xn[0] * xn[1]);
	cv::Vec2f xd(xn[0] * k_radial + delta[0], xn[1] * k_radial + delta[1]);
	
	// perform linear projection
	cv::Point xp(fc[0] * (xd[0] + alpha_c * xd[1]) + cc[0], fc[1] * xd[1] + cc[1]);
	return xp;
}

// this function display the color and depth image
void IisuReader::ViewVideo(void)
{
	while (1)
	{
		Update();
		if( cv::waitKey(10) >= 0 )
			break;
	}
}

cv::Mat g_mtxPoints;
void OnMouse( int event, int x, int y, int flags, void* param )
{
	int mx, my;
	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		mx = x;
		my = y;
		cv::Vec3f vPoint = g_mtxPoints.at<cv::Vec3f>(my, mx);
		printf("%d %d: %f, %f, %f\n", mx, my, vPoint[0], vPoint[1], vPoint[2]);
		printf("\n");
		break;
	}
}

void IisuReader::ShowDepth(void)
{
	for (int i = 0; i < m_mtxDepthImg.rows; i++)
	{
		for (int j = 0; j < m_mtxDepthImg.cols; j++)
		{
			double fValue = m_mtxDepthImg.at<float>(i, j);
			m_mtxPseudoDepth.at<cv::Vec3b>(i, j) = MonoToColor(fValue, m_fMinDepth, m_fMaxDepth, true);
		}
	}
	cv::imshow("Depth", m_mtxPseudoDepth);
}
