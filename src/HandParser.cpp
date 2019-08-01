#include "HandParser.h"
#include "Global.h"
#include "ImageProcessor.h"
#include "DepthUtilities.h"
#include "FileIO.h"
#include <sys/timeb.h>
#include <omp.h>
using namespace std;

void HandParser::SetRFParams(double f0, double fu0, double fv0, 
	int nAnchor, double fRadius, cv::Size sFrame, std::string strForestFile)
{
	int nFeatureDim = (2 * nAnchor + 1) * (2 * nAnchor + 1) - 1;
	SetFeatureParam(f0, fu0, fv0, sFrame.width / 2, sFrame.height / 2, 2.0, 0.3);
	GenerateFeatureIndicesApt(g_vecFeatureIndices, nFeatureDim, nAnchor, fRadius);
	m_gRF.readForest(strForestFile);
}

bool HandParser::Update(const cv::Mat &mtxDepthImg, const cv::Mat &mtxMask, bool bActive[2], int nGestures[2])
{
	cv::Mat mtxMaskImg(mtxMask.size(), CV_8UC3);
	mtxMaskImg.setTo(cv::Scalar(255, 255, 255));
	for (int i = 0; i < mtxMask.rows; i++)
	{
		for (int j = 0; j < mtxMask.cols; j++)
		{
			if (mtxMask.at<int>(i, j) == 0)
				mtxMaskImg.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
			else if (mtxMask.at<int>(i, j) == 1)
				mtxMaskImg.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
		}
	}

	if (!bActive[0] && !bActive[1])
		return false;
		
	char text[255];
/*	static int s_nSaveIdx = 0;
	string strPath = "J:\\Kinect2 Gesture\\Dataset\\LIANG_HUI\\9";
	sprintf(text, "%s\\label_%d.jpg", strPath.c_str(), s_nSaveIdx);
	cv::imwrite(text, mtxMaskImg);
	sprintf(text, "%s\\label_%d.rhd", strPath.c_str(), s_nSaveIdx);
	SaveLabelImage(mtxMask, text);
	sprintf(text, "%s\\depth_%d.rhd", strPath.c_str(), s_nSaveIdx);
	SaveDepthImage(mtxDepthImg, text);
	s_nSaveIdx++;*/

	static cv::Mat mtxDepthRF(mtxDepthImg.size(), CV_32FC1);

	// determine the left hand gesture
	if (bActive[0])
	{
		PrepareDepth(mtxDepthImg, mtxMask, 0, false, mtxDepthRF);
		nGestures[0] = InferGesture(mtxDepthRF);
	}
	else
		nGestures[0] = -1;

	// determine the right hand gesture
	if (bActive[1])
	{
		PrepareDepth(mtxDepthImg, mtxMask, 1, true, mtxDepthRF);
		nGestures[1] = InferGesture(mtxDepthRF);
	}
	else
		nGestures[1] = -1;

//	char text[255];
	sprintf(text, "Left: %s %d, Right: %s %d", bActive[0] ? "Active" : "Inactive", nGestures[0], bActive[1] ? "Active" : "Inactive", nGestures[1]);
//	sprintf(text, "save: %d", s_nSaveIdx);
	cv::putText(mtxMaskImg, std::string(text), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 0, 0));
	cv::imshow("Hand", mtxMaskImg);
	return true;
}

void HandParser::PrepareDepth(const cv::Mat &mtxDepthImg, const cv::Mat &mtxMask, int nLabel, bool bRight, cv::Mat &mtxDepthRF)
{
	mtxDepthRF.setTo(cv::Scalar(g_fPlaneDepth));
	for (int i = 0; i < mtxDepthImg.rows; i++)
	{
		for (int j = 0; j < mtxDepthImg.cols; j++)
		{
			if (mtxMask.at<int>(i, j) == nLabel)
				mtxDepthRF.at<float>(i, j) = mtxDepthImg.at<float>(i, j);
		}
	}

	// horizontal flip for left hand image
	if (!bRight)
		cv::flip(mtxDepthRF, mtxDepthRF, 1);
}


void HandParser::ShowParsedParts(cv::Mat &mtxPartLabels)
{
	static cv::Mat mtxTest(mtxPartLabels.rows, mtxPartLabels.cols, CV_8UC3);
	for (int i = 0; i < mtxPartLabels.rows; i++)
	{
		for (int j = 0; j < mtxPartLabels.cols; j++)
		{
			int nLabel = mtxPartLabels.at<unsigned char>(i, j);	
			unsigned char R, G, B;
			GetDistinctColor(nLabel, R, G, B);
			mtxTest.at<cv::Vec3b>(i, j) = cv::Vec3b(B, G, R);
		}
	}

//	cv::imshow("PredLabel", mtxTest);
}

int HandParser::InferGesture(const cv::Mat &mtxDepthImg)
{
	g_mtxDepthImg = mtxDepthImg;
	std::vector<double> vecGestConf(10, 0.0);

#pragma omp parallel for ordered
	for (int i = 0; i < mtxDepthImg.rows; i++)
	{
		for (int j = 0; j < mtxDepthImg.cols; j++)
		{
			if (mtxDepthImg.at<float>(i, j) != g_fPlaneDepth)
			{
				Result result = m_gRF.eval(cv::Point(j, i));
#pragma omp critical
				{
					add(result.Confidence, vecGestConf);
				}
			}
		}
	}
	int nGesture = argmax(vecGestConf);
	return nGesture;
}