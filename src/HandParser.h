#pragma once
#include "RandomForest.h"

class HandParser
{
public:
	HandParser(void) {}
	virtual~HandParser(void) {}
	
public:
	void	Init(void) {}
	void	SetRFParams(double f0, double fu0, double fv0, int nAnchor, double fRadius, cv::Size sFrame, std::string strForestFile);
	bool	Update(const cv::Mat &mtxDepthImg, const cv::Mat &mtxMask, bool bActive[2], int nGestures[2]);

private:		// main threads
	void	PrepareDepth(const cv::Mat &mtxDepthImg, const cv::Mat &mtxMask, int nLabel, bool bRight, cv::Mat &mtxDepthRF);
	int		InferGesture(const cv::Mat &mtxDepthImg);
	void	ShowParsedParts(cv::Mat &mtxPartLabels);

	
private:
	RandomForest		m_gRF;
};
