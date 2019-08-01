#pragma once
#include <cv.h>
#include <cxcore.h>
#include <stdio.h>

const int	HAND_PART_NUM = 14;
const int	HAND_LABELS[HAND_PART_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int	CLASS_NUM = 10;

enum DofType {ROTATION_X, ROTATION_Y, ROTATION_Z, TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z, NOT_A_DOF};
const std::string BoneNames[29] = {"Carpal0", "Carpal1", "Carpal2", "ThumbTM", "ThumbMCP", "ThumbIP", "ThumbTip", "IndexTM", 
	"IndexMCP", "IndexPIP", "IndexDIP", "IndexTip", "MiddleTM", "MiddleMCP", "MiddlePIP", "MiddleDIP", "MiddleTip", "Carpal3", 
	"RingTM", "RingMCP", "RingPIP", "RingDIP", "RingTip", "Carpal4", "PinkyTM", "PinkyMCP", "PinkyPIP", "PinkyDIP", "PinkyTip"};

class DoF
{
public:
	DoF() {}
	DoF(std::string name, DofType type, double value) : Name(name), Type(type), Value(value) {}

public:
	std::string	Name;
	DofType	Type;
	double		Value;
};

class HandParams
{
public:
	cv::Rect			OBB;
	std::vector<DoF>	DofSet;
	cv::Point			PalmCenter2D;
	cv::Vec3f			PalmCenter3D;
	cv::Point			Fingertips2D[5];
	cv::Vec3f			Fingertips3D[5];	
	std::map<std::string, cv::Point>		JointLocs2D;
	std::map<std::string, cv::Vec3f>		JointLocs3D;
};

bool	LoadDepthImage(cv::Mat &mtxDepthImage, const char *strName);
bool	LoadDepthPoints(cv::Mat &mtxDepthPoints, const char *strName);
bool	LoadLabelImage(cv::Mat &mtxLabels, const char *strName);
bool	LoadHandParams(HandParams &param, const char *strName);
bool	LoadFrame(std::string strDir, int nIndex, cv::Mat &mtxDepthPoints, cv::Mat &mtxLabels, HandParams &param);
bool	SaveDepthImage(const cv::Mat &mtxDepthImage, const char *strName);

inline int MapToSampleLabel(int nLabel)
{
	static std::vector<std::vector<int> > vecLabels;
	if (vecLabels.size() == 0)
	{
		int nSet0[5] = {0, 18, 24, 6, 12};
		vecLabels.push_back(std::vector<int>(nSet0, nSet0 + 5));		// wrist
		int nSet1[3] = {25, 19, 1};
		vecLabels.push_back(std::vector<int>(nSet1, nSet1 + 3));		// palm part 1
		int nSet2[2] = {7, 13};
		vecLabels.push_back(std::vector<int>(nSet2, nSet2 + 2));		// palm part 2
		int nSet3[1] = {26};
		vecLabels.push_back(std::vector<int>(nSet3, nSet3 + 1));		// thumb part 1
		int nSet4[2] = {27, 28};
		vecLabels.push_back(std::vector<int>(nSet4, nSet4 + 2));		// thumb part 2
		int nSet5[2] = {20, 21};
		vecLabels.push_back(std::vector<int>(nSet5, nSet5 + 2));		// index part 1
		int nSet6[2] = {22, 23};
		vecLabels.push_back(std::vector<int>(nSet6, nSet6 + 2));		// index part 2
		int nSet7[2] = {2, 3};
		vecLabels.push_back(std::vector<int>(nSet7, nSet7 + 2));		// middle part 1
		int nSet8[2] = {4, 5};
		vecLabels.push_back(std::vector<int>(nSet8, nSet8 + 2));		// middle part 2
		int nSet9[2] = {8, 9};
		vecLabels.push_back(std::vector<int>(nSet9, nSet9 + 2));		// ring part 1
		int nSet10[2] = {10, 11};
		vecLabels.push_back(std::vector<int>(nSet10, nSet10 + 2));	// ring part 2
		int nSet11[2] = {14, 15};
		vecLabels.push_back(std::vector<int>(nSet11, nSet11 + 2));	// pinky part 1
		int nSet12[2] = {16, 17};
		vecLabels.push_back(std::vector<int>(nSet12, nSet12 + 2));	// pinky part 2
	}

	if (nLabel == 255)
		return vecLabels.size();

	int nCount = 0;
	for (std::vector<std::vector<int> >::iterator itl = vecLabels.begin();
		itl != vecLabels.end(); itl++)
	{
		for (std::vector<int>::iterator itsl = itl->begin(); itsl != itl->end(); itsl++)
		{
			if (*itsl == nLabel)
				return nCount;
		}
		nCount++;
	}
	return -1;
}

inline bool LoadUVImg(cv::Mat &mtxUVImg, const char *strName)
{
	FILE *pFile = fopen(strName, "rb");
	if (!pFile)
		return false;
	int nWidth, nHeight;
	fread(&nWidth, sizeof(int), 1, pFile);
	fread(&nHeight, sizeof(int), 1, pFile);
	if (mtxUVImg.cols != nWidth || mtxUVImg.rows != nHeight)
	{
		mtxUVImg.release();
		mtxUVImg.create(cv::Size(nWidth, nHeight), CV_32FC2);
	}
	for (int i = 0; i < nHeight; i++)
	{
		for (int j = 0; j < nWidth; j++)
		{
			float u, v;
			fread(&u, sizeof(float), 1, pFile);
			fread(&v, sizeof(float), 1, pFile);
			mtxUVImg.at<cv::Vec2f>(i, j) = cv::Vec2f(u, v);
		}
	}
	fclose(pFile);
	return true;
}

inline bool SaveDepthPoints(const cv::Mat &mtxDepthPoints, const char *strName)
{
	FILE *pFile = fopen(strName, "wb");
	if (!pFile)
		return false;
	fwrite(&mtxDepthPoints.cols, sizeof(int), 1, pFile);
	fwrite(&mtxDepthPoints.rows, sizeof(int), 1, pFile);
	float *pBuf = new float[mtxDepthPoints.cols * 3];
	for (int i = 0; i < mtxDepthPoints.rows; i++)
	{
		for (int j = 0; j < mtxDepthPoints.cols; j++)
		{
			cv::Vec3f vTmp = mtxDepthPoints.at<cv::Vec3f>(i, j);
			pBuf[3 * j] = vTmp[0];
			pBuf[3 * j + 1] = vTmp[1];
			pBuf[3 * j + 2] = vTmp[2];
		}
		fwrite((void*)pBuf, sizeof(float), 3 * mtxDepthPoints.cols, pFile);
	}
	delete[] pBuf;
	fclose(pFile);
	return true;
}

inline bool SaveLabelImage(const cv::Mat &mtxLabels, const char *strName)
{
	FILE *pFile = fopen(strName, "wb");
	if (!pFile)
		return false;
	fwrite(&mtxLabels.cols, sizeof(int), 1, pFile);
	fwrite(&mtxLabels.rows, sizeof(int), 1, pFile);
	unsigned char *pBuf = new unsigned char[mtxLabels.cols];
	for (int i = 0; i < mtxLabels.rows; i++)
	{
		for (int j = 0; j < mtxLabels.cols; j++)
			pBuf[j] = mtxLabels.at<int>(i, j);
		fwrite((void*)pBuf, sizeof(unsigned char), mtxLabels.cols, pFile);
	}
	delete[] pBuf;
	fclose(pFile);
	return true;
}

inline bool SaveUVImg(const cv::Mat &mtxUVImg, const char *strName)
{
	FILE *pFile = fopen(strName, "wb");
	if (!pFile)
		return false;
	fwrite(&mtxUVImg.cols, sizeof(int), 1, pFile);
	fwrite(&mtxUVImg.rows, sizeof(int), 1, pFile);
	float *pBuf = new float[mtxUVImg.cols * 2];
	for (int i = 0; i < mtxUVImg.rows; i++)
	{
		for (int j = 0; j < mtxUVImg.cols; j++)
		{
			pBuf[2 * j] = mtxUVImg.at<cv::Vec2f>(i, j)[0];
			pBuf[2 * j + 1] = mtxUVImg.at<cv::Vec2f>(i, j)[1];
		}
		fwrite((void*)pBuf, sizeof(float), mtxUVImg.cols * 2, pFile);
	}
	delete[] pBuf;
	fclose(pFile);
	return true;
}

inline bool SaveDepthImage(const cv::Mat &mtxDepthImage, const char *strName)
{
	FILE *pFile = fopen(strName, "wb");
	if (!pFile)
		return false;
	fwrite(&mtxDepthImage.cols, sizeof(int), 1, pFile);
	fwrite(&mtxDepthImage.rows, sizeof(int), 1, pFile);
	float *pBuf = new float[mtxDepthImage.cols];
	for (int i = 0; i < mtxDepthImage.rows; i++)
	{
		for (int j = 0; j < mtxDepthImage.cols; j++)
		{
			float fDepth = mtxDepthImage.at<float>(i, j);
			pBuf[j] = fDepth;
		}
		fwrite((void*)pBuf, sizeof(float), mtxDepthImage.cols, pFile);
	}
	delete[] pBuf;
	fclose(pFile);
	return true;
}