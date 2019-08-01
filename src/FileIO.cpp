#include "FileIO.h"
#include <highgui.h>

bool LoadDepthImage(cv::Mat &mtxDepthImage, const char *strName)
{
	FILE *pFile = fopen(strName, "rb");
	if (!pFile)
		return false;
	int nWidth, nHeight;
	fread(&nWidth, sizeof(int), 1, pFile);
	fread(&nHeight, sizeof(int), 1, pFile);
	if (mtxDepthImage.cols != nWidth || mtxDepthImage.rows != nHeight)
	{
		mtxDepthImage.release();
		mtxDepthImage.create(cv::Size(nWidth, nHeight), CV_32FC1);
	}
	for (int i = 0; i < mtxDepthImage.rows; i++)
	{
		for (int j = 0; j < mtxDepthImage.cols; j++)
		{
			float fDepth;
			fread(&fDepth, sizeof(float), 1, pFile);
			mtxDepthImage.at<float>(i, j) = fDepth;
		}
	}
	fclose(pFile);
	return true;
}

bool LoadDepthPoints(cv::Mat &mtxDepthPoints, const char *strName)
{
	FILE *pFile = fopen(strName, "rb");
	if (!pFile)
		return false;
	int nWidth, nHeight;
	fread(&nWidth, sizeof(int), 1, pFile);
	fread(&nHeight, sizeof(int), 1, pFile);
	if (mtxDepthPoints.cols != nWidth || mtxDepthPoints.rows != nHeight)
	{
		mtxDepthPoints.release();
		mtxDepthPoints.create(cv::Size(nWidth, nHeight), CV_32FC3);
	}
	for (int i = 0; i < mtxDepthPoints.rows; i++)
	{
		for (int j = 0; j < mtxDepthPoints.cols; j++)
		{
			float x, y, z;
			fread(&x, sizeof(float), 1, pFile);
			fread(&y, sizeof(float), 1, pFile);
			fread(&z, sizeof(float), 1, pFile);
			mtxDepthPoints.at<cv::Vec3f>(i, j) = cv::Vec3f(x, y, z);
		}
	}
	fclose(pFile);
	return true;
}

bool LoadLabelImage(cv::Mat &mtxLabels, const char *strName)
{
	FILE *pFile = fopen(strName, "rb");
	if (!pFile)
		return false;
	int nWidth, nHeight;
	fread(&nWidth, sizeof(int), 1, pFile);
	fread(&nHeight, sizeof(int), 1, pFile);
	if (mtxLabels.cols != nWidth || mtxLabels.rows != nHeight)
	{
		mtxLabels.release();
		mtxLabels.create(cv::Size(nWidth, nHeight), CV_32SC1);
	}
	for (int i = 0; i < nHeight; i++)
	{
		for (int j = 0; j < nWidth; j++)
		{
			unsigned char cLabel;
			fread(&cLabel, sizeof(unsigned char), 1, pFile);
			mtxLabels.at<int>(i, j) = cLabel;
		}
	}
	fclose(pFile);
	return true;
}

bool LoadHandParams(HandParams &param, const char *strName)
{
	FILE *pFile = fopen(strName, "r");
	if (!pFile)
		return false;

	// read the hand region
	cv::Rect rtOBB;
	fscanf(pFile, "%d, %d, %d, %d\n", &rtOBB.x, &rtOBB.y, &rtOBB.width, &rtOBB.height);
	param.OBB = rtOBB;

	// read the pose parameters
	int nDofNum;
	fscanf(pFile, "%d\n", &nDofNum);
	std::vector<DoF> vecDofSet;
	for (int i = 0; i < nDofNum; i++)
	{
		char strDofName[255];
		int nType;
		double fValue;
		fscanf(pFile, "%s	%d	%lf\n", strDofName, &nType, &fValue);
		vecDofSet.push_back(DoF(strDofName, (DofType)nType, fValue));
	}
	param.DofSet = vecDofSet;

	// read the projected palm center and the fingertips
	int m, n;
	double x, y, z;
	fscanf(pFile, "%d, %d, %lf, %lf, %lf\n", &m, &n, &x, &y, &z);
	param.PalmCenter2D = cv::Point(m, n);
	param.PalmCenter3D = cv::Vec3f(x, y, z);
	for (int i = 0; i < 5; i++)
	{
		fscanf(pFile, "%d, %d, %lf, %lf, %lf\n", &m, &n, &x, &y, &z);
		param.Fingertips2D[i] = cv::Point(m, n);
		param.Fingertips3D[i] = cv::Vec3f(x, y, z);
	}

	// read the projected joint locations
	for (int i = 0; i < 29; i++)
	{
		fscanf(pFile, "%d, %d, %lf, %lf, %lf\n", &m, &n, &x, &y, &z);
		param.JointLocs2D[BoneNames[i]] = cv::Point(m, n);
		param.JointLocs3D[BoneNames[i]] = cv::Vec3f(x, y, z);
	}

	fclose(pFile);
	return true;
}

bool LoadFrame(std::string strDir, int nIndex, cv::Mat &mtxDepthPoints, 
	cv::Mat &mtxLabels, HandParams &param)
{
	bool bLoaded = true;
	char text[255];

	sprintf(text, "%s\\depth3d_%d.rhd", strDir.c_str(), nIndex);
	if (!LoadDepthPoints(mtxDepthPoints, text))
		bLoaded = false;

	sprintf(text, "%s\\label_%d.rhd", strDir.c_str(), nIndex);
	if (!LoadLabelImage(mtxLabels, text))
		bLoaded = false;
	
	sprintf(text, "%s\\para_%d.txt", strDir.c_str(), nIndex);
	if (!LoadHandParams(param, text))
		bLoaded = false;
	return bLoaded;
}
