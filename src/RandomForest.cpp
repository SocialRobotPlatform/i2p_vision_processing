#include "RandomForest.h"
#include "DepthUtilities.h"
#include <omp.h>

RandomForest::RandomForest(void)
{
	m_bUseSoftVoting = false;
	m_nTreeNum = 0;
}

RandomForest::RandomForest(const HyperParams &hp)
{
	m_bUseSoftVoting = hp.UseSoftVoting;
	m_nTreeNum = hp.TreeNum;
	m_gHP = hp;
	for (int i = 0; i < m_nTreeNum; i++)
	{
		RandomDecisionTree *pTree = new RandomDecisionTree(i, hp);
		m_vecTrees.push_back(pTree);
	}
}

RandomForest::~RandomForest() 
{
	for (vector<RandomDecisionTree*>::iterator itrdt = m_vecTrees.begin();
		itrdt != m_vecTrees.end(); itrdt++)
		delete (*itrdt);
}

Result RandomForest::eval(const Sample &sample)
{
	Result result, treeResult;
	for (int i = 0; i < m_nClassNum; i++) 
		result.Confidence.push_back(0.0);

	for (int i = 0; i < m_nTreeNum; i++)
	{
		treeResult = m_vecTrees[i]->eval(sample);
		if (m_bUseSoftVoting) 
			add(treeResult.Confidence, result.Confidence);
		else 
			result.Confidence[treeResult.Prediction]++;
	}

	scale(result.Confidence, 1.0 / m_nTreeNum);
	result.Prediction = argmax(result.Confidence);
	return result;
}

Result RandomForest::eval(cv::Point p)
{
	Result result, treeResult;
	for (int i = 0; i < m_nClassNum; i++) 
		result.Confidence.push_back(0.0);

	for (int i = 0; i < m_nTreeNum; i++)
	{
		treeResult = m_vecTrees[i]->eval(p);
		if (m_bUseSoftVoting) 
			add(treeResult.Confidence, result.Confidence);
		else 
			result.Confidence[treeResult.Prediction]++;
	}

	scale(result.Confidence, 1.0 / m_nTreeNum);
	result.Prediction = argmax(result.Confidence);
	int confcounter = 0;
	for (vector<double>::iterator itc = result.Confidence.begin();
		itc != result.Confidence.end(); itc++)
	{
		confcounter++;
	}
	return result;
}

void RandomForest::readForest(std::string strFileName)
{
	FILE *pFile = fopen(strFileName.c_str(), "r");
	int nUseSoftVoting;
	fscanf(pFile, "%d	%d %d	%d	%d	%d	%d	%d\n", &m_nClassNum, &m_nFeatureDim, &m_gHP.TreeNum, 
		&nUseSoftVoting, &m_gHP.MaxDepth, &m_gHP.MinCounter, &m_gHP.ProjFeatureNum, &m_gHP.RandomTestNum);
	m_bUseSoftVoting = (nUseSoftVoting == 0) ? true : false;
	m_gHP.UseSoftVoting = m_bUseSoftVoting;
	m_nTreeNum = m_gHP.TreeNum;
	for (int i = 0; i < m_nTreeNum; i++)
	{
		RandomDecisionTree *pTree = new RandomDecisionTree(i, m_gHP);
		pTree->readTree(pFile, m_nClassNum, m_nFeatureDim);
		m_vecTrees.push_back(pTree);
	}
	fclose(pFile);
}