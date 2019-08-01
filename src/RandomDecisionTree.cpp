#include <iostream>
#include "RandomDecisionTree.h"
#include "DepthUtilities.h"
#include <omp.h>
using namespace std;

int g_nDebugIdx = 1;
bool g_bDebug;
RandomDecisionTree::RandomDecisionTree(int nTreeLabel, const HyperParams &hp)
{
	m_nTreeLabel = nTreeLabel;
	m_gHP = hp;
	m_pRootNode = NULL;
	m_nNodeNum = 0;
}

RandomDecisionTree::~RandomDecisionTree() 
{
	if (m_pRootNode != NULL)
		delete m_pRootNode;
	m_vecNodes.clear();
	m_nNodeNum = 0;
}

Result RandomDecisionTree::eval(const Sample &sample) 
{
	return m_pRootNode->eval(sample);
}

Result RandomDecisionTree::eval(cv::Point p) 
{
	Result result = m_pRootNode->eval(p);
	int confcounter = 0;
	for (vector<double>::iterator itc = result.Confidence.begin();
		itc != result.Confidence.end(); itc++)
	{
		confcounter++;
	}
	return result;
}

void RandomDecisionTree::readTree(FILE *pFile, int nClassNum, int nFeatureDim)
{
	m_nClassNum = nClassNum;
	m_nFeatureDim = nFeatureDim;
	m_pRootNode = new RDTNode(0, m_gHP.MaxDepth, m_nClassNum, m_nFeatureDim, m_gHP.MinCounter);
	m_vecNodes.push_back(m_pRootNode);
	m_pRootNode->readNode(pFile, m_vecNodes);
}