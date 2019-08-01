#include <iostream>
#include <map>
#include "Global.h"
#include "RDTNode.h"
using namespace std;

RDTNode::RDTNode(void) : MAX_DEPTH(0), MIN_COUNTER(0.0)
{
	m_bRoot = true;
	m_pLeftChildNode = m_pRightChildNode = NULL;
	m_nNodeLabel = -1;
}

RDTNode::RDTNode(int nDepth, int nMaxDepth, int nClassNum, int nFeatureDim, double fMinCounter) : m_nDepth(nDepth),
	MAX_DEPTH(nMaxDepth), m_nClassNum(nClassNum), m_nFeatureDim(nFeatureDim), m_bLeaf(true), 
	m_fCounter(0.0), m_nLabel(-1), MIN_COUNTER(fMinCounter)
{
	if (m_nDepth == 0)
		m_bRoot = true;
	else
		m_bRoot = false;
	for (int i = 0; i < m_nClassNum; i++)
		m_vecLabelStats.push_back(0.0);
	m_pLeftChildNode = m_pRightChildNode = NULL;
}

RDTNode::RDTNode(int nDepth, int nMaxDepth, int nClassNum, int nFeatureDim, double fMinCounter,
	const std::vector<double> &vecLabelStats) :  m_nDepth(nDepth), MAX_DEPTH(nMaxDepth), 
	m_nClassNum(nClassNum), m_nFeatureDim(nFeatureDim), m_bLeaf(true), 
	m_fCounter(0.0), m_nLabel(-1), MIN_COUNTER(fMinCounter)
{
	if (m_nDepth == 0)
		m_bRoot = true;
	else
		m_bRoot = false;
	m_fCounter = sum(vecLabelStats);
	m_vecLabelStats = vecLabelStats;
	m_nLabel = argmax(m_vecLabelStats);
	m_pLeftChildNode = m_pRightChildNode = NULL;
}

RDTNode::~RDTNode()
{
	if (!m_bLeaf)
	{
		delete m_pLeftChildNode;
		delete m_pRightChildNode;
		m_pLeftChildNode = m_pRightChildNode = NULL;
	}
}

Result RDTNode::eval(const Sample &sample) 
{
	if (m_bLeaf) 
	{
		Result result;
		if (m_fCounter != 0.0)
		{
			result.Confidence = m_vecLabelStats;
			scale(result.Confidence, 1.0 / m_fCounter);
			result.Prediction = m_nLabel;
		}
		else
		{
			for (int i = 0; i < m_nClassNum; i++)
			{
				result.Confidence.push_back(1.0 / m_nClassNum);
			}
			result.Prediction = -1;
		}
		return result;
	}
	else 
	{
		if (m_tOpt.eval(sample))
			return m_pRightChildNode->eval(sample);
		else 
			return m_pLeftChildNode->eval(sample);
	}
}

// evaluate using the depth context
Result RDTNode::eval(cv::Point p) 
{
	if (m_bLeaf) 
	{
		Result result;
		if (m_fCounter != 0.0)
		{
			result.Confidence = m_vecLabelStats;
			scale(result.Confidence, 1.0 / m_fCounter);
			result.Prediction = m_nLabel;
		}
		else
		{
			for (int i = 0; i < m_nClassNum; i++)
			{
				result.Confidence.push_back(1.0 / m_nClassNum);
			}
			result.Prediction = 0;
		}
		return result;
	}
	else 
	{
		bool bFlag = m_tOpt.eval(p);
		if (g_bDebug)
			cout << (bFlag ? 1 : 0) << ", ";
		if (bFlag)
			return m_pRightChildNode->eval(p);
		else 
			return m_pLeftChildNode->eval(p);
	}
}

void RDTNode::readNode(FILE *pFile, vector<RDTNode*> &vecNodes)
{
	int nLeaf;
	fscanf(pFile, "%d	", &nLeaf);
	if (nLeaf == 1)
	{	
		m_bLeaf = false;
		fscanf(pFile, "%d	", &m_nLabel);

		double fThreshold;
		int nProjNum;
		std::vector<int> vecProjFeatures;
		std::vector<double> vecProjWeights;
		fscanf(pFile, "%lf	%d	", &fThreshold, &nProjNum);
		for (int i = 0; i < nProjNum; i++)
		{
			int nProjFeature;
			double fWeight;
			fscanf(pFile, "%d	%lf	", &nProjFeature, &fWeight);
			vecProjFeatures.push_back(nProjFeature);
			vecProjWeights.push_back(fWeight);
		}
		m_tOpt.setInfo(fThreshold, nProjNum, vecProjFeatures, vecProjWeights);

		m_pRightChildNode = new RDTNode(m_nDepth + 1, MAX_DEPTH, m_nClassNum, m_nFeatureDim, MIN_COUNTER);
		m_pLeftChildNode = new RDTNode(m_nDepth + 1, MAX_DEPTH, m_nClassNum, m_nFeatureDim, MIN_COUNTER);
		vecNodes.push_back(m_pRightChildNode);
		vecNodes.push_back(m_pLeftChildNode);
		m_pLeftChildNode->readNode(pFile, vecNodes);
		m_pRightChildNode->readNode(pFile, vecNodes);
	}
	else
	{
		m_bLeaf = true;
		fscanf(pFile, "%d	", &m_nLabel);
		vector<double> vecLabelStats;
		for (int i = 0; i < m_nClassNum; i++)
		{
			double fStat;
			fscanf(pFile, "%lf	", &fStat);
			vecLabelStats.push_back(fStat);
		}
		m_vecLabelStats = vecLabelStats;
		m_fCounter = sum(m_vecLabelStats);
		m_pLeftChildNode = m_pRightChildNode = NULL;
	}
}