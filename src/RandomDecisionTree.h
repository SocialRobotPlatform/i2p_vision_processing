#pragma once
#include "Classifier.h"
#include "DataSet.h"
#include "HyperParams.h"
#include "RDTNode.h"

extern int g_nDebugIdx;
extern bool g_bDebug;
class RandomDecisionTree: public Classifier
{
public:
	RandomDecisionTree(int nTreeLabel, const HyperParams &hp);
	~RandomDecisionTree();

public:
	virtual Result	eval(const Sample &sample);
	virtual Result	eval(cv::Point p);
	void			readTree(FILE *pFile, int nClassNum, int nFeatureDim);

private:
	int				m_nClassNum;
	int				m_nFeatureDim;
	int				m_nTreeLabel;
	RDTNode*	m_pRootNode;
	HyperParams	m_gHP;
	int				m_nNodeNum;
	std::vector<RDTNode*>	m_vecNodes;
};
