#pragma once
#include "Classifier.h"
#include "DataSet.h"
#include "HyperParams.h"
#include "RandomDecisionTree.h"

class RandomForest: public Classifier
{
public:
	RandomForest(void);
	RandomForest(const HyperParams &hp);
	~RandomForest();

public:
	virtual Result	eval(const Sample &sample);
	virtual Result	eval(cv::Point p);
	void			readForest(std::string strFileName);

protected:
	int				m_nClassNum;
	int				m_nFeatureDim;
	bool			m_bUseSoftVoting;
	int				m_nTreeNum;
	HyperParams	m_gHP;
	std::vector<RandomDecisionTree*>	m_vecTrees;
};
