#pragma once
#include <cv.h>
#include <cxcore.h>
#include "DataSet.h"
#include "MathUtilities.h"

// note the two classes are designed for Sequential Training
// in this class, the hypothesis is a hyperplane parallel to one dimension of the feature space
class NaiveRandomTest
{
public:
	NaiveRandomTest(void) {}
	NaiveRandomTest(int nClassNum);
	NaiveRandomTest(int nClassNum, double fFeatMin, double fFeatMax);

public:
	virtual void reset(void);
	virtual bool eval(const Sample &sample) = 0;
	virtual void update(const Sample &sample) = 0;
	virtual void print(void) = 0;
	std::pair<std::vector<double> , std::vector<double> > getStats(void);
	double score(void);

protected:
public:
	int m_nClassNum;
	double m_fThreshold;
	double m_fFeatMin, m_fFeatMax;
	double m_fTrueCount;
	double m_fFalseCount;
	std::vector<double> m_vecTrueStats;
	std::vector<double> m_vecFalseStats;
};

// in this class, the hypothesis is an arbitrary hyperplane
class HyperPlaneTest: public NaiveRandomTest 
{
public:
	HyperPlaneTest(void) {}
	HyperPlaneTest(int nClassNum, int nFeatureDim, int nProjFeatureNum, 
		const std::vector<std::pair<double, double> > &vecFeatureRanges);

public:
	virtual void update(const Sample &sample);
	virtual bool eval(const Sample &sample);
	virtual bool eval(cv::Point p);		// evaluate the sample point in terms of depth context (for Depth Labeling purpose)
	virtual void print(void);
	void getInfo(double &fThreshold, int &nProjFeatureNum, std::vector<int> &vecProjFeatures,
		std::vector<double> &vecProjWeights);
	void setInfo(double fThreshold, int nProjFeatureNum, std::vector<int> vecProjFeatures,
		std::vector<double> vecProjWeights);

private:
public:
	int m_nProjFeatureNum;
	std::vector<int> m_vecProjFeatures;
	std::vector<double> m_vecProjWeights;
};
