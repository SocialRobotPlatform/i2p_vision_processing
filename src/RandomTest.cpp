#include "RandomTest.h"
#include "DepthUtilities.h"
#include <iomanip>
using namespace std;

NaiveRandomTest::NaiveRandomTest(int nClassNum) : m_nClassNum(nClassNum), 
	m_fTrueCount(0.0), m_fFalseCount(0.0)
{
	for (int i = 0; i < m_nClassNum; i++)
	{
		m_vecTrueStats.push_back(0.0);
		m_vecFalseStats.push_back(0.0);
	}
	m_fThreshold = randomFromRange(-1, 1);
	m_fFeatMin = -1;
	m_fFeatMax = 1;
}

NaiveRandomTest::NaiveRandomTest(int m_nClassNum, double fFeatMin, double fFeatMax)
	: m_nClassNum(m_nClassNum), m_fTrueCount(0.0), m_fFalseCount(0.0) 
{
	for (int i = 0; i < m_nClassNum; i++)
	{
		m_vecTrueStats.push_back(0.0);
		m_vecFalseStats.push_back(0.0);
	}
	m_fThreshold = randomFromRange(fFeatMin, fFeatMax);
	m_fFeatMin = fFeatMin;
	m_fFeatMax = fFeatMax;
}

void NaiveRandomTest::reset(void)
{
	m_fTrueCount = m_fFalseCount = 0;
	for (vector<double>::iterator itts = m_vecTrueStats.begin();
		itts != m_vecTrueStats.end(); itts++)
		*itts = 0.0;
	for (vector<double>::iterator itfs = m_vecFalseStats.begin();
		itfs != m_vecFalseStats.end(); itfs++)
		*itfs = 0.0;
}

pair<vector<double> , vector<double> > NaiveRandomTest::getStats()
{
	return pair<vector<double> , vector<double> > (m_vecTrueStats, m_vecFalseStats);
}

double NaiveRandomTest::score(void)
{
	double totalCount = m_fTrueCount + m_fFalseCount;

	// split entropy
	double splitEntropy = 0.0;
	if (m_fTrueCount)
	{
		double p = m_fTrueCount / totalCount;
		splitEntropy -= p * log2(p);
	}

	// entropy before partition
	double priorEntropy = 0.0;
	for (int i = 0; i < m_nClassNum; i++) 
	{
		double p = (m_vecTrueStats[i] + m_vecFalseStats[i]) / totalCount;
		if (p != 0.0)
		{
			priorEntropy -= p * log2(p);
		}
	}

	// entropy after partition
	double trueScore = 0.0, falseScore = 0.0;
	for (int i = 0; i < m_nClassNum; i++)
	{
		// entropy of the samples with true index
		if (m_fTrueCount != 0.0)
		{
			double p = m_vecTrueStats[i] / m_fTrueCount;
			if (p != 0.0)
				trueScore -= p * log2(p);
		}
		// entropy of the samples with false index
		if (m_fFalseCount != 0.0)
		{
			double p = m_vecFalseStats[i] / m_fFalseCount;
			if (p != 0.0)
				falseScore -= p * log2(p);
		}
	}
	double posteriorEntropy = (m_fTrueCount * trueScore + m_fFalseCount * falseScore) / totalCount;

	// information gain
	//	return (2.0 * (priorEntropy - posteriorEntropy)) / (priorEntropy * splitEntropy + 1e-10);
	return (priorEntropy - posteriorEntropy);
}

HyperPlaneTest::HyperPlaneTest(int nClassNum, int nFeatureDim, int nProjFeatureNum, 
	const std::vector<std::pair<double, double> > &vecFeatureRanges)
	: NaiveRandomTest(nClassNum), m_nProjFeatureNum(nProjFeatureNum)
{
	// generate a hyperplane random test, decision of a sample can be made based on 
	// its relative position to the hyperplane in the feature space
	randPerm(nFeatureDim, nProjFeatureNum, m_vecProjFeatures);
//	fillWithRandomNumbers(nProjFeatureNum, m_vecProjWeights);
	m_vecProjWeights.resize(nProjFeatureNum);

	// find the min and max range of the projection
	double minRange = 0.0, maxRange = 0.0;
	for (int i = 0; i < nProjFeatureNum; i++)
	{
		m_vecProjWeights[i] = 1.0;
		minRange += vecFeatureRanges[m_vecProjFeatures[i]].first * m_vecProjWeights[i];
		maxRange += vecFeatureRanges[m_vecProjFeatures[i]].second * m_vecProjWeights[i];
	}
	m_fThreshold = randomFromRange(minRange, maxRange);
	m_fFeatMin = minRange;
	m_fFeatMax = maxRange;
}

// the statistics are only related to the label distribution
void HyperPlaneTest::update(const Sample &sample) 
{ 
	if (eval(sample))
	{
		m_fTrueCount += sample.Weight;
		m_vecTrueStats[sample.Label] += sample.Weight;
	} 
	else
	{
		m_fFalseCount += sample.Weight;
		m_vecFalseStats[sample.Label] += sample.Weight;
	}
}

bool HyperPlaneTest::eval(const Sample &sample) 
{
	double proj = 0.0;
	for (int i = 0; i < m_nProjFeatureNum; i++) 
	{
		proj += sample.Feature[m_vecProjFeatures[i]] * m_vecProjWeights[i];
	}
	return (proj > m_fThreshold) ? true : false;
}
extern bool g_bDebug;
bool HyperPlaneTest::eval(cv::Point p)
{
	double proj = 0.0;
	for (int i = 0; i < m_nProjFeatureNum; i++) 
	{
		cv::Point2f u = g_vecFeatureIndices.at(m_vecProjFeatures[i]).u;
		DepthFeature dfTmp(u);
		double fDepthValue = g_mtxDepthImg.at<float>(p.y, p.x);
		double fFeatureValue = CalcFeatureValue(dfTmp, p, fDepthValue, g_f, g_fu, g_fv, g_mtxDepthImg);
		proj += fFeatureValue * m_vecProjWeights[i];
		if (g_bDebug)
			cout << dfTmp.u.x << "	" << dfTmp.u.y << ": " << fFeatureValue << ", " << this->m_fThreshold << endl;
	}
	return (proj > m_fThreshold) ? true : false;
}

void HyperPlaneTest::print(void)
{
	cout << "[";
	for (vector<int>::const_iterator itpf = m_vecProjFeatures.begin();
		itpf != m_vecProjFeatures.end(); itpf++)
		cout << *itpf << "	";
	cout << "]";
	cout << "-	" << setprecision(4) << m_fThreshold;
	cout << ",	Range: " << setprecision(4) << m_fFeatMin << "	" << setprecision(4) << m_fFeatMax << endl;
}

void HyperPlaneTest::getInfo(double &fThreshold, int &nProjFeatureNum,
	std::vector<int> &vecProjFeatures, std::vector<double> &vecProjWeights)
{
	fThreshold = m_fThreshold;
	nProjFeatureNum = m_nProjFeatureNum;
	vecProjFeatures = m_vecProjFeatures;
	vecProjWeights = m_vecProjWeights;
}

void HyperPlaneTest::setInfo(double fThreshold, int nProjFeatureNum,
	std::vector<int> vecProjFeatures, std::vector<double> vecProjWeights)
{
	m_fThreshold = fThreshold;
	m_nProjFeatureNum = nProjFeatureNum;
	m_vecProjFeatures = vecProjFeatures;
	m_vecProjWeights = vecProjWeights;
}