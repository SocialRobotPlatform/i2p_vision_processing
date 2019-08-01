#pragma once
#include <vector>
#include <string>

// the training sample class
class Sample
{
public:
	void print(void);

public:	
	std::vector<float> Feature;	// feature vector
	int Label;					// label
	double Weight;				// weight
};

class DataSet 
{
public:
	std::vector<Sample> Samples;
	int SampleNum;
	int FeatureDim;
	int ClassNum;
	std::vector<std::pair<double, double> > FeatureRanges;

public:
	void findFeatRange(void);
	void loadDataset(std::string strFileName);
	void loadMultipleDatasets(std::vector<std::string> vecFileNames);
	void generateSubsets(std::vector<DataSet> &vecSubsets, int size, int N) const;
	void generateSubset(DataSet &dsSub, int size) const;
};

class Result
{
public:
	std::vector<double> Confidence;
	int Prediction;
};
