#include "DataSet.h"
#include "MathUtilities.h"
#include "Global.h"
using namespace std;

void Sample::print(void)
{
	cout << "Sample: label = " << Label << ", weight = " << Weight << ", value = ";
	for (vector<float>::iterator it = Feature.begin(); it != Feature.end(); it++)
		cout << *it << ", ";
	cout << std::endl;
}

// find the range of the feature values for each dimension
void DataSet::findFeatRange(void)
{
	FeatureRanges.resize(FeatureDim);	
	for (int i = 0; i < FeatureDim; i++) 
	{
		double fMin = _DOUBLE_MAX;
		double fMax = _DOUBLE_MIN;
		for (int j = 0; j < SampleNum; j++)
		{
			if (Samples[j].Feature[i] < fMin)
				fMin = Samples[j].Feature[i];
			if (Samples[j].Feature[i] > fMax)
				fMax = Samples[j].Feature[i];
		}
		FeatureRanges.at(i) = pair<double, double>(fMin, fMax);
	}
}

void DataSet::loadDataset(string strFileName) 
{
	FILE *pDataFile = fopen(strFileName.c_str(), "rb");
	if (!pDataFile)
	{
		cout << "Could not open input file " << strFileName << endl;
		exit(EXIT_FAILURE);
	}
	cout << "Loading data file: " << strFileName << " ... " << endl;

	// Reading the header
	fread(&SampleNum, sizeof(int), 1, pDataFile);
	fread(&FeatureDim, sizeof(int), 1, pDataFile);
	fread(&ClassNum, sizeof(int), 1, pDataFile);

	// Reading the training samples
	Samples.clear();
	vector<int> vecLabelCounters(ClassNum, 0);
	for (int i = 0; i < SampleNum; i++) 
	{
		Sample sp;
		sp.Feature.resize(FeatureDim);

		// read the feature values
		for (int j = 0; j < FeatureDim; j++)
		{
			float fValue;
			fread(&fValue, sizeof(float), 1, pDataFile);
			sp.Feature[j] = fValue;
		}
		
		// read the class no. of the current sample point
		int label;
		fread(&label, sizeof(int), 1, pDataFile);
		sp.Label = label;
		vecLabelCounters[label]++;
		sp.Weight = 1.0;
		Samples.push_back(sp);
	}
	fclose(pDataFile);

	// output the number of negative and positive classes
	cout << "Label count: ";
	for (int i = 0; i < ClassNum; i++)
		cout << vecLabelCounters[i] << "	";
	cout << endl;

	// check whether the input is valid
	if (SampleNum != (int) Samples.size())
	{
		cout << "Could not load " << SampleNum << " samples from " << strFileName;
		cout << ". There were only " << Samples.size() << " samples!" << endl;
		exit(EXIT_FAILURE);
	}

	// Find the data range
	findFeatRange();

	cout << "Loaded " << SampleNum << " samples with " << FeatureDim;
	cout << " features and " << ClassNum << " classes." << endl;
}

void DataSet::loadMultipleDatasets(vector<string> vecFileNames)
{
	SampleNum = 0;
	Samples.clear();
	for (std::vector<string>::iterator itfn = vecFileNames.begin();
		itfn != vecFileNames.end(); itfn++)
	{
		FILE *pDataFile = fopen((*itfn).c_str(), "rb");
		if (!pDataFile)
		{
			std::cout << "Could not open input file " << *itfn << endl;
			exit(EXIT_FAILURE);
		}
		std::cout << "Loading data file: " << *itfn << " ... " << endl;

		// Reading the header
		int nTemp, nCurSampleNum;
		fread(&nCurSampleNum, sizeof(int), 1, pDataFile);
		fread(&nTemp, sizeof(int), 1, pDataFile);
		if (itfn == vecFileNames.begin())
			FeatureDim = nTemp;
		else
			assert(FeatureDim == nTemp);
		fread(&nTemp, sizeof(int), 1, pDataFile);
		if (itfn == vecFileNames.begin())
			ClassNum = nTemp;
		else
			assert(ClassNum == nTemp);
		
		// Reading the training samples
		vector<int> vecLabelCounters(ClassNum, 0);
		do
		{
			Sample sp;
			sp.Feature.resize(FeatureDim);

			// read the feature values
			for (int j = 0; j < FeatureDim; j++)
			{
				float fValue;
				if (fread(&fValue, sizeof(float), 1, pDataFile) != 1)
					break;
				sp.Feature[j] = fValue;
			}

			// read the class no. of the current sample point
			int label;
			if (fread(&label, sizeof(int), 1, pDataFile) != 1)
				break;
			sp.Label = label;
			vecLabelCounters[label]++;
			sp.Weight = 1.0;
			Samples.push_back(sp);
		}while (!feof(pDataFile));

		fclose(pDataFile);
		SampleNum += nCurSampleNum;

		// output the number of each class
		cout << "Label count: ";
		for (int i = 0; i < ClassNum; i++)
			cout << vecLabelCounters[i] << "	";
		cout << endl;

		// check whether the input is valid
		if (SampleNum != (int) Samples.size())
		{
			cout << "Could not load " << SampleNum << " samples from " << *itfn;
			cout << ". There were only " << Samples.size() << " samples!" << endl;
			exit(EXIT_FAILURE);
		}
		cout << "Loaded " << nCurSampleNum << " samples with " << FeatureDim;
		cout << " features and " << ClassNum << " classes." << endl;
	}
	
	// Find the data range
	findFeatRange();
}

void DataSet::generateSubsets(std::vector<DataSet> &vecSubsets, int size, int N) const
{
	vecSubsets.clear();
	vecSubsets.resize(N);
	for (int i = 0; i < N; i++)
		generateSubset(vecSubsets.at(i), size);
}

void DataSet::generateSubset(DataSet &dsSub, int size) const
{
	// init the dataset parameters
	dsSub.ClassNum = ClassNum;
	dsSub.FeatureDim = FeatureDim;
	dsSub.SampleNum = size;
	dsSub.Samples.clear();

	// perform resampling on the original dataset
	vector<int> vecRandIndex;
	randPerm(SampleNum, size, vecRandIndex);
	for (vector<int>::iterator itri = vecRandIndex.begin(); 
		itri != vecRandIndex.end(); itri++)
	{
		dsSub.Samples.push_back(Samples.at(*itri));
	}

	// update the feature ranges
	dsSub.findFeatRange();
}