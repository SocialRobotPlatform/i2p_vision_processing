#include <iostream>
#include <sstream>
#include <algorithm>
#include "HyperParams.h"
using namespace std;

HyperParams::HyperParams(const string& confFile)
{
	fstream fsConfig(confFile);
	
	cout << "MaxDepth: " << (MaxDepth = ReadOneInt(fsConfig)) << endl;
	cout << "RandomTestNum: " << (RandomTestNum = ReadOneInt(fsConfig)) << endl;
	cout << "ProjFeatureNum: " << (ProjFeatureNum = ReadOneInt(fsConfig)) << endl;
	cout << "MinCounter: " << (MinCounter = ReadOneInt(fsConfig)) << endl;
	cout << "TreeNum: " << (TreeNum = ReadOneInt(fsConfig)) << endl;

	int flag;
	if ((flag = ReadOneInt(fsConfig)) == 1)
		UseSoftVoting = true;
	else
		UseSoftVoting = false;
	cout << "UseSoftVoting: " << UseSoftVoting << endl;
}

int HyperParams::ReadOneInt(fstream &fsConfig)
{
	string line;
	getline(fsConfig, line);
	int nParam;
	stringstream(line) >> nParam;
	return nParam;
}