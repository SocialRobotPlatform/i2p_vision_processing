#pragma once
#include <string>
#include <fstream>

class HyperParams
{
public:
	HyperParams(void) {}
	HyperParams(const std::string& confFile);

private:
	int		ReadOneInt(std::fstream &fsConfig);

public:
	int		RandomTestNum;				// number of generated hypothesis to build the random forest
	int		ProjFeatureNum;				// number of features to determine a hyperplane for random test
	int		MinCounter;					// threshold to determine whether to split the tree node
	int		MaxDepth;					// maximum depth of the random decision tree	
	int		TreeNum;					// number of trees in the random forest
	bool	UseSoftVoting;				// flag indicating to use soft voting or not
};
