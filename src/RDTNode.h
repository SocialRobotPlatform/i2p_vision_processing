#pragma once
#include <stdio.h>
#include <vector>
#include "DataSet.h"
#include "HyperParams.h"
#include "RandomTest.h"

extern int g_nDebugIdx;
extern bool g_bDebug;
// the RDT node is designed for batch training, not for sequential training
class RDTNode 
{
public:
	RDTNode(void);
	RDTNode(int nDepth, int nMaxDepth, int nClassNum, int nFeatureDim, double fMinCounter);
	RDTNode(int nDepth, int nMaxDepth, int nClassNum, int nFeatureDim, double fMinCounter,
		const std::vector<double> &vecLabelStats);
	~RDTNode();

public:
	Result eval(const Sample &sample);
	Result eval(cv::Point p);			

public:
	void setLabel(int nNodeLabel) { m_nNodeLabel = nNodeLabel; }
	bool isLeaf(void) { return m_bLeaf; }
	bool isRoot(void) {return m_bRoot; }
	void readNode(FILE *pFile, std::vector<RDTNode*> &vecNodes);

private:
	bool m_bLeaf;									// whether the node is a leaf
	bool m_bRoot;									// whether the node is a root
	int m_nDepth;									// the tree level of the current node
	const int MAX_DEPTH;
	int m_nLabel;
	double m_fCounter;
	const double MIN_COUNTER;
	std::vector<double> m_vecLabelStats;
	HyperPlaneTest m_tOpt;
	int m_nNodeLabel;								// the unique label to identify a tree node

private:	// these parameters are used across the whole random forest
	int m_nClassNum;							
	int m_nFeatureDim;					

private:	// the left and right children nodes
	RDTNode* m_pLeftChildNode;
	RDTNode* m_pRightChildNode;

public:
	std::vector<int> m_vecSampleIndices;
};