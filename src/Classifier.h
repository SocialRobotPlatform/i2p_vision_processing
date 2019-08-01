#pragma once
#include <vector>
#include <cxcore.h>
#include <cv.h>
#include "DataSet.h"

class Classifier
{
public:
	virtual Result eval(const Sample &sample) = 0;

public:
	virtual Result eval(cv::Point p) {return Result();}								// evaluate a pixel using the depth context	
};
