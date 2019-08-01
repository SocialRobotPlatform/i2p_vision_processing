#include "Global.h"
#include "ImageProcessor.h"
using namespace std;

// generate the pseudo color image for the 32F mono image
const unsigned char BLUE_CHANNEL[256] = {0, 3, 6, 9, 12, 15, 19, 22, 25, 28, 31, 35, 38, 41, 44, 47, 51, 54, 57, 60, 63, 66, 70, 73, 76, 79, 82, 86,
	89, 92, 95, 98, 102, 105, 108, 111, 114, 117, 121, 124, 127, 130, 133, 137, 140, 143, 146, 149, 153, 156, 159, 162, 165, 168, 172, 175, 178, 181,
	184, 188, 191, 194, 197, 200, 204, 207, 210, 213, 216, 219, 223, 226, 229, 232, 235, 239, 242, 245, 248, 251, 240, 237, 234, 231, 228, 225, 222, 
	219, 216, 213, 210, 207, 204, 201, 198, 195, 192, 189, 186, 183, 180, 177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 141, 138, 135,
	132, 129, 126, 123, 120, 117, 114, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 69, 66, 63, 60, 57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 
	27, 24, 21, 18, 15, 12, 9, 6, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const unsigned char GREEN_CHANNEL[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39,
	42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 126, 129, 132, 135, 138, 141, 
	144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 189, 192, 195, 198, 201, 204, 207, 210, 213, 216, 219, 222, 225, 228, 231,
	234, 237, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 
	255, 255, 245, 238, 231, 224, 217, 210, 203, 196, 189, 182, 175, 168, 161, 154, 147, 140, 133, 126, 119, 112, 105, 98, 91, 84, 77, 70, 63, 56, 49, 42, 
	35, 28, 21, 14, 7, 0};
const unsigned char RED_CHANNEL[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 255, 255, 255, 255, 255, 255, 255, 
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};

cv::Vec3b MonoToColor(double fValue, double fMin, double fMax, bool bInverse)
{
	if ((fValue <= fMin) || fValue >= fMax)
		return cv::Vec3b(0, 0, 0);
	else
	{
		// check whether it is normal mapping or inverse mapping
		unsigned char cGray;
		if (!bInverse)
			cGray= (unsigned char)((fValue - fMin) / (fMax - fMin) * 255);
		else
			cGray = (unsigned char)((fMax - fValue) / (fMax - fMin) * 255);
		return cv::Vec3b(BLUE_CHANNEL[cGray], GREEN_CHANNEL[cGray], RED_CHANNEL[cGray]);
	}
}

void MonoToColor(const cv::Mat &mtxMonoImg, cv::Mat &mtxColorImg, double fMin, double fMax, bool bInverse)
{	
	for (int i = 0; i < mtxMonoImg.rows; i++)
		for (int j = 0; j < mtxMonoImg.cols; j++)
		{
			double fValue = mtxMonoImg.at<float>(i, j);
			mtxColorImg.at<cv::Vec3b>(i, j) = MonoToColor(fValue, fMin, fMax, bInverse);
		}
}