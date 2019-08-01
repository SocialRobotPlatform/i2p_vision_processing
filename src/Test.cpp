#include "HandProcessor.h"
#include <iostream>
using namespace std;

int main(void)
{			
	HandProcessor gProcessor;
	gProcessor.Init("..\\data\\KinectConfig.txt");
	gProcessor.Start();

	while (true)
	{	
		gProcessor.Update();

		if (cv::waitKey(200) >= 0)
			break;
	//	cv::waitKey(0);
	}
	gProcessor.Stop();
}