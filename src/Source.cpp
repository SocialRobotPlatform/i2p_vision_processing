


#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include "FaceBasics.h"
#include "AboutDlg.h"
#include "AudioBasics-IStream.h"
#include "Face_Recognition.h"
#include "FaceDlg.h"
#include <windows.h>
#include <string>
#include <iostream>
#include <boost/thread.hpp> 
#include "HandProcessor.h"




using namespace std;

//Global//

//IP Address//
string IP = "localhost";
//string IP = "155.69.52.58";	//Razer laptop
//string IP = "155.69.54.79";  //Aryel PC

bool flag_skel = false; bool flag_ges = false;
float SkeletonDistance = 4.0;

HINSTANCE hInst;


//using boost::shared_ptr;

void Body()
{
	
	CBodyBasics application1;
	application1.Run(hInst, SW_HIDE);
}

void Face()
{
	flag_skel = true; flag_ges = true;
	CFaceBasics application2;
	application2.Run(hInst, SW_SHOWDEFAULT);
}

void Audio()
{
	HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

	if (SUCCEEDED(hr))
	{
		{
			CAudioBasics application3;
			application3.Run(hInst, SW_HIDE);
		}

		CoUninitialize();
	}

}

void Gesture()
{
	//flag_ges = true;
	//CBodyBasics application4;
	//application4.Run(hInst, SW_SHOWDEFAULT);
}

void Face_Recog()
{
	//FaceDlg application5;
	//application5.Run(hInst, SW_HIDE);
}

void About()
{
	AboutDlg application6;
	application6.Run(hInst, SW_SHOW);
}

void Hand_Tracking()
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

/*
void facebook()
{
	int port = 14000;
	boost::shared_ptr<FacebookHandler> handler(new FacebookHandler());
	imi::createServer<FacebookHandler, FacebookProcessor>(handler, port);
}
*/

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	hInst = (HINSTANCE)GetModuleHandle(NULL);

	//Boost//
	boost::thread T1(Body);
	boost::thread T2(Face);
	boost::thread T3(Audio);
	//boost::thread T4(Gesture);
	//boost::thread T5(Face_Recog);
	//boost::thread T6(About);
	//boost::thread T7(Hand_Tracking);


	//T1.join();
	T2.join();
	T3.join();
	//T4.join();
	//T5.join();
	//T6.join();
	//T7.join();


	return 0;
}




