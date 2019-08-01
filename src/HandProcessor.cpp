#include "HandProcessor.h"
#include <sys/timeb.h>
#include <sstream>
#include <fstream>
using namespace std;

void HandProcessor::Init(const string& confFile)
{
	// parse the configuration file
	fstream fsConfig(confFile);
	string strLine;
	
	cout << "Parsing the configuration file" << " ... " << endl;
	double fDepthMin, fDepthMax;
	getline(fsConfig, strLine);
	stringstream(strLine) >> m_sFrame.width >> m_sFrame.height;
	cout << "Size: " << m_sFrame.width << ", " << m_sFrame.height << endl;
	getline(fsConfig, strLine);
	stringstream(strLine) >> fDepthMin >> fDepthMax;
	cout << "Depth Range: " << fDepthMin << ", " << fDepthMax << endl;

	// read the camera projection parameters
	double fc[2], cc[2], kc[5], alpha_c;
	getline(fsConfig, strLine);
	stringstream(strLine) >> fc[0] >> fc[1];
	cout << "fc: " << fc[0] << ", " << fc[1] << endl;
	getline(fsConfig, strLine);
	stringstream(strLine) >> cc[0] >> cc[1];
	cout << "cc: " << cc[0] << ", " << cc[1] << endl;
	getline(fsConfig, strLine);
	stringstream(strLine) >> kc[0] >> kc[1] >> kc[2] >> kc[3] >> kc[4];
	cout << "kc: " << kc[0] << ", " << kc[1] << ", " << kc[2] << ", " << kc[3] << ", " << kc[4] << endl ;
	getline(fsConfig, strLine);
	stringstream(strLine) >> alpha_c;
	cout << "alpha_c: " << alpha_c << endl;

	// read the Random Forest parameters	
	int nAnchor;
	double f, fu, fv, fRadius;
	string strForestFile;
	getline(fsConfig, strLine);
	stringstream(strLine) >> nAnchor;
	cout << "Anchor Number: " << nAnchor << endl;
	getline(fsConfig, strLine);
	stringstream(strLine) >> fRadius;
	cout << "Feature Radius: " << fRadius << endl;
	getline(fsConfig, strLine);
	stringstream(strLine) >> f >> fu >> fv;
	cout << "RF Camera Params: " << f << ", " << fu << ", " << fv << endl;
	getline(fsConfig, strLine);
	int nLen = 1 + min(strLine.find_last_not_of('\t', strLine.find("//") - 1), strLine.find_last_not_of('\0x20', strLine.find("//") - 1));
	strForestFile = strLine.substr(0, nLen);
	cout << "RF File Name: " << strForestFile << endl;

	// set the parameters for the members
	m_gReader.SetSize(m_sFrame);
	m_gReader.SetProjParams(fc, cc, kc, alpha_c);
	m_gReader.SetDepthRange(fDepthMin, fDepthMax);
	
	m_gParser.SetRFParams(f, fu, fv, nAnchor, fRadius, m_sFrame, strForestFile);
}

bool HandProcessor::Start(void)
{
	return m_gReader.Open();
}

void HandProcessor::Stop(void)
{
	m_gReader.Close();
}

bool HandProcessor::Update(void)
{
	if (m_gReader.Update())
	{
		cv::Mat mtxDepthImg, mtxMask;
		bool bActive[2];
		m_gReader.GetHandInfo(mtxDepthImg, mtxMask, bActive);

		int nGestures[2];
		m_gParser.Update(mtxDepthImg, mtxMask, bActive, nGestures);

		return true;
	}
	else
		return false;
}