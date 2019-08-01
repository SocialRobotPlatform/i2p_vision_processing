#pragma once
#define _WINSOCKAPI_ 
#include "IisuKinectReader.h"
#include "HandParser.h"

class HandProcessor
{
public:
	HandProcessor(void) {}

public:
	void	Init(const std::string& confFile);
	bool	Start(void);
	bool	Update(void);
	void	Stop(void);

private:	
	cv::Size					m_sFrame;	
	IisuKinectReader			m_gReader;
	HandParser					m_gParser;
};