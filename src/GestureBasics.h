
#pragma once


#include <iostream>
#include <string>


#include "ThriftTools.hpp"
#include "UserTrackingServiceNewKinect.h"
#include "ProtectedClient.h"
#include "Inputs_constants.h"

class GestureBasics
{
public:
	GestureBasics(void);
	~GestureBasics(void);

	void ProcessSkeletons();

private:

	
};

