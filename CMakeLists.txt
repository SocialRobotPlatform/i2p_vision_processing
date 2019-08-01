#####################################################################
# i2p_applications cmake
#####################################################################

cmake_minimum_required(VERSION 2.6)

message(STATUS "The CMAKE Generator Version is " ${CMAKE_GENERATOR})

set(PROJECT_NAME "i2p_vision_processing")
project(${PROJECT_NAME})

#Set as Unicode Project
add_definitions(-DUNICODE -D_UNICODE)
	
#setup where are the modules: for findKinectSDK.cmake
set(CMAKE_MODULE_PATH $ENV{I2P_MODULE}/i2pCmake)

if (WIN32)
    #set(BOOST_ROOT $ENV{IMI_LIBRARIES})
	set(BOOST_ROOT "$ENV{IMI_LIBRARIES}/boost_1_56_0")
endif()

SET(OpenCV_DIR "$ENV{IMI_LIBRARIES}/opencv/build")

message(STATUS "OpenCV_DIR " ${OpenCV_DIR})

include($ENV{I2P_MODULE}/i2pCmake/i2p.cmake)

FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(KinectSDK REQUIRED)

#####################################################################
# Thrift
#####################################################################
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Thrift REQUIRED)
IF( NOT THRIFT_FOUND )
	message("Failed to find thrift!")
ELSE()
	include_directories(${THRIFT_INCLUDE_DIR})
ENDIF()

#####################################################################
# Generate Thrift stubs
#####################################################################
include_directories($ENV{I2P_MODULE}/i2pThrift/gen-cpp)
GENERATE_STUBS(I2P.thrift cpp)
GENERATE_STUBS(Control.thrift cpp)
GENERATE_STUBS(Inputs.thrift cpp)
GENERATE_STUBS(SocialNetwork.thrift cpp)
GENERATE_STUBS(Devices.thrift cpp)
GENERATE_STUBS(Feedback.thrift cpp )

#GENERATE_STUBS(I2P.thrift py)
#GENERATE_STUBS(Control.thrift py)
#GENERATE_STUBS(Inputs.thrift py)
#GENERATE_STUBS(SocialNetwork.thrift py)
#GENERATE_STUBS(Devices.thrift py)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${KinectSDK_INCLUDE_DIR})
INCLUDE_DIRECTORIES($ENV{I2P_MODULE}/i2pThrift/tools/cpp)
INCLUDE_DIRECTORIES($ENV{I2P_MODULE}/i2pThrift/gen-cpp)
INCLUDE_DIRECTORIES(${PROJECT_NAME})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
LINK_DIRECTORIES(${KinectSDK_LIBRARY_DIRS1})
LINK_DIRECTORIES(${KinectSDK_LIBRARY_DIRS2})
LINK_DIRECTORIES(${KinectSDK_LIBRARY_DIRS3})
LINK_DIRECTORIES(${KinectSDK_LIBRARY_DIRS4})
LINK_DIRECTORIES(${KinectSDK_LIBRARY_DIRS5})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})


file(TO_CMAKE_PATH "$ENV{I2P_MODULE}/i2pThrift" I2P_MODULE_PATH)

set( I2P_SRCS
    ${I2P_MODULE_PATH}/gen-cpp/I2P_constants.cpp
    ${I2P_MODULE_PATH}/gen-cpp/I2P_constants.h
    ${I2P_MODULE_PATH}/gen-cpp/I2P_types.cpp
    ${I2P_MODULE_PATH}/gen-cpp/I2P_types.h
    ${I2P_MODULE_PATH}/gen-cpp/Inputs_constants.cpp
    ${I2P_MODULE_PATH}/gen-cpp/Inputs_constants.h
    ${I2P_MODULE_PATH}/gen-cpp/Inputs_types.cpp
    ${I2P_MODULE_PATH}/gen-cpp/Inputs_types.h
    ${I2P_MODULE_PATH}/gen-cpp/UserTrackingServiceNewKinect.cpp
    ${I2P_MODULE_PATH}/gen-cpp/UserTrackingServiceNewKinect.h
	${I2P_MODULE_PATH}/gen-cpp/SoundService.cpp
	${I2P_MODULE_PATH}/gen-cpp/SoundService.h
	${I2P_MODULE_PATH}/gen-cpp/Facebook.h
	${I2P_MODULE_PATH}/gen-cpp/Facebook.cpp
	${I2P_MODULE_PATH}/gen-cpp/SocialNetwork_constants.h
	${I2P_MODULE_PATH}/gen-cpp/SocialNetwork_constants.cpp
    ${I2P_MODULE_PATH}/tools/cpp/ThriftTools.cpp
    ${I2P_MODULE_PATH}/tools/cpp/ThriftTools.hpp
    ${I2P_MODULE_PATH}/tools/cpp/ProtectedClient.h #this is a tool

)

set(BodyBasics-D2D_SRC
	src/BodyBasics.aps
	src/BodyBasics.cpp
	src/BodyBasics.h
	src/BodyBasics.rc
	src/resource.h
	src/stdafx.h
	src/app.ico
	src/FaceBasics.aps
	src/FaceBasics.cpp
	src/FaceBasics.h
	src/ImageRenderer.h
	src/ImageRenderer.cpp
	src/Source.cpp
	src/GestureBasics.cpp
	src/GestureBasics.h
	src/app1.ico
	src/AudioBasics-IStream.aps
	src/AudioBasics-IStream.h
	src/AudioBasics-IStream.cpp
	src/AudioPanel.h
	src/AudioPanel.cpp
	src/app3.ico
	src/FaceDlg.h
	src/FaceDlg.cpp
	src/Face_Recognition.h
	src/Face_Recognition.cpp
	src/AboutDlg.cpp
	src/AboutDlg.h
	src/app5.ico
	src/Classifier.h
	src/DataSet.h
	src/DataSet.cpp
	src/DepthUtilities.h
	src/DepthUtilities.cpp
	src/Global.h
	src/HyperParams.h
	src/HyperParams.cpp
	src/MathUtilities.h
	src/MathUtilities.cpp
	src/RandomDecisionTree.h
	src/RandomDecisionTree.cpp
	src/RandomForest.h
	src/RandomForest.cpp
	src/RandomTest.h
	src/RandomTest.cpp
	src/RDTNode.h
	src/RDTNode.cpp
	src/FileIO.h
	src/FileIO.cpp
	src/HandParser.h
	src/HandParser.cpp
	src/HandProcessor.h
	src/HandProcessor.cpp
	src/IisuKinectReader.h
	src/IisuKinectReader.cpp
	src/IisuReader.h
	src/IisuReader.cpp
	src/ImageProcessor.h
	src/ImageProcessor.cpp
	src/Test.cpp
	src/Facebook_server.skeleton.cpp
	

	)
	
SOURCE_GROUP( i2p FILES ${I2P_SRCS})

#####################################################################
# Common Tools from I2P
set( COMMON_HDRS
    ${I2P_MODULE_PATH}/tools/cpp/ThriftTools.hpp
)
set( COMMON_SRCS
    ${I2P_MODULE_PATH}/tools/cpp/ThriftTools.cpp
)

if(WIN32)
    set( COMMON_SRCS ${COMMON_SRCS} 
        ${I2P_MODULE_PATH}/tools/cpp/ThriftTools_Win32.cpp ) 
else(WIN32)
    set( COMMON_SRCS ${COMMON_SRCS} 
        ${I2P_MODULE_PATH}/tools/cpp/ThriftTools_Unix.cpp ) 
endif(WIN32)

SOURCE_GROUP( I2PCommonSources FILES ${COMMON_SRCS})
set(HDRS ${HDRS} ${COMMON_HDRS})
set(SRCS ${SRCS} ${COMMON_SRCS})

ADD_EXECUTABLE(${PROJECT_NAME}
    ${I2P_SRCS}
	${HDRS} ${SRCS} 
	${HDR} 
	${BodyBasics-D2D_SRC}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
#libthrift
debug ${THRIFT_LIBRARY_DEBUG}
optimized ${THRIFT_LIBRARY_RELEASE}
${KinectSDK_LIBRARY1}
${KinectSDK_LIBRARY2}
${KinectSDK_LIBRARY3}
${KinectSDK_LIBRARY4}
${KinectSDK_LIBRARY5}
${OpenCV_LIBS}
)

message(STATUS "OpenCV_LIBS " ${OpenCV_LIBS})

if(WIN32)
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:WINDOWS")
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
endif(WIN32)