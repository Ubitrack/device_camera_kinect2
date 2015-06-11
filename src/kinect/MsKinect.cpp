/*if)
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


/**
 * @ingroup driver_components
 * @file
 * Ubitrack Module for Kinect
 *
 * @author  <pankratz@in.tum.de>, <yuta.itoh@in.tum.de>
 */

#include "MsKinect.h"
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

namespace Ubitrack { namespace Drivers {

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.MsKinect20" ) );

void Kinect20Module::startModule()
{
	if ( !m_running )
	{
		m_running = true;
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &Kinect20Module::kinectThread, this ) ) );
	}
}

void Kinect20Module::stopModule()
{
	if ( m_running )
	{
		m_running = false;
		m_bStop = true;
		LOG4CPP_INFO( logger, "Trying to stop Kinect 20 module");
		if ( m_Thread )
		{
			m_Thread->join();
			LOG4CPP_INFO( logger, "Trying to stop Kinect 20 module, thread joined");
		}

		//shut down Kinect
		if (m_sensor)
		{
			m_sensor->Close();
			m_sensor->Release();
		}
	}
}

void Kinect20Module::kinectInit()
{
	HRESULT hr;
	int iSensorCount = 0;

	hr = GetDefaultKinectSensor(&m_sensor);
	if (FAILED(hr)) {
		LOG4CPP_ERROR(logger, "Cannot get default kinect sensor");
		return;
	}

	hr = m_sensor->Open();
	if (FAILED(hr)) {
		LOG4CPP_ERROR(logger, "Cannot open kinect sensor");
		m_sensor->Release();
		return;
	}

	LOG4CPP_INFO(logger, "Kinect open!");
}

boost::shared_ptr< Kinect20ModuleComponent > Kinect20Module::createComponent( const std::string& type, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
		const ComponentKey& key, ModuleClass* pModule )
	{

	if ( type == "Kinect20Image" )
		return boost::shared_ptr< Kinect20ModuleComponent >( new Kinect20ImageComponent( name, subgraph, key, pModule ) );
	// if ( type == "KinectSkeleton" )
	// 	return boost::shared_ptr< Kinect20ModuleComponent >( new KinectSkeletonComponent( name, subgraph, key, pModule ) );
	else if ( type == "Kinect20AbsoluteSkeleton" )
	 	return boost::shared_ptr< Kinect20ModuleComponent >( new Kinect20AbsoluteSkeletonComponent( name, subgraph, key, pModule ) );

	UBITRACK_THROW( "Class " + type + " not supported by Kinect20 module" );
}

// float waitTime = 5.0f;
// float deltaTime = waitTime;
// Measurement::Timestamp hackTime = 0;
// //float signs[3] = { 1.0f, 1.0f, 1.0f};
// bool showData = false;
// int count = 0;

bool Kinect20AbsoluteSkeletonComponent::getSkeletonPoseList(
	std::vector< Math::Pose >& poseList)
{
    IBodyFrame* bodyFrame = NULL;

    HRESULT hr = m_bodyFrameReader->AcquireLatestFrame(&bodyFrame);
    if (FAILED(hr)) {
    	// fail silently: will fail if no new frame is ready..
		return false;
    }

    INT64 nTime = 0;

    hr = bodyFrame->get_RelativeTime(&nTime);
    if (FAILED(hr)) {
		LOG4CPP_INFO(logger, "Failed to get time");
		return false;
    }

    IBody* bodies[BODY_COUNT] = {0};

	//LOG4CPP_INFO(logger, "Getting body data");
    hr = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

    if (FAILED(hr)) {
		//LOG4CPP_INFO(logger, "Failed to get body data");
		return false;
    }

    Vector4 floorClipPlane;
    hr = bodyFrame->get_FloorClipPlane(&floorClipPlane);
    if (FAILED(hr)) {
    	return false;
    }

    bool foundNew = false;

	// LOG4CPP_INFO(logger, "converting to unitrack");
    for (int i = 0; i < _countof(bodies); ++i) {
    	IBody* body = bodies[i];

    	// only process first body for now
    	//if (i != 0) {
    	//	break;
    	//}

    	if (body) {
	    	BOOLEAN isTracked = false;
	    	hr = body->get_IsTracked(&isTracked);

	    	if (SUCCEEDED(hr) && isTracked) {
	    		// LOG4CPP_INFO(logger, "Body " << i << " is tracked");
	    		Joint joints[JointType_Count];
	    		JointOrientation jointOrientations[JointType_Count];

				// LOG4CPP_INFO(logger, "getting joints");
	    		hr = body->GetJoints(_countof(joints), joints);
	    		if (SUCCEEDED(hr)) {
					// LOG4CPP_INFO(logger, "getting joint oprientations");
		    		hr = body->GetJointOrientations(_countof(jointOrientations),
		    			jointOrientations);
	    			if (SUCCEEDED(hr)) {
	    				for (int jointNum = 0; jointNum < JointType_Count;
	    					++jointNum) {
	    					Math::Vector<double, 3U> tmp(
	    						joints[jointNum].Position.X * m_scale,
	    						joints[jointNum].Position.Y * m_scale,
	    						joints[jointNum].Position.Z * m_scale);
	    					Math::Quaternion rotQ(
	    						jointOrientations[jointNum].Orientation.x,
	    						jointOrientations[jointNum].Orientation.y,
	    						jointOrientations[jointNum].Orientation.z,
	    						jointOrientations[jointNum].Orientation.w);
	    					double lenSq = rotQ.x()*rotQ.x() +
	    						rotQ.y()*rotQ.y() + rotQ.z()*rotQ.z() +
	    						rotQ.w()*rotQ.w();
	    					if (lenSq < 0.1) {
	    						rotQ = Math::Quaternion(0.0, -1.0, 0.0, 0.0);
	    					}
	    					Math::Pose pose(rotQ, tmp);
							Math::Pose poseCorrection(
								Math::Quaternion(0, M_PI, 0),
								Math::Vector3d(0,0,0));
							//Math::Pose poseCorrection(
							//	Math::Quaternion(0, 0, 0),
							//	Math::Vector3d(0,0,0));
							// LOG4CPP_INFO(logger, "setting pose");
	    					poseList[jointNum] = poseCorrection * pose;
	    					foundNew = true;

	    					/*
	    					if (joints[jointNum].TrackingState !=
	    						TrackingState_Tracked) {
	    						poseList[jointNum] = Math::Pose();
	    					}
	    					*/

	    					// XXX for demonstration
	    					/*
	    					if (jointNum == JointType_WristRight ||
	    						jointNum == JointType_HandRight ||
	    						jointNum == JointType_WristLeft ||
	    						jointNum == JointType_HandLeft ||
	    						jointNum == JointType_HandTipLeft ||
	    						jointNum == JointType_HandTipRight ||
	    						jointNum == JointType_ThumbLeft ||
	    						jointNum == JointType_ThumbRight
	    						) {
	    						poseList[jointNum] = Math::Pose();
	    					}
							*/
	    				}
	    			}
	    		}
	    	}

	    	// Store floor clipping plane
			Math::Quaternion floorClipQ(floorClipPlane.x,
				floorClipPlane.y, floorClipPlane.z, floorClipPlane.w);
			Math::Vector<double, 3U> nPos(0.0,0.0,0.0);
	    	m_poseList[JointType_Count] = Math::Pose(floorClipQ, nPos);
	    }
    }

    for (int i = 0; i < _countof(bodies); ++i) {
    	if (bodies[i] != NULL) {
    		bodies[i]->Release();
    	}
    }


    if (bodyFrame != NULL) {
    	bodyFrame->Release();
    }

    return foundNew;
}


void Kinect20AbsoluteSkeletonComponent::sendData(Measurement::Timestamp ts)
{
	bool newData = getSkeletonPoseList(m_poseList);
	if (newData) {
		boost::shared_ptr< std::vector< Math::Pose > > pPoseList(new std::vector<Math::Pose>(m_poseList));
		m_skeletonPorts.send(Measurement::PoseList(ts, pPoseList));
	}
}

void Kinect20ImageComponent::sendData(Measurement::Timestamp ts)
{
	boost::shared_ptr< Vision::Image > pImageRAW;
	boost::shared_ptr< Vision::Image > pImage;
	boost::shared_ptr< Vision::Image > pUVMap;

	int bodyID = getKey().m_targetID;
	switch(bodyID){
	case 0:
		pImageRAW = getColorImage();
		if (!pImageRAW) {
			return;
		}
		if (m_undistorter) {
			pImage = m_undistorter->undistort(pImageRAW);
		}
		else {
			pImage = pImageRAW;
		}
		break;
	case 1:
		pImageRAW = getIRImage();
		if (!pImageRAW) {
			return;
		}
		if (m_undistorter) {
			pImage = m_undistorter->undistort(pImageRAW);
		}
		else {
			pImage = pImageRAW;
		}
		break;
	case 2:
		pImageRAW = getDepthImage(false, pUVMap);
		if (!pImageRAW) {
			return;
		}
		pImage = pImageRAW;
		break;
	case 3:
		pImageRAW = getDepthImage(true, pUVMap);
		if (!pImageRAW) {
			return;
		}
		pImage = pImageRAW;
		break;
	}

	LOG4CPP_DEBUG(logger, "send image of type:" << bodyID);

	if (m_imagePortRAW.isConnected()) {
		m_imagePortRAW.send(Measurement::ImageMeasurement(ts, pImageRAW));
	}
	
	//if (m_imagePort.isConnected()){
		m_imagePort.send(Measurement::ImageMeasurement(ts, pImage));
	//}
	
	
	if (m_imagePortGray.isConnected()) {
		LOG4CPP_DEBUG(logger, "create gray image for " << bodyID);
		boost::shared_ptr< Vision::Image > pGrayImage;

		switch (bodyID){
		case 0:
			pGrayImage = pImage->CvtColor(CV_RGB2GRAY, 1);
			break;
		case 1:
			pGrayImage.reset(new Vision::Image(pImage->width, pImage->height, 1, 8, pImage->origin));
			cvConvertImage(pImage.get(), pGrayImage.get());
			break;
		case 2:
			pGrayImage.reset(new Vision::Image(pImage->width, pImage->height, 1, 8, pImage->origin));
			cvConvertImage(pImage.get(), pGrayImage.get());
			break;
		case 3:
		
			pGrayImage = pUVMap;			
			break;

		}	
		
		m_imagePortGray.send(Measurement::ImageMeasurement(ts, pGrayImage));
	}
}

boost::shared_ptr<Vision::Image> Kinect20ImageComponent::getColorImage() {
	HRESULT hr;

	IColorFrame* colorFrame;

	if (m_colorFrameReader == NULL) {
		LOG4CPP_ERROR(logger, "No m_colorFrameReader available");
		return NULL;
	}

	hr = m_colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(hr)) {
		return NULL;
	}

	int width, height;

	IFrameDescription* frameDescription;
	hr = colorFrame->get_FrameDescription(&frameDescription);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Width(&width);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Height(&height);
	if (FAILED(hr)) {
		return NULL;
	}

	if (frameDescription != NULL) {
		frameDescription->Release();
	}

	boost::shared_ptr< Vision::Image > pImage(new Vision::Image(width, height, 3, IPL_DEPTH_8U));
	// hr = colorFrame->CopyConvertedFrameDataToArray(pImage->imageSize, reinterpret_cast<BYTE*>(pImage->imageData), ColorImageFormat_Bgra);


	IplImage* tmp1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 4);
	IplImage* tmp2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	hr = colorFrame->CopyConvertedFrameDataToArray(tmp1->imageSize, reinterpret_cast<BYTE*>(tmp1->imageData), ColorImageFormat_Bgra);
	cvConvertImage(tmp1, tmp2, 0);
	cvFlip(tmp2, *pImage, 1);

	cvReleaseImage(&tmp2);
	cvReleaseImage(&tmp1);

	// Release the frame
	if (colorFrame != NULL) {
		colorFrame->Release();
	}

	return pImage;
}

boost::shared_ptr<Vision::Image> Kinect20ImageComponent::getDepthImage(bool createUVMap, boost::shared_ptr< Vision::Image >& uvMap) {
	HRESULT hr;

	IDepthFrame* depthFrame;
	IFrameDescription* frameDescription;
	int width, height;
    USHORT depthMinReliableDistance = 0;
    USHORT nDepthMaxDistance = 0;

	if (m_depthFrameReader == NULL) {
		LOG4CPP_ERROR(logger, "No m_depthFrameReader available");
		return NULL;
	}

	hr = m_depthFrameReader->AcquireLatestFrame(&depthFrame);

	if (FAILED(hr)) {
		return NULL;
	}

	hr = depthFrame->get_FrameDescription(&frameDescription);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Width(&width);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Height(&height);
	if (FAILED(hr)) {
		return NULL;
	}

	if (frameDescription != NULL) {
		frameDescription->Release();
	}

	hr = depthFrame->get_DepthMinReliableDistance(&depthMinReliableDistance);
	if (FAILED(hr)) {
		return NULL;
	}

	nDepthMaxDistance = USHRT_MAX;
	// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
    //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
	if (FAILED(hr)) {
		return NULL;
	}

	boost::shared_ptr< Vision::Image > pImage(new Vision::Image(width, height, 1, IPL_DEPTH_32F));

	cv::Mat tmpMat(height, width, CV_16UC1);
	
	hr = depthFrame->CopyFrameDataToArray(width*height,
		reinterpret_cast<UINT16*>(tmpMat.data));
	
	cv::Mat tmpMat2(height, width, CV_16UC1);
	cv::flip(tmpMat, tmpMat2, 1);
	
	cv::Mat tmpImage(*pImage, false);

	tmpMat2.convertTo(tmpImage, CV_32FC1);
	/*
	UINT16* p1 = reinterpret_cast<UINT16*>(tmpMat.data);
	float* p2 = reinterpret_cast<float*>(pImage->imageData);
	for (int i = 0; i < width*height; i++){
		p2[i] = (float)p2[i];			
	}*/

	if (createUVMap){
		uvMap.reset(new Vision::Image(width, height, 2, IPL_DEPTH_32F));
		ICoordinateMapper* pMapper;
		m_sensor->get_CoordinateMapper(&pMapper);
		// cast to ColorSpacePoint should work as it is a struct with float x, float y
		pMapper->MapDepthFrameToColorSpace(width*height, reinterpret_cast<UINT16*>(tmpMat.data), width*height, reinterpret_cast<ColorSpacePoint*>(uvMap->imageData));				
	}

	

	if (depthFrame != NULL) {
		depthFrame->Release();
	}

	return pImage;
}

//TODO: Caution!!! Not raw Depth-Image Data, but beautiful
boost::shared_ptr<Vision::Image> Kinect20ImageComponent::getIRImage()
{
	HRESULT hr;

	IInfraredFrame* infraredFrame;
	IFrameDescription* frameDescription;
	int width, height;

	if (m_infraredFrameReader == NULL) {
		LOG4CPP_ERROR(logger, "No m_infraredFrameReader available");
		return NULL;
	}

	hr = m_infraredFrameReader->AcquireLatestFrame(&infraredFrame);
	if (FAILED(hr)) {
		return NULL;
	}

	hr = infraredFrame->get_FrameDescription(&frameDescription);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Width(&width);
	if (FAILED(hr)) {
		return NULL;
	}
	hr = frameDescription->get_Height(&height);
	if (FAILED(hr)) {
		return NULL;
	}

	if (frameDescription != NULL) {
		frameDescription->Release();
	}

	IplImage* tmp1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_16U, 1);
	boost::shared_ptr< Vision::Image > pImage(new Vision::Image(width, height, 1, IPL_DEPTH_16U));

	hr = infraredFrame->CopyFrameDataToArray(width*height,
		reinterpret_cast<UINT16*>(tmp1->imageData));

	cvFlip(tmp1, *pImage, 1);

	cvReleaseImage(&tmp1);

	if (infraredFrame != NULL) {
		infraredFrame->Release();
	}

	return pImage;
}

void Kinect20Module::kinectThread()
{

	//std::vector<HANDLE> events;
	kinectInit();

	ComponentList allComponents = getAllComponents();
	std::vector<HANDLE> waitHandles;

	for (ComponentList::iterator i = allComponents.begin(); i != allComponents.end(); ++i) {
		(*i)->init(m_sensor);
		waitHandles.push_back((*i)->getEvent());
	}

	while(!m_bStop)
	{
		DWORD dwWaitResult = WaitForMultipleObjects(waitHandles.size(),
			&waitHandles[0], FALSE, 400);
		if (dwWaitResult == WAIT_TIMEOUT) {
			LOG4CPP_ERROR(logger, "Timeout waiting for kinect events");
			continue;
		}
		if (dwWaitResult == WAIT_FAILED) {
			LOG4CPP_ERROR(logger, "Kinect wait failed: " << GetLastError());
			continue;
		}
		if (dwWaitResult >= (WAIT_OBJECT_0 + waitHandles.size())) {
			LOG4CPP_ERROR(logger, "Abandoned event? / Strange wait result: "  << dwWaitResult);
			continue;
		}
		ResetEvent(waitHandles[dwWaitResult]);
		Measurement::Timestamp ts = Measurement::now();
		allComponents[dwWaitResult]->sendData(ts);
	}
	LOG4CPP_INFO(logger, "Main Thread done.");
}

void Kinect20Module::addHandle(HANDLE eventhandler)
{
	m_hevents.push_back(eventhandler);
}

std::ostream& operator<<( std::ostream& s, const Kinect20ComponentKey& k )
{
        s << "Kinect20ComponentKey[ " << k.m_targetID << " "
                             << k.m_targetClass << " ]";
        return s;
}

// register module at factory
UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) {
	std::vector< std::string > moduleComponents;
	moduleComponents.push_back( "Kinect20Skeleton" );
	moduleComponents.push_back( "Kinect20Image" );
	moduleComponents.push_back( "Kinect20AbsoluteSkeleton" );


	cf->registerModule< Kinect20Module > ( moduleComponents );
}

} } // namespace Ubitrack::Drivers




// void KinectSkeletonComponent::getSkeletonPoseList(std::vector< Math::Pose >& poseList)
// {
// 	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextEvent, 0))
//     {
// 		NUI_SKELETON_FRAME skeletonFrame = { 0 };

// 		HRESULT hr = m_sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
// 		if (FAILED(hr))
// 		{
// 			LOG4CPP_ERROR(logger, "Failed to get next frame.");
// 			return;
// 		}

// 		// smooth out the skeleton data
// 		m_sensor->NuiTransformSmooth(&skeletonFrame, NULL);
// 		for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
// 		{
// 			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

// 			if (NUI_SKELETON_TRACKED == trackingState)
// 			{
// 				//For bone orientation
// 				const NUI_SKELETON_DATA & skeleton = skeletonFrame.SkeletonData[i];
// 				NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
// 				NuiSkeletonCalculateBoneOrientations(&skeleton, &boneOrientations[0]);

// 				// We're tracking the skeleton, show coordinates
// 				for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
// 				{
// 					//bone position
// 					Math::Vector<double,3U> tmp(	skeleton.SkeletonPositions[j].x * m_scale,
// 													skeleton.SkeletonPositions[j].y * m_scale,
// 													skeleton.SkeletonPositions[j].z * m_scale);

// 					//bone orientation
// 					NUI_SKELETON_BONE_ORIENTATION & orientation = boneOrientations[j];

// 					Math::Quaternion rotQ(	orientation.absoluteRotation.rotationQuaternion.x,
// 											orientation.absoluteRotation.rotationQuaternion.y,
// 											orientation.absoluteRotation.rotationQuaternion.z,
// 											orientation.absoluteRotation.rotationQuaternion.w );

// 					//einfache rotierung
// 					Math::Pose pose(Math::Quaternion(0, M_PI, 0), Math::Vector3d(0,0,0));
// 					Math::Pose pose2(rotQ, tmp);

// 					poseList[j] = pose * pose2;
// 				}
// 			}
// 		}
// 	}
// }

// void KinectSkeletonComponent::sendData(Measurement::Timestamp ts)
// {
// 	getSkeletonPoseList(m_poseList);

// 	makeHierarchicalPose(m_poseListHierarchical, m_poseList);

// 	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
// 	{
// 		boost::shared_ptr< Math::Pose > pPose(new Math::Pose(m_poseListHierarchical[i]));
// 		m_skeletonPorts[i]->send( Measurement::Pose( ts, pPose ) );
// 	}
// }

//void KinectSkeletonComponent::makeHierarchicalPose(std::vector< Math::Pose >& poseH , std::vector< Math::Pose >& pose)
//{
//	//skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER] = skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER];
//	poseH[0] = pose[0];
//	// Upper Body
//	/*Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_SPINE], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SPINE], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SPINE]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HEAD], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HEAD], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);*/
//	poseH[1] = ~pose[0] * pose[1];
//	poseH[2] = ~pose[1] * pose[2];
//	poseH[3] = ~pose[2] * pose[3];
//
//	//Upper Body Left Part
//	/*Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT]);*/
//	poseH[4] = ~pose[2] * pose[4];
//	poseH[5] = ~pose[4] * pose[5];
//	poseH[6] = ~pose[5] * pose[6];
//	poseH[7] = ~pose[6] * pose[7];
//
//	//Upper Body Right Part
//	/*Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT]);*/
//	poseH[8] = ~pose[2] * pose[8];
//	poseH[9] = ~pose[8] * pose[9];
//	poseH[10] = ~pose[9] * pose[10];
//	poseH[11] = ~pose[10] * pose[11];
//
//	//LowerBody Left Part
//	/*Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT]);*/
//	poseH[12] = ~pose[0] * pose[12];
//	poseH[13] = ~pose[12] * pose[13];
//	poseH[14] = ~pose[13] * pose[14];
//	poseH[15] = ~pose[14] * pose[15];
//
//	//LowerBody Right Part
//	/*Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT]);
//	Vec4Sub(skeletonH.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT], skeleton.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT]);*/
//	poseH[16] = ~pose[0] * pose[16];
//	poseH[17] = ~pose[16] * pose[17];
//	poseH[18] = ~pose[17] * pose[18];
//	poseH[19] = ~pose[18] * pose[19];
//}
