/*
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
 * Ubitrack Module for Leap Motion
 *
 * @author  <pankratz@in.tum.de>, <yuta.itoh@in.tum.de>
 */
#ifndef __MS_KINECT20_H_INCLUDED__
#define __MS_KINECT20_H_INCLUDED__

// Windows Header Files
#include <windows.h>
#include <Shlobj.h>

#include <string>
#include <cstdlib>

#include <Kinect.h>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <utVision/Undistortion.h>

namespace Ubitrack { namespace Drivers {

using namespace Dataflow;

// forward declaration
class Kinect20ModuleComponent;

static log4cpp::Category& logger2( log4cpp::Category::getInstance( "Drivers.MsKinect20" ) );
/**
 * Module key
 */
MAKE_NODEIDKEY( KinectKey, "Kinect20" );

/**
 * Component key
 * Represents what to track
 */
class Kinect20ComponentKey
{
	public:

		/// set priority based on dataflow class as stored in the subgraph by the dataflow network
		Kinect20ComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		{

			Graph::UTQLSubgraph::EdgePtr config;

			  if ( subgraph->hasEdge( "Output" ) )
				  config = subgraph->getEdge( "Output" );

			  if ( !config )
			  {
				  UBITRACK_THROW( "Kinect Pattern has no \"Output\" edge");
			  }

			  config->getAttributeData( "targetID", m_targetID );
			  if ( m_targetID < 0 )
					UBITRACK_THROW( "Missing or invalid \"m_targetID\" attribute on \"Output\" edge" );

			std::string& dfclass = subgraph->m_DataflowClass;

			m_targetClass = 0;
			if ( dfclass == "KinectSkeleton" ) {
				// do something
				m_targetClass = 1;
			};
			if ( dfclass == "KinectImage" ) {
				// do something
				m_targetClass = 2;
			}
			if ( dfclass == "KinectAbsoluteSkeleton" ) {
				// do something
				m_targetClass = 3;
			}
		}

		Kinect20ComponentKey( int target, int tagetclass)
			: m_targetID(target)
			, m_targetClass(tagetclass)
		{

		}


		bool operator<( const Kinect20ComponentKey& b ) const
		{
			if (m_targetClass == b.m_targetClass) return (m_targetID < b.m_targetID);
			return (m_targetClass < b.m_targetClass);
		}

		int m_targetID;
		int m_targetClass;
};


/**
 * Test Module
 * Does all the work
 */
class Kinect20Module
	: public Module< KinectKey, Kinect20ComponentKey, Kinect20Module, Kinect20ModuleComponent >
{
public:
	/** UTQL constructor */
	Kinect20Module( const KinectKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
		: Module< KinectKey, Kinect20ComponentKey, Kinect20Module, Kinect20ModuleComponent >( key, pFactory )
	{



	};

	/** destructor */
	~Kinect20Module()
	{};

	virtual void startModule();

	virtual void stopModule();

	virtual boost::shared_ptr< Kinect20ModuleComponent > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
		const ComponentKey& key, ModuleClass* pModule );

	void kinectThread();

	void kinectInit();

	void addHandle(HANDLE eventhandler);

protected:
    IKinectSensor* m_sensor;

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	std::vector<HANDLE> m_hevents;

private:

};

std::ostream& operator<<( std::ostream& s, const Kinect20ComponentKey& k );

/**
 * First Component for module test
 * Does nothing but provide a push port
 */
class Kinect20ModuleComponent
	: public Kinect20Module::Component
{
public:
	/** constructor */
	Kinect20ModuleComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const Kinect20ComponentKey& componentKey, Kinect20Module* pModule )
		: Kinect20Module::Component( name, componentKey, pModule )
	{}

	/** destructor */
	~Kinect20ModuleComponent(){
	}

	virtual void init(IKinectSensor* sensor){};

	virtual void sendData( Measurement::Timestamp ts)
	{};

	HANDLE getEvent(){
		return reinterpret_cast<HANDLE>(m_hNextEvent);
	};

protected:
	WAITABLE_HANDLE m_hNextEvent;
	IKinectSensor* m_sensor;
};

// class KinectSkeletonComponent
// 	: public Kinect20ModuleComponent
// {
// public:
// 	/** constructor */
// 	KinectSkeletonComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const Kinect20ComponentKey& componentKey, Kinect20Module* pModule )
// 		: Kinect20ModuleComponent( name, subgraph, componentKey, pModule )
// 		, m_hipCenter( "Output", *this )
// 		, m_spine( "SpinePos", *this )
// 		, m_shoulderCenter( "ShoulderCenterPos", *this )
// 		, m_head( "HeadPos", *this )
// 		, m_shoulderLeft( "ShoulderLeftPos", *this )
// 		, m_elbowLeft( "ElbowLeftPos", *this )
// 		, m_wristLeft( "WristLeftPos", *this )
// 		, m_handLeft( "HandLeftPos", *this )
// 		, m_shoulderRight( "ShoulderRightPos", *this )
// 		, m_elbowRight( "ElbowRightPos", *this )
// 		, m_wristRight( "WristRightPos", *this )
// 		, m_handRight( "HandRightPos", *this )
// 		, m_hipLeft( "HipLeftPos", *this )
// 		, m_kneeLeft( "KneeLeftPos", *this )
// 		, m_ankleLeft( "AnkleLeftPos", *this )
// 		, m_footLeft( "FootLeftPos", *this )
// 		, m_hipRight( "HipRightPos", *this )
// 		, m_kneeRight( "KneeRightPos", *this )
// 		, m_ankleRight( "AnkleRightPos", *this )
// 		, m_footRight( "FootRightPos", *this )
// 		, m_scale( 0 )
// 	{
// 		m_skeletonPorts.push_back(&m_hipCenter);
// 		m_skeletonPorts.push_back(&m_spine);
// 		m_skeletonPorts.push_back(&m_shoulderCenter);
// 		m_skeletonPorts.push_back(&m_head);
// 		m_skeletonPorts.push_back(&m_shoulderLeft);
// 		m_skeletonPorts.push_back(&m_elbowLeft);
// 		m_skeletonPorts.push_back(&m_wristLeft);
// 		m_skeletonPorts.push_back(&m_handLeft);
// 		m_skeletonPorts.push_back(&m_shoulderRight);
// 		m_skeletonPorts.push_back(&m_elbowRight);
// 		m_skeletonPorts.push_back(&m_wristRight);
// 		m_skeletonPorts.push_back(&m_handRight);
// 		m_skeletonPorts.push_back(&m_hipLeft);
// 		m_skeletonPorts.push_back(&m_kneeLeft);
// 		m_skeletonPorts.push_back(&m_ankleLeft);
// 		m_skeletonPorts.push_back(&m_footLeft);
// 		m_skeletonPorts.push_back(&m_hipRight);
// 		m_skeletonPorts.push_back(&m_kneeRight);
// 		m_skeletonPorts.push_back(&m_ankleRight);
// 		m_skeletonPorts.push_back(&m_footRight);

// 		for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
// 		{
// 			Math::Pose pose(Math::Quaternion(0, 0, 0), Math::Vector3d(0,0,1));
// 			m_poseList.push_back(pose);
// 			m_poseListHierarchical.push_back(pose);
// 		}

// 		subgraph->m_DataflowAttributes.getAttributeData( "scale", m_scale );
// 	}

// 	/** destructor */
// 	~KinectSkeletonComponent(){
// 	}

// 	virtual void init(IKinectSensor* sensor){
// 		m_sensor = sensor;
// 		m_hNextEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

// 		// Open a skeleton stream to receive skeleton data
// 		sensor->NuiSkeletonTrackingEnable(m_hNextEvent, 0);
// 		getModule().addHandle(m_hNextEvent);
// 	};

// 	virtual void sendData( Measurement::Timestamp ts);
// 	void getSkeletonPoseList(std::vector< Math::Pose >& poseList);
// protected:
// 	Dataflow::PushSupplier< Measurement::Pose > m_hipCenter;
// 	Dataflow::PushSupplier< Measurement::Pose > m_spine;
// 	Dataflow::PushSupplier< Measurement::Pose > m_shoulderCenter;
// 	Dataflow::PushSupplier< Measurement::Pose > m_head;
// 	Dataflow::PushSupplier< Measurement::Pose > m_shoulderLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_elbowLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_wristLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_handLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_shoulderRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_elbowRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_wristRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_handRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_hipLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_kneeLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_ankleLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_footLeft;
// 	Dataflow::PushSupplier< Measurement::Pose > m_hipRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_kneeRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_ankleRight;
// 	Dataflow::PushSupplier< Measurement::Pose > m_footRight;
// 	std::vector<Dataflow::PushSupplier< Measurement::Pose >* > m_skeletonPorts;

// private:
// 	void makeHierarchicalPose(std::vector< Math::Pose >& poseH , std::vector< Math::Pose >& pose);

// 	std::vector< Math::Pose > m_poseList;
// 	std::vector< Math::Pose > m_poseListHierarchical;
// 	float m_scale;
// };

class Kinect20AbsoluteSkeletonComponent
	: public Kinect20ModuleComponent
{
public:
	/** constructor */
	Kinect20AbsoluteSkeletonComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const Kinect20ComponentKey& componentKey, Kinect20Module* pModule )
		: Kinect20ModuleComponent( name, subgraph, componentKey, pModule )
		, m_skeletonPorts( "Output", *this )
		, m_scale( 1.0 )
		, m_targetDistance( 0.0 )
		, m_targetRange( 0.0 )
	{
		// added 1 for floor clipping plane at the end
		for (int i = 0; i < JointType_Count+1; ++i)
		{
			Math::Pose pose(Math::Quaternion(0, 0, 0), Math::Vector3d(0,0,1));
			m_poseList.push_back((pose));
		}

		subgraph->m_DataflowAttributes.getAttributeData( "scale", m_scale );

	  	if ( subgraph->hasEdge( "Output" ) ) {
	  		Graph::UTQLSubgraph::EdgePtr config = subgraph->getEdge( "Output" );

		  	config->getAttributeData( "targetDistance", m_targetDistance );
		  	config->getAttributeData( "targetRange", m_targetRange );
	  	}

	  	LOG4CPP_ERROR(logger2, "Distance: " << m_targetDistance);
	  	LOG4CPP_ERROR(logger2, "Range: " << m_targetRange);

		//LOG4CPP_INFO(logger2, "KinectScale:" << m_scale);
	}

	/** destructor */
	~Kinect20AbsoluteSkeletonComponent(){
	}

	virtual void init(IKinectSensor* sensor){
		m_sensor = sensor;

		if (m_sensor == NULL) {
			return;
		}

		HRESULT hr;

		IBodyFrameSource* bodyFrameSource;
		hr = sensor->get_BodyFrameSource(&bodyFrameSource);
		if (FAILED(hr)) {
			LOG4CPP_ERROR(logger2, "failed to get bodyFrameSource");
			return;
		}

		hr = bodyFrameSource->OpenReader(&m_bodyFrameReader);
		if (FAILED(hr)) {
			LOG4CPP_ERROR(logger2, "failed to open reader");
			return;
		}

		if (bodyFrameSource != NULL) {
			bodyFrameSource->Release();
		}

		m_hNextEvent = NULL;
		hr = m_bodyFrameReader->SubscribeFrameArrived(&m_hNextEvent);
		if (FAILED(hr)) {
			LOG4CPP_ERROR(logger2, "failed to subscribe to body events");
			return;
		}
	};

	virtual void sendData( Measurement::Timestamp ts);
	bool getSkeletonPoseList(std::vector< Math::Pose >& poseList);
protected:
	Dataflow::PushSupplier< Measurement::PoseList > m_skeletonPorts;

private:
	std::vector< Math::Pose > m_poseList;
	float m_scale;
	float m_targetDistance;
	float m_targetRange;
	IBodyFrameReader* m_bodyFrameReader;
};

class Kinect20ImageComponent
	: public Kinect20ModuleComponent
{
	static const int        cDepthWidth = 640;
	static const int        cDepthHeight = 480;
	static const int        cBytesPerPixel = 4;

public:
	/** constructor */
	Kinect20ImageComponent(const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const Kinect20ComponentKey& componentKey, Kinect20Module* pModule)
		: Kinect20ModuleComponent(name, subgraph, componentKey, pModule)
		, m_imagePort("Output", *this)
		, m_imagePortGray("OutputGray", *this)
		, m_imagePortRAW("OutputRAW", *this)
		, m_onlyPlayer(0)
	{
		m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];

		subgraph->getEdge("Output")->getAttributeData("onlyPlayer", m_onlyPlayer);

		LOG4CPP_INFO(logger2, "Kinect20ImageComponent created, module:" << pModule );

		if (subgraph->m_DataflowAttributes.hasAttribute("cameraModelFile")){
			std::string cameraModelFile = subgraph->m_DataflowAttributes.getAttributeString("cameraModelFile");
			LOG4CPP_INFO(logger2, "create undistorter from model file");
			m_undistorter.reset(new Vision::Undistortion(cameraModelFile));
		}
		else {
			std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString("intrinsicMatrixFile");
			std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString("distortionFile");


			LOG4CPP_INFO(logger2, "create undistorter ");
			m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
		}


	}

	/** destructor */
	~Kinect20ImageComponent(){
	}

	void initialize_infrared_lookuptable() {
		// initialize lut matrix - make parameters configurable !!
		float amp = 3.0f;
		float gamma = 1.1f;

		m_irLutMatrix = cv::Mat(1, 256, CV_8UC1);
		unsigned char* ptr = m_irLutMatrix.ptr<unsigned char>();
		for (int i = 0; i < 256; i++)
			ptr[i] = (unsigned char)(std::min(std::pow(amp * ((float)i / 255.0f), gamma), 0.99f) * 255.0f);
	}

	virtual void init(IKinectSensor* sensor)
	{
		m_sensor = sensor;

		if (m_sensor == NULL) {
			return;
		}

		HRESULT hr;

		int targetID = getKey().m_targetID;

		m_hNextEvent = NULL;
		switch (targetID) {
			case 0:
				IColorFrameSource* colorFrameSource;
				hr = sensor->get_ColorFrameSource(&colorFrameSource);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "failed to get colorFrameSource");
					return;
				}

				hr = colorFrameSource->OpenReader(&m_colorFrameReader);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "failed to open reader");
					return;
				}

				if (colorFrameSource != NULL) {
					colorFrameSource->Release();
				}

				hr = m_colorFrameReader->SubscribeFrameArrived(&m_hNextEvent);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2,
						"failed to subscribe to image events (colorFrame)");
					return;
				}
				break;
			case 1:

				initialize_infrared_lookuptable();

				IInfraredFrameSource* infraredFrameSource;
				hr = sensor->get_InfraredFrameSource(&infraredFrameSource);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "failed to get infraredFrameSource");
					return;
				}

				hr = infraredFrameSource->OpenReader(&m_infraredFrameReader);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "failed to open infrared reader");
					return;
				}

				if (infraredFrameSource != NULL) {
					infraredFrameSource->Release();
				}

				hr = m_infraredFrameReader->SubscribeFrameArrived(&m_hNextEvent);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2,
						"failed to subscribe to image events (infraredFrame)");
					return;
				}
				break;
			case 2: // depth frames, either normal or
			case 3: // Depth as Output, UV Map Depth to Color as GrayOutput
				IDepthFrameSource* depthFrameSource;
				hr = sensor->get_DepthFrameSource(&depthFrameSource);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "failed to open depthFrameSource");
					return;
				}

				hr = depthFrameSource->OpenReader(&m_depthFrameReader);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2, "fail to open depth reader");
					return;
				}

				if (depthFrameSource != NULL) {
					depthFrameSource->Release();
				}
				hr = m_depthFrameReader->SubscribeFrameArrived(&m_hNextEvent);
				if (FAILED(hr)) {
					LOG4CPP_ERROR(logger2,
						"failed to subscribe to image events (depthFrame)");
					return;
				}

				break;
			default:
				LOG4CPP_ERROR(logger2, "unknown component targetID: "
					<< targetID);
				break;
		}
	};

	virtual void sendData(Measurement::Timestamp ts);

	int m_onlyPlayer;
protected:
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_imagePort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_imagePortRAW;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_imagePortGray;
	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	boost::shared_ptr< Vision::Image > getColorImage();
	boost::shared_ptr< Vision::Image > getIRImage();
	boost::shared_ptr< Vision::Image > getDepthImage(bool createUVMap, boost::shared_ptr< Vision::Image > &uvMap);

	IColorFrameReader* m_colorFrameReader;
	IDepthFrameReader* m_depthFrameReader;
	IInfraredFrameReader* m_infraredFrameReader;

	BYTE* m_depthRGBX;

	cv::Mat m_irLutMatrix;
};

} } // namespace Ubitrack::Drivers

#endif

