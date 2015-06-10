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
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "HandFusion" ) );

namespace Ubitrack { namespace Components {

/**
 */
class HandFusionComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	HandFusionComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_inPortA( "Skeleton", *this,
			boost::bind( &HandFusionComponent::inFun, this, _1 ) )
		, m_inPortRH( "RightHandPose", *this )
		, m_inPortLH( "LeftHandPose", *this )
		, m_outPort( "FusedSkeleton", *this )
    {
		LOG4CPP_ERROR(logger, "In constructir");
    }

	/** Method that computes the result. */
	void inFun( const Measurement::PoseList& inEvt )
	{
		Measurement::Timestamp t = inEvt.time();

		std::vector<Math::Pose> inList = *inEvt;

		Math::Pose rh = Math::Pose();
		Math::Pose lh = Math::Pose();

		try {
			rh = *m_inPortRH.get(t);
		} catch (...) {
		}
		try {
			lh = *m_inPortLH.get(t);
		} catch (...) {
		}

		/*
		Math::Pose inRH = inList.at(11);
		Math::Pose inLH = inList.at(7);

		if (inRH == Math::Pose()) {
			inList.at(11) = rh;
		}
		if (inLH == Math::Pose()) {
			inList.at(7) = lh;
		}
		*/

		inList.push_back(lh);
		inList.push_back(rh);

		m_outPort.send( Measurement::PoseList( t, inList ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::PushConsumer< Measurement::PoseList > m_inPortA;


	/** Input port B of the component. */
	Dataflow::PullConsumer< Measurement::Pose > m_inPortRH;
	/** Input port B of the component. */
	Dataflow::PullConsumer< Measurement::Pose > m_inPortLH;

	/** Output port of the component. */
	Dataflow::PushSupplier< Measurement::PoseList > m_outPort;
};

class LHSelectorComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	LHSelectorComponent( const std::string& sName,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "PoseList", *this )
		, m_outPort( "Pose", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		std::vector<Math::Pose> inList = *m_inPort.get();
		// 7 : JointType_HandLeft
		m_outPort.send( Measurement::Pose( t, inList.at(7) ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::PoseList > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
};

class LWSelectorComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	LWSelectorComponent( const std::string& sName,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "PoseList", *this )
		, m_outPort( "Pose", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		std::vector<Math::Pose> inList = *m_inPort.get();
		// 7 : JointType_HandLeft
		m_outPort.send( Measurement::Pose( t, inList.at(6) ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::PoseList > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
};


class RHSelectorComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RHSelectorComponent( const std::string& sName,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "PoseList", *this )
		, m_outPort( "Pose", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		std::vector<Math::Pose> inList = *m_inPort.get();
		// 7 : JointType_HandLeft
		m_outPort.send( Measurement::Pose( t, inList.at(11) ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::PoseList > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
};

class RWSelectorComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RWSelectorComponent( const std::string& sName,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "PoseList", *this )
		, m_outPort( "Pose", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		std::vector<Math::Pose> inList = *m_inPort.get();
		// 7 : JointType_HandLeft
		m_outPort.send( Measurement::Pose( t, inList.at(10) ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::PoseList > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
};




UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// PoseList * Pose = PoseList
	cf->registerComponent< HandFusionComponent > ( "HandFusion" );
	cf->registerComponent< LHSelectorComponent > ( "LHSelector" );
	cf->registerComponent< RHSelectorComponent > ( "RHSelector" );
	cf->registerComponent< LWSelectorComponent > ( "LWSelector" );
	cf->registerComponent< RWSelectorComponent > ( "RWSelector" );
}

} } // namespace Ubitrack::Components
