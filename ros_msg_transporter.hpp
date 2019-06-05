/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:18:49 CET 2010  ros_msg_transporter.hpp

                        ros_msg_transporter.hpp -  description
                           -------------------
    begin                : Tue November 16 2010
    copyright            : (C) 2010 Ruben Smits
    email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/


// Copyright  (C)  2010  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _ROS_MSG_TRANSPORTER_HPP_
#define _ROS_MSG_TRANSPORTER_HPP_

#include <rtt/types/TypeTransporter.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/internal/ConnFactory.hpp>
#include <ros/ros.h>
#include <boost/type_traits/is_same.hpp>

#include "ros_convertions.hpp"
#include "ros_publish_activity.hpp"

namespace ros_convertions {
  /** This is never going to be called, as a static comparison of types will
   * always bypass this convertions calls
   *
   * It is there for non-optimizing cases where the call will still be emitted
   * (but the branch never taken)
   */
  template <typename Msg>
  void toROS(Msg& data, Msg const& msg) { }
  /** This is never going to be called, as a static comparison of types will
   * always bypass this convertion calls
   *
   * It is there for non-optimizing cases where the call will still be emitted
   * (but the branch never taken)
   */
  template <typename Msg>
  void fromROS(Msg& data, Msg const& msg) { }
}

namespace RTT {
namespace ros {
  using namespace ::ros;
  /**
   * A ChannelElement implementation to publish data over a ros topic
   * 
   */
  template<typename T,typename Msg>
  class RosPubChannelElement: public RTT::base::ChannelElement<T>,public RosPublisher
  {
    char hostname[1024];
    std::string topicname;
    ros::NodeHandle ros_node;
    ros::Publisher ros_pub;
      //! We must cache the RosPublishActivity object.
    RosPublishActivity::shared_ptr act;
    Msg msg;

  public:

    /** 
     * Contructor of to create ROS publisher ChannelElement, it will
     * create a topic from the name given by the policy.name_id, if
     * this is empty a default is created as hostname/componentname/portname/pid
     * 
     * @param port port for which we will create a the ROS publisher
     * @param policy connection policy containing the topic name and buffer size
     * 
     * @return ChannelElement that will publish data to topics
     */
    RosPubChannelElement(RTT::base::PortInterface* port,const ConnPolicy& policy)
    {
      if ( policy.name_id.empty() ){
	std::stringstream namestr;
	gethostname(hostname, sizeof(hostname));
	
	namestr << hostname<<'/' << port->getInterface()->getOwner()->getName()
		<< '/' << port->getName() << '/'<<this << '/' << getpid();
	policy.name_id = namestr.str();
      }
      topicname=policy.name_id;
      Logger::In in(topicname);
      log(Debug)<<"Creating ROS publisher for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();

      ros_pub = ros_node.advertise<Msg>(policy.name_id, policy.size ? policy.size : 1); // minimum 1
      act = RosPublishActivity::Instance();
      act->addPublisher( this );
    }

    ~RosPubChannelElement() {
      Logger::In in(topicname);
      log(Debug)<<"Destroying RosPubChannelElement"<<endlog();
      act->removePublisher( this );
    }

    /** 
     * Function to see if the ChannelElement is ready to receive inputs
     * 
     * @return always true in our case
     */
    virtual bool inputReady() {
      return true;
    }

    /**
     * Helper function for data_sample. It allows to differentiate the case
     * where SampleType == MsgType from SampleType != MsgType
     */
    template<typename SampleType>
    bool doDataSample(typename RTT::base::ChannelElement<T>::param_t sample, typename boost::disable_if< boost::is_same<SampleType,Msg> >::type* enabler=0)
    {
      try { ros_convertions::toROS(msg, sample); }
      catch(ros_convertions::InvalidROSConvertion const& e)  {
        log(Warning) << "faile to convert sample to ROS: " << e.what() << endlog();
        return false;
      }
      return true;
    }

    /**
     * Helper function for data_sample. It allows to differentiate the case
     * where SampleType == MsgType from SampleType != MsgType
     */
    template<typename SampleType>
    bool doDataSample(typename RTT::base::ChannelElement<T>::param_t sample, typename boost::enable_if< boost::is_same<SampleType,Msg> >::type* enabler=0)
    {
      return true;
    }
    
    /** 
     * Create a data sample, this could be used to allocate the necessary memory, it is not needed in our case
     * 
     * @param sample 
     * 
     * @return always true
     */
    virtual bool data_sample(typename RTT::base::ChannelElement<T>::param_t sample)
    {
      return doDataSample<T>(sample);
    }

    /** 
     * signal from the port that new data is availabe to publish
     * 
     * @return true if publishing succeeded
     */
    bool signal(){
      //Logger::In in(topicname);
      //log(Debug)<<"Requesting publish"<<endlog();
      return act->requestPublish(this);
    }

    /**
     * Helper function for publish. It allows to differentiate the case
     * where SampleType == MsgType from SampleType != MsgType
     */
    template<typename SampleType>
    void doPublishMsg(typename RTT::base::ChannelElement<T>::param_t sample, typename boost::enable_if< boost::is_same<SampleType,Msg> >::type* enabler=0)
    {
      ros_pub.publish(sample);
    }

    /**
     * Helper function for publish. It allows to differentiate the case
     * where SampleType == MsgType from SampleType != MsgType
     */
    template<typename SampleType>
    void doPublishMsg(typename RTT::base::ChannelElement<T>::param_t sample, typename boost::disable_if< boost::is_same<SampleType,Msg> >::type* enabler=0)
    {
      try { ros_convertions::toROS(msg, sample); }
      catch(ros_convertions::InvalidROSConvertion const& e)  {
        log(Warning) << "failed to convert sample to ROS: " << e.what() << endlog();
        return;
      }

      ros_pub.publish(msg);
    }

    void publish(){
      typename RTT::base::ChannelElement<T>::value_t sample; // XXX: real-time !
      // this read should always succeed since signal() means 'data available in a data element'.
      typename RTT::base::ChannelElement<T>::shared_ptr input = this->getInput();
      while( input && (input->read(sample,false) == NewData) ){
        doPublishMsg<T>(sample);
      }
    }
    
  };

  template<typename T, typename Msg>
  class RosSubChannelElement: public RTT::base::ChannelElement<T>
  {
    ros::NodeHandle ros_node;
    ros::Subscriber ros_sub;
    T sample;
    
  public:
    /** 
     * Contructor of to create ROS subscriber ChannelElement, it will
     * subscribe to a topic with the name given by the policy.name_id
     * 
     * @param port port for which we will create a the ROS publisher
     * @param policy connection policy containing the topic name and buffer size
     * 
     * @return ChannelElement that will publish data to topics
     */
    RosSubChannelElement(RTT::base::PortInterface* port, const ConnPolicy& policy)
    {
      log(Debug)<<"Creating ROS subscriber for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      ros_sub=ros_node.subscribe(policy.name_id,policy.size,&RosSubChannelElement::newData,this);
      this->ref();
    }

    ~RosSubChannelElement() {
    }

    virtual bool inputReady() {
      return true;
    }

    template <typename MsgType>
    void doWrite(typename RTT::base::ChannelElement<T>::shared_ptr& output, MsgType const& msg, typename boost::enable_if< boost::is_same<T,MsgType> >::type* enabler=0){
      output->write(msg);
    }
    template <typename MsgType>
    void doWrite(typename RTT::base::ChannelElement<T>::shared_ptr& output, MsgType const& msg, typename boost::disable_if< boost::is_same<T,MsgType> >::type* enabler=0){
      try { ros_convertions::fromROS(sample, msg); }
      catch(ros_convertions::InvalidROSConvertion const& e)  {
        log(Warning) << "failed to convert sample from ROS: " << e.what() << endlog();
        return;
      }

      output->write(sample);
    }
    /** 
     * Callback function for the ROS subscriber, it will trigger the ChannelElement's signal function
     * 
     * @param msg The received message
     */
    void newData(const Msg& msg){
      typename RTT::base::ChannelElement<T>::shared_ptr output = this->getOutput();
      if (output){
        doWrite(output, msg);
      }
    }
  };

  template <class T, class Msg>
  class RosMsgTransporter : public RTT::types::TypeTransporter{
    virtual RTT::base::ChannelElementBase::shared_ptr createStream (RTT::base::PortInterface *port, const ConnPolicy &policy, bool is_sender) const{
        RTT::base::ChannelElementBase* buf = internal::ConnFactory::buildDataStorage<T>(policy);
        RTT::base::ChannelElementBase::shared_ptr tmp;
      if(is_sender){
        tmp = RTT::base::ChannelElementBase::shared_ptr(new RosPubChannelElement<T,Msg>(port,policy));
        buf->connectTo(tmp);
        return buf;
      }
      else{
        tmp = new RosSubChannelElement<T,Msg>(port,policy);
        tmp->connectTo(buf);
        return tmp;
      }
    }
  };
}} // namespace RTT::ros
#endif
