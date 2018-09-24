/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RECHARGE_EVENT_REGISTER_HPP
#define RECHARGE_EVENT_REGISTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>
#include <nao_interaction_msgs/RechargeStatus.h>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/recharge.hpp"
// Publisher
#include "../src/publishers/basic.hpp"
// Recorder
#include "../recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
template<class T>
class RechargeEventRegister: public boost::enable_shared_from_this<RechargeEventRegister<T> >
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  RechargeEventRegister();
  RechargeEventRegister(const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session );
  ~RechargeEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void rechargeCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message);
  void rechargeCallbackMessage(std::string &key, qi::AnyValue &value, nao_interaction_msgs::RechargeStatus &msg);
  

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  boost::shared_ptr<converter::RechargeEventConverter<T> > converter_;
  boost::shared_ptr<publisher::BasicPublisher<T> > publisher_;
  //boost::shared_ptr<recorder::BasicEventRecorder<T> > recorder_;

  qi::SessionPtr session_;
  qi::AnyObject p_memory_;
  unsigned int serviceId;
  std::string name_;
  std::map<int, std::string> status_txt;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

protected:
  std::vector<std::string> keys_;
}; // class


class RechargeStatusEventRegister: public RechargeEventRegister<nao_interaction_msgs::RechargeStatus>
{
public:
  RechargeStatusEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session ) : RechargeEventRegister<nao_interaction_msgs::RechargeStatus>(name, keys, frequency, session) {}
};

static bool _qiregisterRechargeEventRegisterStatus() {
  ::qi::ObjectTypeBuilder<RechargeEventRegister<nao_interaction_msgs::RechargeStatus> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, RechargeEventRegister<nao_interaction_msgs::RechargeStatus>, rechargeCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterRechargeEventRegisterStatus();

} //naoqi

#endif
