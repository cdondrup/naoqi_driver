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

#include "recharge.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void RechargeEmptyService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeEmptyService::callback, this);
}

bool RechargeEmptyService::callback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  p_recharge_.call<void>(func_);
  return true;
}

void RechargeAsyncService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeAsyncService::callback, this);
}

bool RechargeAsyncService::callback(nao_interaction_msgs::RechargeRequest &req, nao_interaction_msgs::RechargeResponse &resp) {
  /*resp.status =*/ p_recharge_.async<int>(func_); // Cannot assign return value to variable if async. Otherwise, it blocks.
  return true;
}

void RechargeSyncService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeSyncService::callback, this);
}

bool RechargeSyncService::callback(nao_interaction_msgs::RechargeRequest &req, nao_interaction_msgs::RechargeResponse &resp) {
  resp.status = p_recharge_.call<int>(func_);
  return true;
}

void RechargeReturnPoseService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeReturnPoseService::callback, this);
}

bool RechargeReturnPoseService::callback(nao_interaction_msgs::RechargeReturnPoseRequest &req, nao_interaction_msgs::RechargeReturnPoseResponse &resp) {
  qi::AnyReferenceVector anyref;
  std::ostringstream ss;
  qi::AnyValue value = p_recharge_.call<qi::AnyValue>(func_);
  try{
    anyref = value.asListValuePtr();
  }
  catch(std::runtime_error& e)
  {
    ss << "Could not transform AnyValue into list: " << e.what();
    throw std::runtime_error(ss.str());
  }
  qi::AnyReference ref;

  if ( anyref.size() != 2 ) {
    ss << "AnyValue does not have the expected size of 2 but instead has " << anyref.size();
    throw std::runtime_error(ss.str());
  }
  ref = anyref[0].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    resp.status = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve status";
    throw std::runtime_error(ss.str());
  }
  ref = anyref[1].content();
  if(ref.kind() == qi::TypeKind_List)
  {
    if(ref[0].content().kind() == qi::TypeKind_Float) {
        resp.position.position.x = ref[0].content().asFloat();
        resp.position.position.y = ref[1].content().asFloat();
        float theta = ref[2].content().asFloat();
        resp.position.orientation.z = sin(theta/2);
        resp.position.orientation.w = cos(theta/2);
    } else {
        ss << "Entries are not float but " << ref[0].kind();
        throw std::runtime_error(ss.str());
    }
  }
  else
  {
    ss << "Could not retrieve position list";
    throw std::runtime_error(ss.str());
  }
  return true;
}

//(0, 0, sin(theta/2), cos(theta/2))

}
}
