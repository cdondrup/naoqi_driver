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

#include "memory.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void MemoryInsertService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &MemoryInsertService::callback, this);
}

bool MemoryInsertService::callback(nao_interaction_msgs::MemoryInsertRequest& req, nao_interaction_msgs::MemoryInsertResponse& resp) {
  std::vector<std::vector<std::string> > data;
  for(int i = 0; i < req.to_insert.size(); i++){
    std::vector<std::string> a;
    a.push_back(req.to_insert[i].key);
    a.push_back(req.to_insert[i].value);
    data.push_back(a);
  }
  qi::AnyValue v(65);
  std::cout<<v.<<std::endl;
  p_memory_.call<void>(name_, data);
  return true;
}

//void BehaviorManagerControlService::reset( ros::NodeHandle& nh )
//{
//  service_ = nh.advertiseService(topic_, &BehaviorManagerControlService::callback, this);
//}

//bool BehaviorManagerControlService::callback(nao_interaction_msgs::BehaviorManagerControlRequest& req, nao_interaction_msgs::BehaviorManagerControlResponse& resp) {
//  resp.success = true;
//  if(req.name.compare("ALL")==0 && name_.compare("stopBehavior")==0) { // Stop everything if ALL is given as behavior name
//    p_bm_.call<void>("stopAllBehaviors");
//  } else if(p_bm_.call<bool>("isBehaviorInstalled", req.name)) {
//    if(name_.compare("startBehavior") == 0) {
//      if(p_bm_.call<bool>("isBehaviorRunning", req.name)) {
//        return false; // To make clear that it is successfully started but the service call failed because it was already running
//      } else {
//        p_bm_.call<void>(name_, req.name);
//      }
//    } else {
//      if(!p_bm_.call<bool>("isBehaviorRunning", req.name)) {
//        return false;
//      } else {
//        p_bm_.call<void>(name_, req.name);
//      }
//    }
//  } else {
//    resp.success = false;
//    return false;
//  }
//  return true;
//}

}
}
