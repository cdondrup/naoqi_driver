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

void RechargeGoToStationService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeGoToStationService::callback, this);
}

bool RechargeGoToStationService::callback(nao_interaction_msgs::RechargeRequest &req, nao_interaction_msgs::RechargeResponse &resp) {
  resp.status = p_recharge_.call<int>(func_);
  return true;
}

void RechargeLeaveStationService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RechargeLeaveStationService::callback, this);
}

bool RechargeLeaveStationService::callback(nao_interaction_msgs::RechargeRequest &req, nao_interaction_msgs::RechargeResponse &resp) {
  resp.status = p_recharge_.call<int>(func_);
  return true;
}
}
}
