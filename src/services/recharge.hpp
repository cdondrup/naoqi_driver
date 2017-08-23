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


#ifndef RECHARGE_SERVICE_HPP
#define RECHARGE_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

#include <nao_interaction_msgs/Recharge.h>
#include <nao_interaction_msgs/RechargeReturnPose.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class RechargeService
{
public:
  RechargeService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_recharge_(session->service("ALRecharge")) {
      func_ = split(name_, '-').back();
  }

  ~RechargeService(){};

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  virtual void reset( ros::NodeHandle& nh )=0;

protected:
  const std::string name_;
  const std::string topic_;
  std::string func_;

  const qi::SessionPtr& session_;
  qi::AnyObject p_recharge_;
  ros::ServiceServer service_;

  void split(const std::string &s, char delim, std::vector<std::string> &elems) {
      std::stringstream ss;
      ss.str(s);
      std::string item;
      while (std::getline(ss, item, delim)) {
          elems.push_back(item);
      }
  }

  std::vector<std::string> split(const std::string &s, char delim) {
      std::vector<std::string> elems;
      split(s, delim, elems);
      return elems;
  }
};

class RechargeEmptyService : public RechargeService
{
public:
  RechargeEmptyService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : RechargeService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

};

class RechargeAsyncService : public RechargeService
{
public:
  RechargeAsyncService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : RechargeService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::RechargeRequest& req, nao_interaction_msgs::RechargeResponse& resp);

};

class RechargeSyncService : public RechargeService
{
public:
  RechargeSyncService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : RechargeService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::RechargeRequest& req, nao_interaction_msgs::RechargeResponse& resp);

};

class RechargeReturnPoseService : public RechargeService
{
public:
  RechargeReturnPoseService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : RechargeService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::RechargeReturnPoseRequest& req, nao_interaction_msgs::RechargeReturnPoseResponse& resp);

};

} // service
} // naoqi
#endif
