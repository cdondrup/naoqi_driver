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


#ifndef MEMORY_SERVICE_HPP
#define MEMORY_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <nao_interaction_msgs/MemoryInsert.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class MemoryService
{
public:
  MemoryService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_memory_(session->service("ALMemory")) {}

  ~MemoryService(){};

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

  const qi::SessionPtr& session_;
  qi::AnyObject p_memory_, p_value_;
  ros::ServiceServer service_;
};

class MemoryInsertService : public MemoryService
{
public:
  MemoryInsertService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : MemoryService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::MemoryInsertRequest& req, nao_interaction_msgs::MemoryInsertResponse& resp);

};

//class BehaviorManagerControlService : public MemoryService
//{
//public:
//  BehaviorManagerControlService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : MemoryService(name, topic, session) {}
//  void reset(ros::NodeHandle& nh);
//  bool callback(nao_interaction_msgs::BehaviorManagerControlRequest& req, nao_interaction_msgs::BehaviorManagerControlResponse& resp);

//};

} // service
} // naoqi
#endif
