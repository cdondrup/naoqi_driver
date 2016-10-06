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


#ifndef TABLET_CONTROL_SERVICE_HPP
#define TABLET_CONTROL_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <std_srvs/Empty.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class TabletControlService
{
public:
  TabletControlService( const std::string& name, const std::string& topic, const qi::SessionPtr& session ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_tablet_(session->service("ALTabletService")) {}

  ~TabletControlService(){};

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
  qi::AnyObject p_tablet_;
  ros::ServiceServer service_;
};

class TabletControlToggleService : public TabletControlService
{
public:
  TabletControlToggleService(const std::string& name, const std::string& topic, const qi::SessionPtr& session) : TabletControlService(name, topic, session) {}
  void reset(ros::NodeHandle& nh);
  bool callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

};

} // service
} // naoqi
#endif
