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

/*
* LOCAL includes
*/
#include "charging_station.hpp"
#include "../tools/from_any_value.hpp"

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace naoqi
{
namespace converter
{
 
PodConverter::PodConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_recharge_( session->service("ALRecharge") )
  
{
}

void PodConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void PodConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  std::string func("getStationPosition");
  std::vector<float> al_pose;
  try
  {
    al_pose = p_recharge_.call<std::vector<float> >( func );
  }
  catch(std::exception& e)
  {
    std::cerr << "Exception caught in ALRecharge." << func << " : "
              << e.what() << std::endl;
  }

  if (!al_pose.empty())
    if (al_pose.size() != 3)
      return;

  float theta = al_pose[2];

  static geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();

  msg.pose.position.x = al_pose[0];
  msg.pose.position.y = al_pose[1];
  msg.pose.position.z = 0.0;
  msg.pose.orientation.z = sin(theta/2);
  msg.pose.orientation.w = cos(theta/2);

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action](msg);
  }
}

void PodConverter::reset( )
{
}

} //converter
} // naoqi
