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


#ifndef NAVIGATETO_SUBSCRIBER_HPP
#define NAVIGATETO_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>

#include <nao_interaction_msgs/NavigateToAction.h>

#include <actionlib/client/simple_action_client.h>

namespace naoqi
{
namespace subscriber
{

typedef actionlib::SimpleActionClient<nao_interaction_msgs::NavigateToAction> NavigateToAcionClient;

class NavigateToSubscriber: public BaseSubscriber<NavigateToSubscriber>
{
public:
  NavigateToSubscriber( const std::string& name,
                        const std::string& topic,
                        const qi::SessionPtr& session);
  ~NavigateToSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void callback( const geometry_msgs::PoseStampedConstPtr& pose_msg );

private:
  std::string name_;
  qi::AnyObject p_navigation_;
  ros::Subscriber sub_navigateto_;
  NavigateToAcionClient *navigate_client_;
  ros::Publisher navigate_goal_pub_;
};

} // subscriber
}// naoqi
#endif
