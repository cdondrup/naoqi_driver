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
#include "navigateto.hpp"

namespace naoqi
{
namespace subscriber
{

NavigateToSubscriber::NavigateToSubscriber( const std::string& name,
                                            const std::string& topic,
                                            const qi::SessionPtr& session ):
  BaseSubscriber( name, topic, session ),
  name_(name),
  p_navigation_( session->service("ALNavigation") ),
  navigate_client_(NULL)
{}

void NavigateToSubscriber::reset( ros::NodeHandle& nh )
{
  sub_navigateto_ = nh.subscribe( topic_, 10, &NavigateToSubscriber::callback, this );
  is_initialized_ = true;

  navigate_client_ = new NavigateToAcionClient(nh, name_, true);
  ROS_INFO_STREAM("Starting " << name_ << " client");

  navigate_goal_pub_ = nh.advertise<nao_interaction_msgs::NavigateToActionGoal>(name_ + "/goal", 1);
}

void NavigateToSubscriber::callback( const geometry_msgs::PoseStampedConstPtr& pose_msg )
{
  if (!navigate_client_->isServerConnected())
    while(!navigate_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO_STREAM("Waiting for " << name_ << " server to come up");

  nao_interaction_msgs::NavigateToActionGoal navigate_goal;
  navigate_goal.header.stamp = ros::Time::now();
  navigate_goal.goal.target_pose = *pose_msg;
  navigate_goal_pub_.publish(navigate_goal);
}

} // subscriber
} // naoqi
