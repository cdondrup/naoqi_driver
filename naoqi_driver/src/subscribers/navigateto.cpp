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

void spinThread()
{
  ros::spin();
}

NavigateToSubscriber::NavigateToSubscriber( const std::string& name,
                                            const std::string& topic,
                                            const qi::SessionPtr& session ):
  BaseSubscriber( name, topic, session ),
  name_(name),
  navigate_client_(NULL),
  initialized_(false)
{}

void NavigateToSubscriber::reset( ros::NodeHandle& nh )
{
  sub_navigateto_ = nh.subscribe( topic_, 10, &NavigateToSubscriber::callback, this );
  is_initialized_ = true;

  navigate_client_ = new NavigateToAcionClient(nh, name_, true);
  ROS_INFO_STREAM("Starting " << name_ << " client");

  spin_thread_ = new boost::thread(&spinThread);
}

void NavigateToSubscriber::callback( const geometry_msgs::PoseStampedConstPtr& pose_msg )
{
  if (!navigate_client_->isServerConnected())
    while(!navigate_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO_STREAM("Waiting for " << name_ << " server to come up");

  if (initialized_)
    if (navigate_client_->getState() != actionlib::SimpleClientGoalState::LOST)
    {
      navigate_client_->stopTrackingGoal();
      ROS_DEBUG_STREAM(name_ << "client stopped tracking the goal.");
    }
  navigate_client_->cancelAllGoals();

  nao_interaction_msgs::NavigateToGoal goal;
  goal.target_pose = *pose_msg;
  goal.dist_threshold = 0.3;
  goal.nb_attempts = 3;
  navigate_client_->sendGoal(goal);
  initialized_ = true;

  navigate_client_->waitForResult();

  if (navigate_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG_STREAM(name_ << " client succeded.");
  }
  else
    ROS_DEBUG_STREAM(name_ << " client failed.");
}

} // subscriber
} // naoqi
