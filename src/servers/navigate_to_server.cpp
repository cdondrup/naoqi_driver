/*
 * Copyright 2017 SoftBank Robotics Europe
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

#include "navigate_to_server.hpp"

#include "../helpers/transform_helpers.hpp"
#include "../tools/from_any_value.hpp"

#include <tf2/transform_datatypes.h>

namespace naoqi
{
namespace server
{

bool NavigationServer::navigateInAnyFrame(const geometry_msgs::PoseStamped& pose)
{
  bool res(false);
  std::string func("navigateTo");

  std::string frame("base_footprint");
  geometry_msgs::PoseStamped pose_msg_bf;
  bool canTransform = tf2_buffer_->canTransform(frame,
                                                pose.header.frame_id,
                                                ros::Time(0),
                                                ros::Duration(2));
  if (!canTransform)
  {
    std::cout << "Cannot transform from "
              << pose.header.frame_id
              << " to " << frame << std::endl;
  }
  else
  {
    try
    {
      tf2_buffer_->transform(pose,
                             pose_msg_bf,
                             frame,
                             ros::Time(0),
                             pose.header.frame_id);

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);

      std::cout << func << " in " << frame
                << " to x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;

      p_navigation_.post(func,
                         pose_msg_bf.pose.position.x,
                         pose_msg_bf.pose.position.y,
                         yaw);
      res = true;
    }
    catch( const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << func << " in frame " << pose.header.frame_id
                << " is not supported; use the " << frame
                << " frame" << std::endl;
    }
    catch( const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALNavigation." << func << " : "
                << e.what() << std::endl;
    }
  }

  return res;
}

bool NavigationServer::navigateInBaseFootprint(const geometry_msgs::PoseStamped& pose)
{
  bool res(false);
  std::string func("navigateTo");

  double yaw = helpers::transform::getYaw(pose.pose);

  std::cout << func << " in " << pose.header.frame_id
            << " to x:" << pose.pose.position.x
            << " y: " << pose.pose.position.y
            << " z: " << pose.pose.position.z
            << " yaw: " << yaw << std::endl;

  try
  {
    p_navigation_.post(func,
                       pose.pose.position.x,
                       pose.pose.position.y,
                       yaw);
    res = true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func << " : "
              << e.what() << std::endl;
  }

  return res;
}

bool NavigationServer::navigateInMap(const geometry_msgs::PoseStamped& pose)
{
  bool res(false);
  std::string func("navigateToInMap");

  double yaw = helpers::transform::getYaw(pose.pose);

  std::cout << func << " in " << pose.header.frame_id
            << " to x:" << pose.pose.position.x
            << " y: " << pose.pose.position.y
            << " z: " << pose.pose.position.z
            << " yaw: " << yaw << std::endl;

  std::vector<float> pose_vector(3);
  pose_vector[0] = pose.pose.position.x;
  pose_vector[1] = pose.pose.position.y;
  pose_vector[2] = yaw;

  try
  {
    int error_code = p_navigation_.call<int>(func, pose_vector);

    if (error_code == 0)
      res = true;
    else
      ROS_INFO_STREAM("ALNavigation." << func
                      << " returned an error code " << error_code);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func << " : "
              << e.what() << std::endl;
  }

  return res;
}

bool NavigationServer::navigateTo(const geometry_msgs::PoseStamped& pose)
{
  bool res(false);
  if (pose.header.frame_id == "base_footprint")
  {
    res = navigateInBaseFootprint(pose);
  }
  else if (pose.header.frame_id == "map")
  {
    res = navigateInMap(pose);
  }
  else
  {
    res = navigateInAnyFrame(pose);
  }
  return res;
}

void NavigationServer::stopNavigateTo()
{
  try
  {
    p_navigation_.call<void>("stopNavigateTo");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopNavigateTo : "
              << e.what() << std::endl;
  }
}

geometry_msgs::PoseStamped NavigationServer::getRobotPositionInMap()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();

  qi::AnyValue anyvalues;
  try
  {
    anyvalues = p_navigation_.call<qi::AnyValue>("getRobotPositionInMap");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.getRobotPositionInMap : "
              << e.what() << std::endl;
    return geometry_msgs::PoseStamped();
  }

  std::vector < std::vector<float> > position_in_map;
  tools::fromAnyValueToFloatVectorVector(anyvalues, position_in_map);


  if (position_in_map.empty())
    return geometry_msgs::PoseStamped();

  if (position_in_map.size() <= 0)
    return geometry_msgs::PoseStamped();

  if (position_in_map[0].size() != 3)
    return geometry_msgs::PoseStamped();

  //set the position
  pose.pose.position.x = position_in_map[0][0];
  pose.pose.position.y = position_in_map[0][1];

  //set the orientation
  tf2::Vector3 axis;
  axis[0] = 0;
  axis[1] = 0;
  axis[2] = 1;

  tf2::Quaternion q = tf2::Quaternion(axis, position_in_map[0][2]);
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();

  return pose;
}

bool NavigationServer::moveTo(const float& x,
                              const float& y,
                              const float& yaw)
{
  std::string func("moveTo");

  std::cout << "going to " << func
            << " to x: " <<  x
            << " y: " << y
            << " yaw: " << yaw << std::endl;
  try
  {
    p_motion_.call<void>(func, x, y, yaw);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALMotion." << func << " : "
              << e.what() << std::endl;
    return false;
  }
  return true;
}

float NavigationServer::getYaw(const geometry_msgs::PoseStamped& pose_current,
                               const geometry_msgs::PoseStamped& pose_target)
{
  //get the current orientation
  tf2::Quaternion q_current = tf2::Quaternion(pose_current.pose.orientation.x,
                                              pose_current.pose.orientation.y,
                                              pose_current.pose.orientation.z,
                                              pose_current.pose.orientation.w);

  //get the target orientation
  tf2::Quaternion q_target = tf2::Quaternion(pose_target.pose.orientation.x,
                                             pose_target.pose.orientation.y,
                                             pose_target.pose.orientation.z,
                                             pose_target.pose.orientation.w);

  //get an angle between two quaternions
  tf2::Quaternion orientation_target = q_target * q_current.inverse();

  //moveTo
  double yaw = helpers::transform::getYaw(orientation_target);

  return yaw;
}

float NavigationServer::getDistance(const geometry_msgs::PoseStamped& pose1,
                                    const geometry_msgs::PoseStamped& pose2)
{
  float res(0.0f);

  res = std::sqrt((pose1.pose.position.x - pose2.pose.position.x)
                  * (pose1.pose.position.x - pose2.pose.position.x)
                  + (pose1.pose.position.y - pose2.pose.position.y)
                  * (pose1.pose.position.y - pose2.pose.position.y));
  return res;
}

void NavigateToServer::reset( ros::NodeHandle& nh )
{
  navigate_to_server_ = new NavigateToActionServer(nh,
                                                   function_,
                                                   boost::bind(&NavigateToServer::execute, this, _1),
                                                   false);

  navigate_to_server_->start();
  ROS_INFO_STREAM(function_ << " server is started.");
}

void NavigateToServer::execute(const nao_interaction_msgs::NavigateToGoalConstPtr& goal)
{
  bool res(false);
  geometry_msgs::PoseStamped goal_pose(goal->target_pose);
  nao_interaction_msgs::NavigateToResult result;
  nao_interaction_msgs::NavigateToFeedback feedback;

  if(navigate_to_server_->isPreemptRequested())
  {
    if(navigate_to_server_->isNewGoalAvailable())
    {
      nao_interaction_msgs::NavigateToGoal new_goal =
              *navigate_to_server_->acceptNewGoal();
      goal_pose = new_goal.target_pose;
      ROS_INFO("New goal is set");

      ROS_INFO("Stopping to navigate to");
      stopNavigateTo();
    }
    else
    {
      ROS_DEBUG_NAMED(function_, "Navigate_to server preempting the current goal");
      navigate_to_server_->setPreempted();

      return;
    }
  }

  //NavigateTo
  res = navigateTo(goal_pose);

  //get the current robot position in the map
  geometry_msgs::PoseStamped pose_current = getRobotPositionInMap();

  //aproach if needed
  int attempts(0);
  while ((getDistance(pose_current, goal_pose) > goal->dist_threshold)
         && (attempts < goal->nb_attempts))
  {
    res = navigateTo(goal_pose);
    pose_current = getRobotPositionInMap();
    feedback.base_position = pose_current;
    ++attempts;
  }

  result.base_position = pose_current;

  //apply orientation if needed
  if ((goal_pose.header.frame_id == "map") && res)
  {
    float yaw = getYaw(pose_current, goal_pose);
    moveTo(0, 0, yaw);
  }

  //publish the result
  if (res)
  {
    ROS_INFO_STREAM(function_ << " server : exectution succeded.");
    navigate_to_server_->setSucceeded(result);
  }
  else
  {
    ROS_INFO_STREAM(function_ << " server : exectution failed.");
    navigate_to_server_->setAborted();
  }
}

} // server
} // naoqi
