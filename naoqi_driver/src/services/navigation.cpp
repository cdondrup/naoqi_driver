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

#include "navigation.hpp"
#include "../helpers/driver_helpers.hpp"
#include "../helpers/transform_helpers.hpp"
#include "../tools/from_any_value.hpp"

namespace naoqi
{
namespace service
{

void NavigationEmptyService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &NavigationEmptyService::callback, this);
}

bool NavigationEmptyService::callback(std_srvs::EmptyRequest& req,
                                      std_srvs::EmptyResponse& resp)
{
  try
  {
    p_navigation_.call<void>(func_);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func_ << " : "
              << e.what() << std::endl;
    return false;
  }
  return true;
}

void NavigateToService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &NavigateToService::callback, this);
}

bool NavigateToService::callback(nao_interaction_msgs::GoToPoseRequest& req,
                                 nao_interaction_msgs::GoToPoseResponse& resp)
{
  if (req.pose.header.frame_id == "base_footprint")
  {
    double yaw = helpers::transform::getYaw(req.pose.pose);

    std::cout << func_ << " in " << req.pose.header.frame_id
              << "to x: " << req.pose.pose.position.x
              << " y: " << req.pose.pose.position.y
              << " z: " << req.pose.pose.position.z
              << " yaw: " << yaw << std::endl;

    try
    {
      p_navigation_.call<void>(func_,
                               req.pose.pose.position.x,
                               req.pose.pose.position.y,
                               yaw);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALNavigation." << func_ << " : "
                << e.what() << std::endl;
      return false;
    }
  }
  else if (req.pose.header.frame_id == "map")
  {
    double yaw = helpers::transform::getYaw(req.pose.pose);
    std::cout << "navigateToInMap in " << req.pose.header.frame_id
              << "to x: " << req.pose.pose.position.x
              << " y: " << req.pose.pose.position.y
              << " z: " << req.pose.pose.position.z
              << " yaw: " << yaw << std::endl;

    std::vector<float> pose(3);
    pose[0] = req.pose.pose.position.x;
    pose[1] = req.pose.pose.position.y;
    pose[2] = yaw;

    try
    {
      p_navigation_.call<void>("navigateToInMap", pose);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALNavigation.navigateToInMap : "
                << e.what() << std::endl;
      return false;
    }
  }
  else
  {
    std::string frame = "base_footprint";
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform(frame,
                                                  req.pose.header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2));
    if (!canTransform)
    {
      std::cout << "Cannot transform from " << req.pose.header.frame_id
                << " to " << frame << std::endl;
      return false;
    }

    try
    {
      tf2_buffer_->transform(req.pose,
                             pose_msg_bf,
                             frame,
                             ros::Time(0),
                             req.pose.header.frame_id);

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
      std::cout << func_ << " in " << frame
                << " to x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;

      p_navigation_.call<void>(func_,
                               pose_msg_bf.pose.position.x,
                               pose_msg_bf.pose.position.y,
                               yaw);
    }
    catch( const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << "navigateto position in frame_id " << req.pose.header.frame_id
                << " is not supported; use the " << frame << " frame" << std::endl;
      return false;
    }
    catch( const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
      return false;
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALNavigation." << func_ << " : "
                << e.what() << std::endl;
      return false;
    }
  }

  return true;
}

void ExploreService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &ExploreService::callback, this);
}

bool ExploreService::callback(nao_interaction_msgs::ExploreRequest& req,
                              nao_interaction_msgs::ExploreResponse& resp)
{
  bool res(false);
  resp.path_to_map = "";

  //explore
  ROS_INFO_STREAM("Starting exploration in " << req.radius << " meters");
  try
  {
    int error_code = p_navigation_.call<int>(func_, req.radius);
    if (error_code != 0)
    {
      ROS_ERROR_STREAM(func_ << " failed. Error code " << error_code);
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func_ << " : "
              << e.what() << std::endl;
    return false;
  }
  ROS_INFO("Finished exploration");

  //stop exploration if it did not stop yet
  try
  {
    p_navigation_.call<void>("stopExploration");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopExploration : "
              << e.what() << std::endl;
  }

  //save exploration
  try
  {
    resp.path_to_map = p_navigation_.call<std::string>("saveExploration");

    if (resp.path_to_map.empty())
    {
      ROS_ERROR("The path to the explored map is empty.");
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.saveExploration : "
              << e.what() << std::endl;
    return false;
  }

  //stop localization
  try
  {
    p_navigation_.call<void>("stopLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopLocalization : "
              << e.what() << std::endl;
  }

  //load exploration
  try
  {
    res = p_navigation_.call<bool>("loadExploration", resp.path_to_map);

    if (!res)
    {
      ROS_ERROR("The explored map cannot be loaded.");
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.loadExploration : "
              << e.what() << std::endl;
    return false;
  }

  //relocalize
  if (res)
    ROS_INFO("Now, please move your robot to zero position in the map and \
      relocalize with RelocalizeInMapService (0,0,0)");

  return res;
}

void LoadExplorationService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &LoadExplorationService::callback, this);
}

bool LoadExplorationService::callback(nao_interaction_msgs::LoadExplorationRequest& req,
                                      nao_interaction_msgs::LoadExplorationResponse& resp)
{
  resp.result = false;

  if (req.path_to_map.empty())
  {
    ROS_ERROR("The path to the map is empty.");
    return false;
  }

  //stop localization
  try
  {
    p_navigation_.call<void>("stopLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopLocalization : "
              << e.what() << std::endl;
  }

  //load exploration
  try
  {
    resp.result = p_navigation_.call<bool>(func_, req.path_to_map);

    if (!resp.result)
      ROS_ERROR("The explored map cannot be loaded.");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func_ << " : "
              << e.what() << std::endl;
    return false;
  }

  return resp.result;
}

void RelocalizeInMapService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &RelocalizeInMapService::callback, this);
}

bool RelocalizeInMapService::callback(nao_interaction_msgs::RelocalizeInMapRequest& req,
                                      nao_interaction_msgs::RelocalizeInMapResponse& resp)
{
  resp.result = false;
  //stop localization
  try
  {
    p_navigation_.call<void>("stopLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopLocalization : "
              << e.what() << std::endl;
  }

  //relocalize
  if (req.pose.header.frame_id == "map")
  {
    pose_[0] = req.pose.pose.position.x;
    pose_[1] = req.pose.pose.position.y;
    pose_[2] = helpers::transform::getYaw(req.pose.pose);

    try
    {
      p_navigation_.call<void>(func_, pose_);
      resp.result = true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALNavigation." << func_ << " : "
                << e.what() << std::endl;
    }
  }
  else
  {
    std::string frame("map");
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform(frame,
                                                  req.pose.header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2));

    if (!canTransform) {
      std::cerr << "Cannot transform from " << req.pose.header.frame_id
                << " to " << frame << std::endl;
    }
    else
    {
      try
      {
        tf2_buffer_->transform( req.pose, pose_msg_bf,
                                frame,
                                ros::Time(0),
                                req.pose.header.frame_id );

        double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
        std::cout << "going to navigate x: " << pose_msg_bf.pose.position.x
                  << " y: " << pose_msg_bf.pose.position.y
                  << " z: " << pose_msg_bf.pose.position.z
                  << " yaw: " << yaw << std::endl;

        pose_[0] = pose_msg_bf.pose.position.x;
        pose_[1] = pose_msg_bf.pose.position.y;
        pose_[2] = yaw;

        p_navigation_.call<void>(func_, pose_);
        resp.result = true;
      }
      catch( const tf2::LookupException& e)
      {
        std::cout << e.what() << std::endl;
        std::cout << func_ << " in frame_id " << req.pose.header.frame_id
                  << "is not supported; use the " << frame << " frame" << std::endl;
      }
      catch( const tf2::ExtrapolationException& e)
      {
        std::cout << "received an error on the time lookup" << std::endl;
      }
      catch (const std::exception& e)
      {
        std::cerr << "Exception caught in ALNavigation." << func_ << " : "
                  << e.what() << std::endl;
      }
    }
  }

  //start localization
  try
  {
    p_navigation_.call<void>("startLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.startLocalization : "
              << e.what() << std::endl;
    return false;
  }

  return resp.result;
}

void GetRobotPositionInMapService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &GetRobotPositionInMapService::callback, this);
}

bool GetRobotPositionInMapService::callback(nao_interaction_msgs::GetRobotPositionInMapRequest& req,
                                            nao_interaction_msgs::GetRobotPositionInMapResponse& resp)
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
    resp.pose = geometry_msgs::PoseStamped();
    return false;
  }

  std::vector < std::vector<float> > position_in_map;
  tools::fromAnyValueToFloatVectorVector(anyvalues, position_in_map);

  if (position_in_map.empty())
  {
    resp.pose = geometry_msgs::PoseStamped();
    return false;
  }

  if (position_in_map.size() <= 0)
  {
    resp.pose = geometry_msgs::PoseStamped();
    return false;
  }

  if (position_in_map[0].size() != 3)
  {
    resp.pose = geometry_msgs::PoseStamped();
    return false;
  }

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

  resp.pose = pose;
  return true;
}

}
}
