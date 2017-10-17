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

#include "motion.hpp"
#include "../helpers/driver_helpers.hpp"
#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace service
{

void MotionEmptyService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &MotionEmptyService::callback, this);
}

bool MotionEmptyService::callback(std_srvs::EmptyRequest& req,
                                  std_srvs::EmptyResponse& resp)
{
  try
  {
    p_motion_.call<void>(func_);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALMotion." << func_ << " : "
              << e.what() << std::endl;
  }
  return true;
}

void EnableBreathingService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &EnableBreathingService::callback, this);
}

bool EnableBreathingService::callback(nao_interaction_msgs::SetBreathEnabledRequest& req,
                                      nao_interaction_msgs::SetBreathEnabledResponse& resp)
{
  if(req.chain_name.compare(req.HEAD) == 0
          || req.chain_name.compare(req.BODY) == 0
          || req.chain_name.compare(req.ARMS) == 0
          || req.chain_name.compare(req.LEGS) == 0
          || req.chain_name.compare(req.LARM) == 0
          || req.chain_name.compare(req.RARM) == 0)
  {
    try
    {
      p_motion_.call<void>(func_, req.chain_name, req.enable);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALMotion." << func_ << " : "
                << e.what() << std::endl;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unknown chain_name '" << req.chain_name <<"'");
    return false;
  }
  return true;
}

void MoveToService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &MoveToService::callback, this);
}

bool MoveToService::callback(nao_interaction_msgs::GoToPoseRequest& req,
                             nao_interaction_msgs::GoToPoseResponse& resp)
{
  if ( req.pose.header.frame_id == "base_footprint" )
  {
    double yaw = helpers::transform::getYaw(req.pose.pose);

    std::cout << "going to " << func_
              << " to x: " <<  req.pose.pose.position.x
              << " y: " << req.pose.pose.position.y
              << " z: " << req.pose.pose.position.z
              << " yaw: " << yaw << std::endl;
    try
    {
      p_motion_.call<void>(func_,
                           req.pose.pose.position.x,
                           req.pose.pose.position.y,
                           yaw);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALMotion." << func_ << " : "
                << e.what() << std::endl;
    }
  }
  else
  {
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform("base_footprint",
                                                  req.pose.header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2));
    if (!canTransform)
    {
      std::cout << "Cannot transform from " << req.pose.header.frame_id
                << " to base_footprint" << std::endl;
      return false;
    }
    try
    {
      tf2_buffer_->transform(req.pose,
                             pose_msg_bf,
                             "base_footprint",
                             ros::Time(0),
                             req.pose.header.frame_id);

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
      std::cout << "odom to " << func_
                << " to x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;

      p_motion_.call<void>(func_,
                           pose_msg_bf.pose.position.x,
                           pose_msg_bf.pose.position.y,
                           yaw);
    }
    catch(const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << func_ << " in frame_id " << req.pose.header.frame_id
                << "is not supported, only in base_footprint" << std::endl;
    }
    catch(const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALMotion." << func_ << " : "
                << e.what() << std::endl;
    }
  }
  return true;
}

void SetExternalCollisionService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &SetExternalCollisionService::callback, this);
}

bool SetExternalCollisionService::callback(nao_interaction_msgs::SetExternalCollisionRequest& req,
                                           nao_interaction_msgs::SetExternalCollisionResponse& resp)
{
  bool res(false);
  if(req.group_name.compare("All") == 0
          || req.group_name.compare("Move") == 0
          || req.group_name.compare("Arms") == 0
          || req.group_name.compare("LArm") == 0
          || req.group_name.compare("RArm") == 0)
  {
    try
    {
      p_motion_.call<void>(func_, req.group_name, req.enable);
      res = true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception caught in ALMotion." << func_ << " : "
                << e.what() << std::endl;
    }
  }
  else
  {
    ROS_ERROR_STREAM("SetExternalCollisionService: unknown group_name '"
                     << req.group_name << "'");
  }
  resp.result = res;
  return res;
}

void SetMoveArmsEnabledService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &SetMoveArmsEnabledService::callback, this);
}

bool SetMoveArmsEnabledService::callback(nao_interaction_msgs::SetMoveArmsEnabledRequest& req,
                                         nao_interaction_msgs::SetMoveArmsEnabledResponse& resp)
{
  bool res(false);
  try
  {
    p_motion_.call<void>(func_, req.left_arm, req.right_arm);
    res = true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALMotion." << func_ << " : "
              << e.what() << std::endl;
  }
  resp.result = res;
  return res;
}

void FollowPathService::reset(ros::NodeHandle& nh)
{
  service_ = nh.advertiseService(topic_, &FollowPathService::callback, this);
}

bool FollowPathService::callback(nao_interaction_msgs::FollowPathRequest& req,
                                 nao_interaction_msgs::FollowPathResponse& resp)
{
  bool res(false);
  ROS_INFO_STREAM("Service " << name_ << " is running");

  if (req.path_array.size() <= 0)
  {
    ROS_ERROR("The FollowPath trajectory is not correctly defined.");
    return false;
  }

  qi::AnyValue traj_final;

  std::vector<qi::AnyValue> init_dir(2);
  std::vector<qi::AnyValue> goal(2);
  std::vector<qi::AnyValue> fin_dir(2);

  try
  {
    std::vector<qi::AnyValue> traj_composed;
    int traj_nb(0);
    if (req.path_array.size() > 1)
    {
      traj_composed.resize(req.path_array.size()+1);
      traj_composed[traj_nb++] = qi::AnyValue(qi::AnyReference::from("Composed"), false, false);
    }
    else
      traj_composed.resize(1);

    for(int i=0; i<req.path_array.size(); ++i)
    {
      int elem(0);
      std::vector<qi::AnyValue> traj_v;
      traj_v.resize(6);

      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].type), false, false);

      init_dir[0] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].init_dir_x), false, false);
      init_dir[1] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].init_dir_y), false, false);
      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(init_dir), false, false);

      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].init_cur), false, false);

      goal[0] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].goal_x), false, false);
      goal[1] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].goal_y), false, false);
      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(goal), false, false);

      fin_dir[0] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].fin_dir_x), false, false);
      fin_dir[1] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].fin_dir_y), false, false);
      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(fin_dir), false, false);

      traj_v[elem++] = qi::AnyValue(qi::AnyReference::from(req.path_array[i].fin_cur), false, false);

      traj_composed[traj_nb++] = qi::AnyValue(qi::AnyReference::from(traj_v), false, false);
    }

    if (req.path_array.size() > 1)
      traj_final = qi::AnyValue(qi::AnyReference::from(traj_composed), false, false);
    else
      traj_final = traj_composed.back();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in conversion to AnyValue : "
              << e.what() << std::endl;
    return false;
  }

  try
  {
    p_motion_.call<void>("_followPath", traj_final);
    res = true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALMotion." << func_ << " : "
              << e.what() << std::endl;
    return false;
  }
  return res;
}

}
}
