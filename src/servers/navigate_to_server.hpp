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

#ifndef NAVIGATION_SERVER_HPP
#define NAVIGATION_SERVER_HPP

#include <qi/session.hpp>

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <actionlib/server/simple_action_server.h>

#include <nao_interaction_msgs/NavigateToAction.h>

namespace naoqi
{
namespace server
{

typedef actionlib::SimpleActionServer<nao_interaction_msgs::NavigateToAction> NavigateToActionServer;

/**
* @brief NavigationServer basic interface
*/
class NavigationServer
{
public:
  /**
  * @brief Constructor for server
  */
  NavigationServer(const std::string& name,
                   const std::string& function,
                   const qi::SessionPtr& session,
                   const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
    name_(name),
    function_(function),
    session_(session),
    p_navigation_( session->service("ALNavigation")),
    p_motion_( session->service("ALMotion")),
    tf2_buffer_( tf2_buffer )
  {
    func_ = split(name_, '-').back();
  }

  /**
  * @brief Descructor for server
  */
  virtual ~NavigationServer()
  {}

/**
  * @brief initializes/resets the server into ROS with a given nodehandle
  * @param ros NodeHandle to register the server on
  */
  virtual void reset( ros::NodeHandle& nh )=0;

/**
  * @brief getting the descriptive name for this server instance
  * @return string with the name
  */
  std::string name() const
  {
    return name_;
  }

/**
  * @brief getting the descriptive function for this server instance
  * @return string with the function
  */
  std::string function() const
  {
    return function_;
  }

protected:
  const std::string name_;
  std::string func_;
  const std::string function_;

  const qi::SessionPtr& session_;
  qi::AnyObject p_navigation_;
  qi::AnyObject p_motion_;
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

/**
  * @brief splits a string
  * @param input string
  * @param input character to split
  * @param vector of output elements
  */
  void split(const std::string &s,
             char delim,
             std::vector<std::string> &elems)
  {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
      elems.push_back(item);
    }
  }

/**
  * @brief splits a string
  * @param input string
  * @param input character to split
  * @return vector of output elements
  */
  std::vector<std::string> split(const std::string &s,
                                 char delim)
  {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
  }

/**
  * @brief navigates to the pose in any frame
  * @param target pose
  * @return boolean true if succeded, false if not
  */
  bool navigateInAnyFrame(const geometry_msgs::PoseStamped& pose);

/**
  * @brief navigates to the pose in base_footprint frame
  * @param target pose
  * @return boolean true if succeded, false if not
  */
  bool navigateInBaseFootprint(const geometry_msgs::PoseStamped& pose);

/**
  * @brief navigates to the pose in map frame
  * @param target pose
  * @return boolean true if succeded, false if not
  */
  bool navigateInMap(const geometry_msgs::PoseStamped& pose);

/**
  * @brief navigates to the pose
  * @param target pose
  * @return boolean true if succeded, false if not
  */
  bool navigateTo(const geometry_msgs::PoseStamped& pose);

/**
  * @brief stops navigating
  */
  void stopNavigateTo();

/**
  * @brief move to the pose
  * @param x position
  * @param y posiiton
  * @param yaw theta
  * @return boolean true if succeded, false if not
  */
  bool moveTo(const float& x,
              const float& y,
              const float& yaw);

/**
  * @brief get the robot position in map
  * @return the robot pose
  */
  geometry_msgs::PoseStamped getRobotPositionInMap();

/**
  * @brief get yaw from the current pose and target pose
  * @param current pose
  * @param target pose
  * @return yaw
  */
  float getYaw(const geometry_msgs::PoseStamped& pose_current,
               const geometry_msgs::PoseStamped& pose_target);

  /**
    * @brief get distance from one pose and another
    * @param first pose
    * @param second pose
    * @return distance
    */
  float getDistance(const geometry_msgs::PoseStamped& pose_current,
                    const geometry_msgs::PoseStamped& pose_target);
};

/**
* @brief NavigateToServer interface based on NavigationServer
*/
class NavigateToServer : public NavigationServer
{
public:
  NavigateToServer(const std::string& name,
                   const std::string& function,
                   const qi::SessionPtr& session,
                   const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
    NavigationServer(name, function, session, tf2_buffer),
    navigate_to_server_(NULL)
  {}

  ~NavigateToServer()
  {
    navigate_to_server_->shutdown();
    if(navigate_to_server_ != NULL)
      delete navigate_to_server_;
  }

/**
  * @brief reset with the nodehandle
  * @param node handle
  */
  void reset(ros::NodeHandle& nh);

/**
  * @brief callback to execute the server
  * @param request
  */
  void execute(const nao_interaction_msgs::NavigateToGoalConstPtr& req);

private:
  NavigateToActionServer *navigate_to_server_;
};

} // server
} // naoqi
#endif
