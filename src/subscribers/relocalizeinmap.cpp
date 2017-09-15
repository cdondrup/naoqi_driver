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
#include "relocalizeinmap.hpp"

/*
 * ROS includes
 */
//#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

RelocalizeSubscriber::RelocalizeSubscriber( const std::string& name,
                                            const std::string& topic,
                                            const qi::SessionPtr& session,
                                            const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
  BaseSubscriber( name, topic, session ),
  p_navigation_( session->service("ALNavigation") ),
  tf2_buffer_( tf2_buffer )
{}

void RelocalizeSubscriber::reset( ros::NodeHandle& nh )
{
  sub_relocalize_ = nh.subscribe( topic_, 10, &RelocalizeSubscriber::callback, this );
  is_initialized_ = true;
}

void RelocalizeSubscriber::callback( const geometry_msgs::PoseWithCovarianceStamped& pose_msg )
{
  std::string func = "relocalizeInMap";

  if ( pose_msg.header.frame_id != "map" )
  {
    std::cout << func << " in frame " << pose_msg.header.frame_id
              << " is not supported; use the map frame" << std::endl;
  }

  //stop localization
  try
  {
    p_navigation_.call<void>("stopLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.stopLocalization "
              << e.what() << std::endl;
  }

  double yaw = helpers::transform::getYaw(pose_msg.pose.pose);

  std::cout << "going to " << func
            << " x: " <<  pose_msg.pose.pose.position.x
            << " y: " << pose_msg.pose.pose.position.y
            << " z: " << pose_msg.pose.pose.position.z
            << " yaw: " << yaw << std::endl;

  std::vector<float> pose(3);
  pose[0] = pose_msg.pose.pose.position.x;
  pose[1] = pose_msg.pose.pose.position.y;
  pose[2] = yaw;

  try
  {
    p_navigation_.call<void>(func, pose);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation." << func << " : "
              << e.what() << std::endl;
  }

  //start localization
  try
  {
    p_navigation_.call<void>("startLocalization");
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception caught in ALNavigation.startLocalization "
              << e.what() << std::endl;
  }
}

} // subscriber
} // naoqi
