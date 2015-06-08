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


#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <alrosbridge/publisher/publisher.hpp>
#include <alrosbridge/tools.hpp>
#include <qi/session.hpp>

#include <boost/filesystem.hpp>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifdef CATKIN_BUILD
#include <ros/package.h>
#endif

namespace alros
{
namespace helpers
{

inline double getYaw(const geometry_msgs::Pose& pose)
{
  double yaw, _pitch, _roll;
  tf2::Matrix3x3(tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w)).getEulerYPR(yaw, _pitch, _roll);
  return yaw;
}

inline bool hasSameTopic( const publisher::Publisher& first, const publisher::Publisher& second )
{
  if ( first.topic() == second.topic() )
    return true;
  else
    return false;
}

inline dataType::DataType getDataType(qi::AnyValue value)
{
  dataType::DataType type;
  if (value.kind() == qi::TypeKind_Int) {
    type = dataType::Int;
  }
  else if (value.kind() == qi::TypeKind_Float) {
    type = dataType::Float;
  }
  else if (value.kind() == qi::TypeKind_String) {
    type = dataType::String;
  }
  else {
    throw std::runtime_error("Cannot get a valid type.");
  }
  return type;
}

static const float bufferDefaultDuration = 10.f;
inline void getFiles(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
  if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

  boost::filesystem::recursive_directory_iterator it(root);
  boost::filesystem::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      ret.push_back(it->path().filename());
    }
    ++it;
  }
}

static const std::string boot_config_file_name = "boot_config.json";

inline std::string& getBootConfigFile()
{
#ifdef CATKIN_BUILD
  static std::string path = ros::package::getPath("naoqi_rosbridge")+"/share/"+boot_config_file_name;
  std::cout << "found a catkin prefix " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData( "/", boot_config_file_name );
  std::cout << "found a qibuild path " << path << std::endl;
  return path;
#endif
}

inline std::string& getURDF( std::string filename )
{
#ifdef CATKIN_BUILD
  static std::string path = ros::package::getPath("naoqi_rosbridge")+"/share/urdf/"+filename;
  std::cout << "found a catkin URDF " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData( "/urdf/", filename );
  std::cout << "found a qibuild URDF " << path << std::endl;
  return path;
#endif
}

} //helpers
} // alros

#endif
