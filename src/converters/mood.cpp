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
#include "mood.hpp"
#include "../tools/from_any_value.hpp"

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

MoodConverter::MoodConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session)
  : BaseConverter( name, frequency, session ),
    p_memory_( session->service("ALMemory") ),
    p_mood_( session->service("ALMood") ),
    is_subscribed_(false)
{}

MoodConverter::~MoodConverter()
{
  if (is_subscribed_)
  {
    p_mood_.call<void>("unsubscribe", "ROS");
    is_subscribed_ = false;
  }
}

void MoodConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void MoodConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  std::cout<<"HERE"<<std::endl;
  if (!is_subscribed_)
  {
    is_subscribed_ = true;
  }

  qi::AnyValue anyvalue;
  try {
    anyvalue = p_mood_.call<qi::AnyValue>("persons");
  } catch (std::runtime_error &e) {
    ROS_INFO_STREAM("Error retrieving mood: " << e.what());
    return;
  }
  
  msg_.header.stamp = ros::Time::now();
  
  if(anyvalue.kind() == qi::TypeKind_List) {
    std::cout<<"Size: " <<anyvalue.size()<<std::endl;
    for(int i = 0; i < anyvalue.size(); i++) {
      std::cout<<anyvalue[i].content().kind()<<std::endl;
    }
  } else {
    ROS_INFO("Error retrieving mood list");
  }

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msg_ );
  }
}

void MoodConverter::reset( )
{
  if (is_subscribed_)
  {
    p_mood_.call<void>("subscribe", "ROS", "Active");
    is_subscribed_ = false;
  }
}

} // publisher
} //naoqi
