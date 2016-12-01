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


#ifndef TEXT_TO_SPEECH_SERVICE_HPP
#define TEXT_TO_SPEECH_SERVICE_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <nao_interaction_msgs/Say.h>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class TextToSpeechService
{
public:
  TextToSpeechService( const std::string& name, const std::string& topic, const qi::SessionPtr& session, const std::string language, const float speed ) :
      name_(name),
      topic_(topic),
      session_(session),
      p_tts_(session->service("ALTextToSpeech"))
  {
    func_ = split(name_, '-').back();
    p_tts_.call<void>("setLanguage", language);
    p_tts_.call<void>("setParameter", "speed", speed);
    p_tts_.call<void>("setParameter", "defaultVoiceSpeed", speed);
  }

  ~TextToSpeechService(){}

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  virtual void reset( ros::NodeHandle& nh )=0;

protected:
  const std::string name_;
  std::string func_;
  const std::string topic_;

  const qi::SessionPtr& session_;
  qi::AnyObject p_tts_;
  ros::ServiceServer service_;

  void split(const std::string &s, char delim, std::vector<std::string> &elems) {
      std::stringstream ss;
      ss.str(s);
      std::string item;
      while (std::getline(ss, item, delim)) {
          elems.push_back(item);
      }
  }

  std::vector<std::string> split(const std::string &s, char delim) {
      std::vector<std::string> elems;
      split(s, delim, elems);
      return elems;
  }
};

class TextToSpeechSayService : public TextToSpeechService
{
public:
  TextToSpeechSayService(const std::string& name, const std::string& topic, const qi::SessionPtr& session, const std::string language, const float speed) : TextToSpeechService(name, topic, session, language, speed) {}
  void reset(ros::NodeHandle& nh);
  bool callback(nao_interaction_msgs::SayRequest& req, nao_interaction_msgs::SayResponse& resp);

};

} // service
} // naoqi
#endif
