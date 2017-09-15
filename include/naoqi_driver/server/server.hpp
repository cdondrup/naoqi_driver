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

#ifndef SERVER_HPP
#define SERVER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace naoqi
{
namespace server
{

/**
* @brief Server concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible server instance has to implement the virtual functions mentioned in the concept
*/
class Server
{

public:

  /**
  * @brief Constructor for server interface
  */
  template<typename T>
  Server( T srv ):
    srvPtr_( boost::make_shared<ServerModel<T> >(srv) )
  {}

  /**
  * @brief initializes/resets the server into ROS with a given nodehandle,
  * this will be called at first for initialization or again when master uri has changed
  * @param ros NodeHandle to register the server on
  */
  void reset( ros::NodeHandle& nh )
  {
    std::cout << name() << " is resetting" << std::endl;
    srvPtr_->reset( nh );
  }

  /**
  * @brief getting the descriptive name for this server instance
  * @return string with the name
  */
  std::string name() const
  {
    return srvPtr_->name();
  }

  /**
  * @brief getting the function to server on
  * @return string indicating the function
  */
  std::string function() const
  {
    return srvPtr_->function();
  }

private:

  /**
  * BASE concept struct
  */
  struct ServerConcept
  {
    virtual ~ServerConcept(){}
    virtual void reset( ros::NodeHandle& nh ) = 0;
    virtual std::string name() const = 0;
    virtual std::string function() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct ServerModel : public ServerConcept
  {
    ServerModel( const T& other ):
      server_( other )
    {}

    std::string name() const
    {
      return server_->name();
    }

    std::string function() const
    {
      return server_->function();
    }

    bool isInitialized() const
    {
      return server_->isInitialized();
    }

    void reset( ros::NodeHandle& nh )
    {
      server_->reset( nh );
    }

    T server_;
  };

  boost::shared_ptr<ServerConcept> srvPtr_;

}; // class server

} //server
} //naoqi

#endif
