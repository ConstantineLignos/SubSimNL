// talker.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <string>
#include <iostream>
#include <sstream>
#include <msclr\marshal_cppstd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace System;
using namespace msclr::interop;

namespace ManagedROS {

  public ref class Publisher {
  private:
    ros::NodeHandle* node_handle_;
    ros::Rate* loop_rate_;
    ros::Publisher* publisher_;
  public:
    Publisher(String^, String^);
    void Publish(String^);
  };

  Publisher::Publisher(String^ topic, String^ node_name) {
    // Marshalled arguments
    std::string topic_m = marshal_as<std::string>(topic);
    std::string node_name_m = marshal_as<std::string>(topic);

    // Dummy arguments for ROS initialization
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, node_name_m);
    node_handle_ = new ros::NodeHandle();
    publisher_ = &(node_handle_->advertise<std_msgs::String>(topic_m, 1000));
    loop_rate_ = new ros::Rate(10);
  }

  void Publisher::Publish(String^ data) {
    std_msgs::String msg;
    msg.data = marshal_as<std::string>(data);
    ROS_INFO("%s", msg.data.c_str());

    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor above.
    */
    publisher_->publish(msg);
  }
}