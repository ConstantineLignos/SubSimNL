#pragma once
#include "stdafx.h"
#include <string>
#include <iostream>
#include <ros\ros.h>
#include <std_msgs/String.h>
#include <msclr\marshal_cppstd.h>

#include "publisherwrapper.h"
#include "managedpublisher.h"

// Many ROS macros trigger this warning
#pragma warning(disable : 4127)

using namespace System;
using namespace msclr::interop;

namespace ManagedROS {
  void RosNode::RosInit(String^ nodename) {
    RosNodeWrapper::RosInit(marshal_as<std::string>(nodename));
  }

  Publisher::Publisher(String^ topic) {
    m_topic = topic;
    m_pub = new PublisherWrapper(marshal_as<std::string>(topic));
  }

  void Publisher::Publish(String^ data) {
    m_pub->Publish(marshal_as<std::string>(data));
  }
}
