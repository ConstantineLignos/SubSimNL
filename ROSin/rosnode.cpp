#pragma once
#include "stdafx.h"
#include "rosnode.h"

using namespace System;
using namespace msclr::interop;

namespace rosin {
  void RosNode::RosInit(String^ nodename) {
    // Give ROS an empty mapping
    std::map<std::string, std::string> mapping;
    ros::init(mapping, marshal_as<std::string>(nodename));
  }

  RosNode::RosNode() {
    m_node = new ros::NodeHandle;
  }
}