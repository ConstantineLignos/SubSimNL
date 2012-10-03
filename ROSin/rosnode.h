#pragma once
#include "stdafx.h"

using namespace System;

namespace rosin {
  public ref class RosNode {
  public:
    static void RosNode::RosInit(String^);
    RosNode::RosNode();
    ros::NodeHandle *node() { return m_node; }

  private:
    ros::NodeHandle *m_node;
  };

}