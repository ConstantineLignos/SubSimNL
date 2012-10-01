#pragma once

#include "stdafx.h"
#include <string>
#include <ros\ros.h>

class ROSNodeWrapper {
public:
  static void ROSInit(std::string);
};

class PublisherWrapper {
private:
  ros::NodeHandle m_node_handle;
  ros::Publisher m_publisher;
  std::string m_topic;
public:
  PublisherWrapper(std::string);
  void Publish(std::string);
};

