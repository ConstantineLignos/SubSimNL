#include "stdafx.h"
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include "publisherwrapper.h"

// Many ROS_INFO macro triggers this warning
#pragma warning(disable : 4127)

void ROSNodeWrapper::RosInit(std::string nodename) {
  // Give ROS an empty mapping
  std::map<std::string, std::string> mapping;
  ros::init(mapping, nodename);
}

PublisherWrapper::PublisherWrapper(std::string topic) {
  m_node_handle = ros::NodeHandle();
  m_publisher = m_node_handle.advertise<std_msgs::String>(topic, 1000);
  std::string pub_topic = m_publisher.getTopic();
  ROS_INFO("Created new publisher on topic %s", pub_topic.c_str());
}

void PublisherWrapper::Publish(std::string data) {
  std_msgs::String msg = std_msgs::String();
  msg.data = data;

  m_publisher.publish(msg);
  ROS_INFO("Published: %s", msg.data.c_str());
}

/*
int main() {
  // Init node and publisher. ROSInit should always be called from your main.
  ROSInit("testnode");
  PublisherWrapper p("testtopic");

  int count = 0;
  char buff[255];
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    sprintf_s(buff, "Hello world! %i", count++);
    p.Publish(buff);
    ros::spinOnce();
    loop_rate.sleep();
  }

  char c;
	std::cout << "Enter 'q' to quit" << std::endl;
	std::cin >> c;
	return 0;
}
*/
