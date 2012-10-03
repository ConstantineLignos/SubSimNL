#pragma once
#include "stdafx.h"
#include "service.h"
#include "String.h"

using namespace msclr::interop;

namespace rosin {
  ServiceClient::ServiceClient(RosNode^ node, String^ service) {
    ros::NodeHandle *nh = node->node();
    m_client = new ros::ServiceClient(
      nh->serviceClient<nlp_comms::String>(marshal_as<std::string>(service)));
  }

  String^ ServiceClient::call(String^ request) {
    nlp_comms::String msg = nlp_comms::String();
    msg.request.in = marshal_as<std::string>(request);
    // TODO: Handle case where service isn't actually live.
    m_client->call(msg);
    return marshal_as<String^>(msg.response.out);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nlp_comms::String>("upenn_nlp_pipeline_service");

  nlp_comms::String srv_msg;
  srv_msg.request.in = "Go to the hallway.";

  if (client.call(srv_msg)) {
    ROS_INFO("Response: %s", srv_msg.response.out.c_str());
  }
  else {
    ROS_ERROR("Service call failed.");
  }
}