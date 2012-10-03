#pragma once
#include "stdafx.h"
#include "rosnode.h"

namespace rosin {
  public ref class ServiceClient{
  public:
    ServiceClient::ServiceClient(RosNode^, String^);
    String^ ServiceClient::call(String^);    

  private:
    ros::ServiceClient* m_client;
  };
}