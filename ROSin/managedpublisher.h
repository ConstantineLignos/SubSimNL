#pragma once

#include "stdafx.h"
#include "publisherwrapper.h"

using namespace System;

namespace ManagedROS {
  public ref class ROSNode {
  public:
    static void ROSInit(String^);
  };

  public ref class Publisher {
  private:
    String^ m_topic;
    PublisherWrapper *m_pub;

  public:
    Publisher(String^);
    void Publish(String^);
  };
}