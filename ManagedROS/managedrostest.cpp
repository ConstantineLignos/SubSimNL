#include "stdafx.h"
#include "managedpublisher.h"

using namespace System;
using namespace System::Threading;

using namespace ManagedROS;

int main(array<System::String ^> ^args)
{
  // Init and let things settle
  RosNode::RosInit("testnode");
  Publisher pub("testtopic");
  Thread::Sleep(5000);

  int i = 0;
  while (i++ < 100) {
    pub.Publish("Test message " + i);
    Thread::Sleep(1000);
  }

  return 0;
}
