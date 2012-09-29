using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Runtime.InteropServices;
using ManagedROS;

namespace ROSTest
{
    class Program
    {
        static void Main(string[] args)
        {
            string nodeName = "test_node";
            RosNode.RosInit(nodeName);
            Thread.Sleep(5000);

            string topic = "test_topic";
            Publisher p = new Publisher(topic);

            for (int i = 0; i < 100; i++)
            {
                p.Publish("Test message " + i);
                Thread.Sleep(1000);
            }
        }
    }
}
