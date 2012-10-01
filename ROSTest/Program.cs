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
            ROSNode.ROSInit(nodeName);
            Thread.Sleep(5000);

            string topic = "test_topic";
            Publisher p = new Publisher(topic);

            int i = 0;
            while (true)
            {
                p.Publish("Test message " + i++);
                Thread.Sleep(5);
            }
        }
    }
}
