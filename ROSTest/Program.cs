using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using ManagedROS;

namespace ROSTest
{
    class Program
    {
        static void Main(string[] args)
        {
            Publisher p;
            string topic = "par";
            string nodeName = "test_node";
            p = new Publisher(topic, nodeName);
            p.Publish("Hello world!");
        }
    }
}
