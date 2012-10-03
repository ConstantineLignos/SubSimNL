/*
 *  Demo.cs - Launches the NL SubSim for standalone testing.
 *  Copyright (C) 2011 Constantine Lignos
 *
 *  This file is part of SubSimNL.
 *
 *  SubSimNL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  SubSimNL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with SubSimNL.  If not, see <http://www.gnu.org/licenses/>.
 */

using System;
using System.Windows;
using System.Threading;
using Forms = System.Windows.Forms;
using Brain;
using SubSimProcessorLanguage;
using SSRICSRobot;
using rosin;

namespace SubSimNLDemo
{
    static class Demo
    {
        static bool INIT_BRAIN = true;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            // Forms boilerplate
            Forms.Application.EnableVisualStyles();
            Forms.Application.SetCompatibleTextRenderingDefault(false);

            // Init ROS
            rosin.RosNode.RosInit("SubSimNLDemo");

            // Make a real brain if needed, dummy otherwise
            CBrain brain;
            IRobot robot = null;
            if (INIT_BRAIN)
            {            
                brain = new CBrain();
                brain.simulatorMapFile = "";
                brain.simulatorType = "MobileSim";
                brain.robotHostName = "";
                brain.cameraPort = "";
                brain.useCamera = false;
                brain.useRemoteVoice = false;
                brain.voicePort = "";
                brain.configFileName = @"C:\Users\Administrator\checkout\SubSimNL\SubSimNLDemo\BrainInterface.exe.config";
                try
                {
                    brain.Init(IntPtr.Zero, "");
                    robot = brain.Robot;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                    Forms.Application.Exit();
                }

                // TODO: it's possible we'll have problems with the ros::init going out of scope here
            }
            else
            {
                brain = null;
                robot = null;

                // Start the subsim ourselves
                CSubSimProcessorLanguage nlInput = new CSubSimProcessorLanguage(brain);
                nlInput.Init(brain, robot);
                nlInput.RunStandalone();
            }
        }
    }
}
