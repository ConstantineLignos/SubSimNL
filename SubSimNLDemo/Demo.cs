using System;
using System.Windows.Forms;
using Brain;
using SubSimProcessorLanguage;
using SSRICSRobot;

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
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            // Make a real brain if needed, dummy otherwise
            CBrain brain;
            IRobot robot;
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
                brain.configFileName = @"C:\Users\Administrator\Documents\Visual Studio 2008\Projects\SubSimNL_2.0\SubSimNLDemo\BrainInterface.exe.config";
                brain.Init(IntPtr.Zero, ""); 
                robot = brain.Robot;
            }
            else
            {
                brain = null;
                robot = null;
            }

            // Start the subsim
            CSubSimProcessorLanguage nlInput = new CSubSimProcessorLanguage(brain);
            nlInput.standalone = true;
            nlInput.Init(brain, robot);
            nlInput.Run();
        }
    }
}
