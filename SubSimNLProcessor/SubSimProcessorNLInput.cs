/*
 *  SubSimProcessorNLInput.cs - A SubSimProcessor for natural language input.
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
using System.Collections.Generic;
using System.Threading;
using System.Windows.Forms;
using Brain;
using rosin;
using SSRICSRobot;

namespace SubSimProcessorLanguage
{
    /// <summary>
    /// A SubSim processor that provides natural language input capabilities.
    /// </summary>
    public class CSubSimProcessorLanguage : CSubSimProcessor
    {
        /// <summary>
        /// Whether we should run a standalone UI.
        /// </summary>
        public Boolean standalone;

        private CBrain brain;
        private IRobot robot;
        private GoalBuilder failureGoal;
        private RosNode node;
        private ServiceClient pipelineClient;
        private SemanticsInterface semantics;
        private Form nlForm;

        /// <summary>
        /// Initializes a new instance of the CSubSimProcessorLanguage class.
        /// </summary>
        /// <param name="brain">The brain the SubSim is operating in</param>
        public CSubSimProcessorLanguage(CBrain brain)
            : base(brain)
        {
            standalone = false;
            node = null;
        }

        /// <summary>
        /// Create default goals.
        /// </summary>
        /// <param name="brain">The brain</param>
        /// <param name="robot">The robot</param>
        public override void Init(Brain.CBrain brain, IRobot robot)
        {
            this.brain = brain;
            this.robot = robot;

            // Make default goals
            GoalBuilder gotoGoal = new GoalBuilder("GotoX", brain);
            int ruleIndex = gotoGoal.AddRule("GotoX");
            gotoGoal.AddAndAntecedent(ruleIndex, "CurrentDestination Arg *=*;1");
            gotoGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"Going to the $CurrentDestination_\"");
            gotoGoal.AddConsequent(ruleIndex, "Execs", "Motion GoTo $CurrentDestination close");
            gotoGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"Commander, I finished moving to the $CurrentDestination_\"");
            gotoGoal.AddConsequent(ruleIndex, "Remove", "CurrentDestination Arg");
            gotoGoal.AddConsequent(ruleIndex, "Quit", "");
            gotoGoal.Commit();

            failureGoal = new GoalBuilder("DidNotUnderstand", brain);
            ruleIndex = failureGoal.AddRule("DidNotUnderstand");
            failureGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"I didn't understand what you said.\"");
            failureGoal.AddConsequent(ruleIndex, "Quit", "");
            failureGoal.Commit();

            GoalBuilder goingToGoal = new GoalBuilder("SayGoingToX", brain);
            ruleIndex = goingToGoal.AddRule("SayGoingToX");
            goingToGoal.AddAndAntecedent(ruleIndex, "CurrentDestination Arg *=*;1");
            goingToGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"I'm Going to the $CurrentDestination_\"");
            goingToGoal.AddConsequent(ruleIndex, "Quit", "");
            goingToGoal.Commit();

            GoalBuilder goingNowhereGoal = new GoalBuilder("SayGoingNowhere", brain);
            ruleIndex = goingNowhereGoal.AddRule("SayGoingNowhere");
            goingNowhereGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"I'm not going anywhere right now.\"");
            goingNowhereGoal.AddConsequent(ruleIndex, "Quit", "");
            goingNowhereGoal.Commit();

            GoalBuilder dontKnowGoal = new GoalBuilder("DontKnowHowToX", brain);
            ruleIndex = dontKnowGoal.AddRule("DontKnowHowToX");
            dontKnowGoal.AddAndAntecedent(ruleIndex, "DontKnow Arg *=*;1");
            dontKnowGoal.AddConsequent(ruleIndex, "Execs", "Voice Say \"Sorry, but I don't know how to $DontKnow_\"");
            dontKnowGoal.AddConsequent(ruleIndex, "Remove", "DontKnow Arg");
            dontKnowGoal.AddConsequent(ruleIndex, "Quit", "");
            dontKnowGoal.Commit();

            // Set up the ROS interface
            RosNode.RosInit("SubSimNL");
            node = new RosNode();
            pipelineClient = new ServiceClient(node, "upenn_nlp_pipeline_service");
            semantics = new SemanticsInterface();
        }

        /// <summary>
        /// Start up the NL input form.
        /// </summary>
        public override void Run()
        {
            // Show the window in a background thread
            // TODO: This may not be the best idea for the brain interface UI, but
            // it works for now.
            Thread uiThread = new Thread(new ThreadStart(ShowBackgroundForm));
            uiThread.IsBackground = false;
            uiThread.Name = "SumSimNL UI";
            uiThread.SetApartmentState(ApartmentState.STA);
            uiThread.Start();
        }

        /// <summary>
        /// Start up the NL input form and block on the UI.
        /// </summary>
        public void RunStandalone()
        {
            nlForm = new NLInputForm(this);
            nlForm.ShowDialog();
        }

        /// <summary>
        /// execs is not implemented.
        /// </summary>
        /// <param name="arg">Argument to execs</param>
        public override void execs(String arg)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// execa is not implemented.
        /// </summary>
        /// <param name="arg">Argument to execa</param>
        public override void execa(String arg)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Always return false when attending to facts, since we can't do that.
        /// </summary>
        /// <param name="fact">Fact to attend to</param>
        /// <returns>Always true</returns>
        public override bool AttendTo(CFact fact)
        {
            return false;
        }

        /// <summary>
        /// Get text from the hearing SubSim.
        /// </summary>
        /// <returns>Recognized text</returns>
        internal string Listen()
        {
            // Get recognized text from the brain
            CSubSimProcessor hearing = (CSubSimProcessor) brain.m_SubSimProcessors["hearing"];
            if (hearing == null)
            {
                return "Could not access hearing SubSim.";
            }

            hearing.execs("(Hearing Listen)");
            // Get WordsHeard
            CFact fact = brain.Mind.getFact("Arg", "WordsHeard");
            String recognizedText;
            if (fact != null)
            {
                recognizedText = (string)fact.Slots["Value"];
            }
            else 
            {
                recognizedText = "Listening did not return any words.";
            }

            return recognizedText;
        }

        /// <summary>
        /// Process text using the NLP pipeline.
        /// </summary>
        internal String Parse(String text)
        {
            return pipelineClient.call(text);
        }

        /// <summary>
        /// Process a parse using the semantics interface.
        /// </summary>
        internal SemanticsResponse AnalyzeSemantics(String parse, String text)
        {
            return semantics.Parse(parse, text);
        }

        /// <summary>
        /// Process a semantic representation, executing any goals it creates.
        /// </summary>
        internal void ProcessSemantics(SemanticsResponse semantics)
        {
            try
            {
                List<String> productions = new List<String>();

                // Get commands
                foreach (Command c in semantics.commands)
                {
                    String newProduction = GoalSemantics.ParseActionGoal(c, brain);
                    if (newProduction != null)
                    {
                        productions.Add(newProduction);
                    }
                }

                if (productions.Count == 0)
                {
                    // Respond that we didn't understand it
                    productions.Add(failureGoal.Name);
                }

                foreach (String productionName in productions)
                {
                    brain.RunGoal(productionName, false, true, true);
                }
            }
            catch (GoalEditException)
            {
                // Do nothing if we fail to add a new goal
            }
        }

        private void ShowBackgroundForm()
        {
            nlForm = new NLInputForm(this);
            nlForm.ShowDialog();
        }
    }
}