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
using System.Text;
using System.Threading;
using System.Windows.Forms;
using Brain;
//using OpenNLP.Tools.Parser;
//using OpenNLP.Tools.Tokenize;
//using OpenNLP.Tools.PosTagger;
//using danbikel.parser;
//using edu.upenn.cis.emptycategories;
using SSRICSRobot;

namespace SubSimProcessorLanguage
{
    /// <summary>
    /// A SubSim processor that provides natural language input capabilities.
    /// </summary>
    public class CSubSimProcessorLanguage : CSubSimProcessor
    {
        private static String sharpNLPPath = @"SharpNLP\";
        private static String pennNLPPath = @"PennNLP\";
        //private Parser dbParser;
        static AutoResetEvent parserLoaded;
        static AutoResetEvent restorerLoaded;

        /// <summary>
        /// Whether we should run a standalone UI. Set to true if we are running without the BrainInterface.
        /// </summary>
        public Boolean standalone;

        //private EnglishMaximumEntropyTokenizer tokenizer;
        //private EnglishMaximumEntropyPosTagger tagger;

        private CBrain brain;
        private IRobot robot;
        private GoalBuilder failureGoal;
        private SemanticsInterface semantics;

        /// <summary>
        /// Initializes a new instance of the CSubSimProcessorLanguage class.
        /// </summary>
        /// <param name="brain">The brain the SubSim is operating in</param>
        public CSubSimProcessorLanguage(CBrain brain)
            : base(brain)
        {
            standalone = false;
        }

        /// <summary>
        /// Create a parser and default goals.
        /// </summary>
        /// <param name="brain">The brain</param>
        /// <param name="robot">The robot</param>
        public override void Init(Brain.CBrain brain, IRobot robot)
        {
            this.brain = brain;
            this.robot = robot;

            // Load external NLP systems in their own threads
            restorerLoaded = new AutoResetEvent(false);
            parserLoaded = new AutoResetEvent(false);
            Thread parserLoader = new Thread(InitParser);
            Thread restorerLoader = new Thread(InitRestorer);
            parserLoader.Start();
            restorerLoader.Start();

            // Load up other systems in the meantime
            //this.tokenizer = null; new EnglishMaximumEntropyTokenizer(sharpNLPPath + "EnglishTok.nbin");
            //this.tagger = null; new EnglishMaximumEntropyPosTagger(sharpNLPPath + "EnglishPOS.nbin", sharpNLPPath + @"\Parser\tagdict");

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

            // Wait for all systems to finish loading
            restorerLoaded.WaitOne();
            parserLoaded.WaitOne();
            this.semantics = new SemanticsInterface();
        }

        /// <summary>
        /// Start up the NL input form.
        /// </summary>
        public override void Run()
        {
            // Open the form, using a blocking call if we are a standalone
            Form nlForm = new NLInputForm(this);
            if (standalone)
            {
                nlForm.ShowDialog();
            }
            else
            {
                nlForm.Show();
            }
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
        /// Always return true.
        /// </summary>
        /// <param name="fact">Fact to attend to</param>
        /// <returns>Always true</returns>
        public override bool AttendTo(CFact fact)
        {
            return false;
        }

        internal SemanticsResponse ParseSemantics(string inputParse, string inputText)
        {
            return semantics.Parse(inputParse, inputText);
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

        private void InitParser()
        {
            /*
            dbParser = Parser.InitParser(new String[] {"-sf", pennNLPPath + @"dbparser\eatb3.properties", "-is", 
                pennNLPPath + @"dbparser\wsjall.obj",  "-sa", "-", "-out", "-"});

            if (dbParser == null)
            {
                // TODO Investigate this further
                // Try it again until it works!
                // This is to work around a seemingly random bug on loading, that has never happened
                // twice in a row
                InitParser();
            }
            else
            {
                parserLoaded.Set();
            }
             */
        }

        private void InitRestorer()
        {
            /*
            RestoreECs.init(new String[] {"run", "--", "--perceptron", "--ante_perceptron", "--nptrace", "--whxp",
                "--wh", "--whxpdiscern", "--nptraceante", "--noante", "--base:" + pennNLPPath + @"addnulls\"});
            restorerLoaded.Set();
             */
        }

        internal String parse(string inputText)
        {
            //String[] tokens = tokenizer.Tokenize(inputText);
            //String[] tags = tagger.Tag(tokens);
            //String sent = CombineTags(tokens, tags);
            String parse = "(S (NP (NN Test)))";
            String restoredParse = "(S (NP (NN Test)))";  //RestoreECs.processParse(parse);
            return restoredParse;
        }

        private static String CombineTags(String[] tokens, String[] tags)
        {
            StringBuilder combined = new StringBuilder("(");
            for (int i = 0; i < tokens.Length; i++)
            {
                String tok = tokens[i];
                String tag = tags[i];
                combined.Append("(" + tok + "(" + tag + "))");
            }
            combined.Append(")");
            return combined.ToString();
        }

        /// <summary>
        /// Process a semantic representation, executing any goals it creates.
        /// </summary>
        /// <param name="inputParse">Parse to process</param>
        /// <param name="inputText">Text that generated the parse</param>
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
                else
                {
                    foreach (String productionName in productions)
                    {
                        // Execute the goal by running it in a new production system in its own thread
                        CProductionSystem prodSys = new CProductionSystem(brain, productionName);
                        brain.ProductionSystemsOutput += prodSys.ProductionSystemOutput;
                        Thread productionThread = new Thread(new ThreadStart(prodSys.Process));
                        productionThread.Priority = ThreadPriority.BelowNormal;
                        productionThread.Name = "NL SubSim Goal " + productionName;
                        productionThread.IsBackground = true;
                        productionThread.Start();
                    }
                }
            }
            catch (GoalEditException)
            {
                // Do nothing if we fail to add a new goal
            }
        }
    }
}