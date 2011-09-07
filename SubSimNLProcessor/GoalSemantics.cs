/*
 *  GoalSemantics.cs - Create goals from semantic represenations.
 *  Copyright (C) 2011 Constantine Lignos
 *
 *  This file is part of SumSimNL.
 *
 *  SumSimNL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  SumSimNL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with SumSimNL.  If not, see <http://www.gnu.org/licenses/>.
 */

using System;
using System.Linq;
using System.Text;
using Brain;
using OpenNLP.Tools.Parser;

namespace SubSimProcessorLanguage
{
    /// <summary>
    /// Representation of transitive actions
    /// </summary>
    struct TransitiveAction
    {
        /// <summary>
        /// Action to perform
        /// </summary>
        public string action;
        /// <summary>
        /// Object to perform it on
        /// </summary>
        public string target;
    }

    /// <summary>
    /// Processes parses into semantic representations and goals.
    /// </summary>
    public class GoalSemantics
    {
        private static String[] conditionalWords = { "if", "when" };
        private static String[] referringWords = { "it" };
        private static String[] goWords = { "go", "move", "drive", "roll" };
        private static String[] goProgressives = { "going", "moving", "driving", "rolling" };
        private static String[] experienceWords = { "see", "find", "encounter", "notice" };
        private static String[] tellWords = { "tell", "call", "notify", "alert" };

        /// <summary>
        /// Process the parse as an unconditional action goal.
        /// </summary>
        /// <param name="parse">Parse to process</param>
        /// <param name="brain">Brain to add goals to</param>
        /// <returns>The name of the goal created; null if no goal could be made.</returns>
        public static string ParseActionGoal(Command command, CBrain brain)
        {
            // If there's no target, give up
            if (command.targets.Length == 0)
            {
                return null;
            }

            TransitiveAction antecedent = new TransitiveAction();
            TransitiveAction consequent = new TransitiveAction();
            consequent.action = command.action;
            // TODO Expand to handle multiple targets
            consequent.target = command.targets[0];
            return AddGoal(antecedent, consequent, brain);
        }

        /// <summary>
        /// Add a goal to the brain.
        /// </summary>
        /// <param name="antecedent">Antecedent of goal. If there is no antecendent, an empty TransitiveAction can be given.</param>
        /// <param name="consequent">Consequent of goal</param>
        /// <param name="brain">Brain to create goal in</param>
        /// <returns>Name of the goal created</returns>
        private static string AddGoal(TransitiveAction antecedent, TransitiveAction consequent, CBrain brain)
        {
            // Simple validation: if the consequent has a null action, give up
            if (consequent.action == null)
            {
                return null;
            }

            // Set up the goal
            string name = MakeGoalName(antecedent, consequent);
            GoalBuilder goal = new GoalBuilder("NL" + name, brain);
            int ruleIndex = goal.AddRule("NL" + name);

            // Convert consequents to goals
            if (goWords.Contains(consequent.action))
            {
                brain.Mind.addFact(new CFact("(CurrentDestination Arg Value=" + consequent.target + ";1)"));
                return "gGotoX";
            }
            else if (tellWords.Contains(consequent.action))
            {
                if (experienceWords.Contains(antecedent.action))
                {
                    string type = TitleCase(antecedent.target);
                    string matchVar = "NotToldAbout" + type;
                    goal.AddAndAntecedent(ruleIndex, matchVar + "? " + type + " *=*;1");
                    goal.AddNegatedAndAntecedent(ruleIndex, "?" + matchVar + " " + type + " ToldAbout=*;1 *=*;1");
                    goal.AddAndAntecedent(ruleIndex, "CurrentView State " + type + "s=*?" + matchVar + "_*;1 *=*;1");
                    goal.AddConsequent(ruleIndex, "Execs", "Voice Say \"Commander, I see a " + type + ".\"");
                    goal.AddConsequent(ruleIndex, "Set", "?" + matchVar + ".Name " + type + " ToldAbout=true");
                    // Add a default Wait rule so the rule doesn't spin
                    int defaultRuleIndex = goal.AddRule("Default" + goal.Name);
                    goal.AddConsequent(defaultRuleIndex, "Wait", "1000");
                    // Confirm the plan back
                    int confirmRuleIndex = goal.AddRule("Initialize" + goal.Name);
                    goal.AddConsequent(confirmRuleIndex, "Execs", "Voice Say \"Okay, if I see a " +
                        antecedent.target + ", I'll let you know.\"");
                    goal.AddConsequent(confirmRuleIndex, "DisableRule", "self");
                    // This rule doesn't have a quit as it should stay standing
                }
                else
                {
                    // Say we don't know how to do it
                    brain.Mind.addFact(new CFact("(DontKnow Arg Value=" + antecedent.action + ";1)"));
                    return "gDontKnowHowToX";
                }
            }
            else if (consequent.action != null)
            {
                // Say we don't know how to do it
                brain.Mind.addFact(new CFact("(DontKnow Arg Value=" + consequent.action + ";1)"));
                return "gDontKnowHowToX";
            }
            else
            {
                return null;
            }

            // Commit the goal if we made it through
            goal.Commit();
            return goal.Name;
        }

        /// <summary>
        /// Make a properly cased goal name.
        /// </summary>
        /// <param name="antecedent">Antecedent</param>
        /// <param name="consequent">Consequent</param>
        /// <returns>Goal name</returns>
        private static string MakeGoalName(TransitiveAction antecedent, TransitiveAction consequent)
        {
            StringBuilder name = new StringBuilder();
            if (antecedent.action != null) { name.Append(TitleCase(antecedent.action)); }
            if (antecedent.target != null) { name.Append(TitleCase(antecedent.target)); }
            if (consequent.action != null) { name.Append(TitleCase(consequent.action)); }
            if (consequent.target != null) { name.Append(TitleCase(consequent.target)); }
            return name.ToString();
        }

        /// <summary>
        /// Convert a string to Titlecase.
        /// </summary>
        /// <param name="str">String to convert</param>
        /// <returns>Titlecased string</returns>
        private static string TitleCase(string str)
        {
            // Uppercase the first letter, lowercase the rest
            StringBuilder s = new StringBuilder(str.Length);
            s.Append(str[0].ToString().ToUpper());
            s.Append(str.Substring(1).ToLower());
            return s.ToString();
        }
    }
}
