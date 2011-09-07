using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Brain;
using SSRICSRobot;

namespace SubSimProcessorLanguage
{
    class GoalEditException : System.Exception
    {
        public GoalEditException(string message) : base(message) { }
    }

    /// <summary>
    /// Creates and edits production system goals.
    /// </summary>
    public class GoalBuilder
    {
        private string name;
        private CBrain brain;
        private CProduction goal;

        private List<CRule> rules;

        /// <summary>
        /// Make a new goal. The goal is not added to the brain until <c>Commit</c> is called.
        /// </summary>
        /// <param name="name">Name of the goal, the leading "g" is added automatically</param>
        /// <param name="brain">The brain to create the goal in when Commit is called.</param>
        public GoalBuilder(string name, CBrain brain)
        {
            this.brain = brain;
            rules = new List<CRule>();

            // Make a new non-decaying production
            goal = new CProduction();
            goal.decays = false;
            goal.Name = this.name = "g" + name;
        }

        /// <summary>
        /// The name of the rule as it is known to the Production System.
        /// </summary>
        public string Name
        {
            get { return name; }
        }

        /// <summary>
        /// Add the goal to the mind. Changes made after <c>Commit</c> are not guaranteed to be reflected
        /// in the mind.
        /// </summary>
        /// <returns>Whether the goal was added successfully</returns>
        public bool Commit()
        {
            return brain.Mind.addProduction(goal);
        }

        /// <summary>
        /// Add a new rule to the goal.
        /// </summary>
        /// <param name="name">The name of the goal. A leading "r" will be added.</param>
        /// <returns>A handle that can be used to refer to the new rule.</returns>
        public int AddRule(string name)
        {
            CRule rule = new CRule();
            rule.decays = false;
            rule.Name = "r" + name;

            // Store the rule and add it to the goal
            rules.Add(rule);
            goal.ruleSet.Add(rule);

            // Return the index we've just added, which will be final index, one below the count
            return rules.Count - 1;
        }

        /// <summary>
        /// Add a consequent to a rule.
        /// </summary>
        /// <param name="ruleIndex">The handle of the rule to modify</param>
        /// <param name="action">The action (e.g., execs) to perform</param>
        /// <param name="arg">The arguments for the action. Parentheses will be added.</param>
        public void AddConsequent(int ruleIndex, string action, string arg)
        {
            CConsequent consequent = new CConsequent(action, "(" + arg + ")");
            rules[ruleIndex].consequents.Add(consequent);
        }

        /// <summary>
        /// Add an AND antecedent to a rule.
        /// </summary>
        /// <param name="ruleIndex">The handle of the rule to modify</param>
        /// <param name="antecedent">The text of the antecedent. Parentheses will be added.</param>
        public void AddAndAntecedent(int ruleIndex, string antecedent)
        {
            CFact mem = new CFact("(" + antecedent + ")");
            rules[ruleIndex].andAntacedents.Add(mem);
        }

        /// <summary>
        /// Add a negated AND antecedent to a rule.
        /// </summary>
        /// <param name="ruleIndex">The handle of the rule to modify</param>
        /// <param name="antecedent">The text of the antecedent. Parentheses will be added.</param>
        public void AddNegatedAndAntecedent(int ruleIndex, string antecedent)
        {
            CFact mem = new CFact("(" + antecedent + ")");
            rules[ruleIndex].andAntacedents.Add(mem);
            rules[ruleIndex].operators[mem] = "true";
        }

        /// <summary>
        /// Add an OR antecedent to a rule.
        /// </summary>
        /// <param name="ruleIndex">The handle of the rule to modify</param>
        /// <param name="antecedent">The text of the antecedent. Parentheses will be added.</param>
        public void AddOrAntecedent(int ruleIndex, string antecedent)
        {
            CFact mem = new CFact("(" + antecedent + ")");
            rules[ruleIndex].orAntacedents.Add(mem);
        }

        /// <summary>
        /// Add a negated OR antecedent to a rule.
        /// </summary>
        /// <param name="ruleIndex">The handle of the rule to modify</param>
        /// <param name="antecedent">The text of the antecedent. Parentheses will be added.</param>
        public void AddNegatedOrAntecedent(int ruleIndex, string antecedent)
        {
            CFact mem = new CFact("(" + antecedent + ")");
            rules[ruleIndex].orAntacedents.Add(mem);
            rules[ruleIndex].operators[mem] = "true";
        }
    }
}
