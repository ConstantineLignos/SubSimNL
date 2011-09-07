using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using IronPython.Hosting;
using Microsoft.Scripting.Hosting;
using Microsoft.Scripting;
using IronPython.Runtime;

namespace SubSimProcessorLanguage
{
    public class SemanticsInterface
    {
        private ScriptEngine pyEng;
        private ScriptScope scope;
        private ScriptSource semanticsInterface;
        private object processTreeFunc;

        public SemanticsInterface()
        {
            // Create engine and scope
            pyEng = IronPython.Hosting.Python.CreateEngine();
            pyEng.SetSearchPaths(new List<String>(new String[] { @"IronPython 2.6\Lib", @"PennNLP\nlpy" }));
            scope = pyEng.CreateScope();

            // Load interface
            semanticsInterface = pyEng.CreateScriptSourceFromFile(@"semantics_interface.py");
            semanticsInterface.Execute(scope);
            processTreeFunc = scope.GetVariable("process_tree");
        }

        public SemanticsResponse Parse(string tree, string text)
        {
            PythonTuple result = (PythonTuple) pyEng.Operations.Invoke(processTreeFunc, new Object[] { tree, text });
            SemanticsResponse sr = ParseSemanticsResult(result);
            return sr;
        }

        private static SemanticsResponse ParseSemanticsResult(PythonTuple result)
        {
            // Convert the commands into C# types
            List<Command> commands = new List<Command>();
            List<String> verbs = new List<string>();
            List<List<String>> targets = new List<List<String>>();

            // First parse the verbs and targets out
            foreach (PythonTuple command in (List) result[1]) 
            {
                verbs.Add((String) command[0]);
                targets.Add(ConvertList<String>((List) command[1]));
            }

            // Then combine them into commands
            for (int i = 0; i < verbs.Count; i++)
            {
                commands.Add(new Command(verbs[i], targets[i].ToArray()));
            } 

            // Make a response
            SemanticsResponse sr = new SemanticsResponse((String) result[0], commands.ToArray());

            return sr;
        }

        private static List<T> ConvertList<T>(List list)
        {
            List<T> convertedList = new List<T>();
            foreach (T item in list)
            {
                convertedList.Add(item);
            }
            return convertedList;
        }
    }

    public class Command
    {
        public String action;
        public String[] targets;

        public Command(String action, String[] targets)
        {
            this.action = action;
            this.targets = targets;
        }

        public override String ToString()
        {
            StringBuilder targetString = new StringBuilder();
            foreach (String t in targets)
            {
                targetString.Append(t);
            }
            return this.action + ": " + targetString.ToString();
        }
    }

    public class SemanticsResponse
    {
        public String answer;
        public Command[] commands;

        public SemanticsResponse(String answer, Command[] commands)
        {
            this.answer = answer;
            this.commands = commands;
        }

        public override string ToString()
        {
            StringBuilder output = new StringBuilder();
            output.AppendLine("Commands:");
            foreach (Command c in commands)
            {
                output.AppendLine(c.ToString());
            }
            output.AppendLine();
            output.AppendLine(answer);
            return output.ToString();
        }
    }
}
