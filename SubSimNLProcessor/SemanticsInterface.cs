/*
 *  SemanticsInterface.cs - Provides an interface to the Python semantics interface.
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
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.IO;
using rosin;

namespace SubSimProcessorLanguage
{
    public class SemanticsInterface
    {
        private RosNode node;
        private ServiceClient semanticsClient;
        private DataContractJsonSerializer requestSerializer;
        private DataContractJsonSerializer responseSerializer;

        public SemanticsInterface()
        {
            node = new RosNode();
            semanticsClient = new ServiceClient(node, "upenn_nlp_semantics_service");
            requestSerializer = new DataContractJsonSerializer(typeof(SemanticsServiceRequest));
            responseSerializer = new DataContractJsonSerializer(typeof(SemanticsServiceResponse));
        }

        public SemanticsResponse Parse(String tree, String text)
        {
            // Serialize the request and call
            SemanticsServiceRequest request = new SemanticsServiceRequest();
            request.tree = tree;
            request.text = text;
            MemoryStream requestStream = new MemoryStream();
            requestSerializer.WriteObject(requestStream, request);
            String jsonResult = semanticsClient.call(Encoding.ASCII.GetString(requestStream.ToArray()));

            // Deserialize the response
            MemoryStream responseStream = new MemoryStream(Encoding.ASCII.GetBytes(jsonResult));
            responseStream.Position = 0;
            SemanticsServiceResponse response = (SemanticsServiceResponse) responseSerializer.ReadObject(responseStream);

            SemanticsResponse sr = ParseSemanticsResult(response);
            return sr;
        }

        private static SemanticsResponse ParseSemanticsResult(SemanticsServiceResponse result)
        {
            // Convert the commands into C# types
            List<Command> commands = new List<Command>();
            List<String> verbs = new List<string>();
            List<String> targets = new List<String>();

            // First parse the verbs and targets out
            foreach (ComplexCommand command in result.commands) 
            {
                commands.Add(new Command(command.command, command.location));

            }

            // Make a response
            SemanticsResponse sr = new SemanticsResponse(result.userResponse, commands.ToArray());
            return sr;
        }
    }

    public class Command
    {
        public String action;
        public String target;

        public Command(String action, String target)
        {
            this.action = action;
            this.target = target;
        }

        public override String ToString()
        {

            return action + ": " + target;
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
            output.AppendLine("Response:");
            output.AppendLine(answer);
            return output.ToString();
        }
    }

    public class SemanticsServiceRequest
    {
        public String tree { get; set; }
        public String text { get; set; }
    }

    [DataContract]
    public class SemanticsServiceResponse
    {

        [DataMember(Name = "user_response")]
        public String userResponse { get; set; }
        [DataMember(Name = "new_commands")]
        public List<ComplexCommand> commands { get; set; }
    }

    [DataContract]
    public class ComplexCommand
    {
        // TODO: Remove location hack
        [DataMember(Name = "Command")]
        public String command { get; set; }
        [DataMember(Name = "Location")]
        public String location { get; set; }
    }
}
