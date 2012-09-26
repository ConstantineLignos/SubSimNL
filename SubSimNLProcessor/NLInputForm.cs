/*
 *  NLInputForm.cs - Form for accepting natural language input.
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
using System.Windows.Forms;
using OpenNLP.Tools.Parser;

namespace SubSimProcessorLanguage
{
    public partial class NLInputForm : Form
    {
        private CSubSimProcessorLanguage nlProcessor;
        private TreeViewer viewer;

        /// <summary>
        /// Create a form that uses the given natural language SubSim.
        /// </summary>
        /// <param name="nlProcessor">The SubSim to use</param>
        public NLInputForm(CSubSimProcessorLanguage nlProcessor)
        {
            InitializeComponent();
            this.nlProcessor = nlProcessor;
            viewer = new TreeViewer(parseTreeControl);
        }

        private void processButton_Click(object sender, EventArgs e)
        {
            // Block the UI
            inputTextBox.Enabled = false;
            processButton.Enabled = false;
            this.Cursor = Cursors.WaitCursor;

            // Get the input text and call the parser on it
            string inputText = inputTextBox.Text;
            string inputParse = nlProcessor.parse(inputText);
            Parse convertedInputParse = Parse.FromParseString(inputParse); 

            // Show the parse
            viewer.ShowParse(convertedInputParse);

            // Get and write out the semantics
            SemanticsResponse semantics = nlProcessor.ParseSemantics(inputParse, inputText);
            semanticsTextBox.Text = semantics.ToString();

            // Create goals from the semantics
            nlProcessor.ProcessSemantics(semantics);

            // Restore the UI
            inputTextBox.Enabled = true;
            processButton.Enabled = true;
            this.Cursor = Cursors.Default;
        }

        private void listenButton_Click(object sender, EventArgs e)
        {
            // Get text from the hearing system 
            inputTextBox.Text = nlProcessor.Listen();
        }
    }
}