/*
 *  ParseViewer.cs - Displays parses. Adapted from the SharpNLP ParseTree demo application.
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

using System.Drawing;
using Netron.Lithium;
using OpenNLP.Tools.Parser;
using OpenNLP.Tools.Util;

namespace SubSimProcessorLanguage
{
    /// <summary>
    /// Displays parse trees in a LithiumControl. 
    /// </summary>
    public class TreeViewer
    {
        private LithiumControl lithiumControl;

        /// <summary>
        /// Initializes a new instance of the TreeViewer class.
        /// </summary>
        /// <param name="control">Control to use for viewing.</param>
        public TreeViewer(LithiumControl control)
        {
            lithiumControl = control;
        }

        private void AddChildNodes(ShapeBase currentShape, Parse[] childParses)
        {
            foreach (Parse childParse in childParses)
            {
                // if this is not a token node (token node = one of the words of the sentence)
                if (childParse.Type != MaximumEntropyParser.TokenNode)
                {
                    ShapeBase childShape = currentShape.AddChild(childParse.Type);
                    if (childParse.IsPosTag)
                    {
                        childShape.ShapeColor = Color.DarkGoldenrod;
                    }
                    else
                    {
                        childShape.ShapeColor = Color.SteelBlue;
                    }
                    AddChildNodes(childShape, childParse.GetChildren());
                    childShape.Expand();
                }
                else
                {
                    Span parseSpan = childParse.Span;
                    string token = childParse.Text.Substring(parseSpan.Start, (parseSpan.End) - (parseSpan.Start));
                    ShapeBase childShape = currentShape.AddChild(token);
                    childShape.ShapeColor = Color.Ivory;
                }
            }
        }

        /// <summary>
        /// Shows the parse in the LithiumControl.
        /// </summary>
        /// <param name="parse">The parse to display</param>
        public void ShowParse(Parse parse)
        {
            lithiumControl.NewDiagram();

            if (parse.Type == MaximumEntropyParser.TopNode)
            {
                parse = parse.GetChildren()[0];
            }

            // Display the parse result
            ShapeBase root = this.lithiumControl.Root;
            root.Text = parse.Type;
            root.Visible = true;

            AddChildNodes(root, parse.GetChildren());
            root.Expand();

            this.lithiumControl.DrawTree();
        }
    }
}
