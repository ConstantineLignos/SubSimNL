namespace SubSimProcessorLanguage
{
    /// <summary>
    /// Form for accepting natural language input.
    /// </summary>
    partial class NLInputForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.inputTextBox = new System.Windows.Forms.TextBox();
            this.processButton = new System.Windows.Forms.Button();
            this.outputGroupBox = new System.Windows.Forms.GroupBox();
            this.parseTreeControl = new Netron.Lithium.LithiumControl();
            this.inputGroupBox = new System.Windows.Forms.GroupBox();
            this.listenButton = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.semanticsTextBox = new System.Windows.Forms.TextBox();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.outputGroupBox.SuspendLayout();
            this.inputGroupBox.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.SuspendLayout();
            // 
            // inputTextBox
            // 
            this.inputTextBox.Dock = System.Windows.Forms.DockStyle.Top;
            this.inputTextBox.Location = new System.Drawing.Point(3, 16);
            this.inputTextBox.Multiline = true;
            this.inputTextBox.Name = "inputTextBox";
            this.inputTextBox.Size = new System.Drawing.Size(376, 54);
            this.inputTextBox.TabIndex = 0;
            // 
            // processButton
            // 
            this.processButton.Location = new System.Drawing.Point(5, 79);
            this.processButton.Name = "processButton";
            this.processButton.Size = new System.Drawing.Size(75, 23);
            this.processButton.TabIndex = 1;
            this.processButton.Text = "Process";
            this.processButton.UseVisualStyleBackColor = true;
            this.processButton.Click += new System.EventHandler(this.processButton_Click);
            // 
            // outputGroupBox
            // 
            this.outputGroupBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.outputGroupBox.Controls.Add(this.parseTreeControl);
            this.outputGroupBox.Location = new System.Drawing.Point(0, 119);
            this.outputGroupBox.Name = "outputGroupBox";
            this.outputGroupBox.Size = new System.Drawing.Size(382, 337);
            this.outputGroupBox.TabIndex = 2;
            this.outputGroupBox.TabStop = false;
            this.outputGroupBox.Text = "Parser Output";
            // 
            // parseTreeControl
            // 
            this.parseTreeControl.AutoScroll = true;
            this.parseTreeControl.AutoScrollMinSize = new System.Drawing.Size(-2147483547, -2147483547);
            this.parseTreeControl.BackColor = System.Drawing.SystemColors.Control;
            this.parseTreeControl.BranchHeight = 70;
            this.parseTreeControl.ConnectionType = Netron.Lithium.ConnectionType.Default;
            this.parseTreeControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.parseTreeControl.LayoutDirection = Netron.Lithium.TreeDirection.Vertical;
            this.parseTreeControl.LayoutEnabled = true;
            this.parseTreeControl.Location = new System.Drawing.Point(3, 16);
            this.parseTreeControl.Name = "parseTreeControl";
            this.parseTreeControl.Size = new System.Drawing.Size(376, 318);
            this.parseTreeControl.TabIndex = 1;
            this.parseTreeControl.WordSpacing = 20;
            // 
            // inputGroupBox
            // 
            this.inputGroupBox.Controls.Add(this.listenButton);
            this.inputGroupBox.Controls.Add(this.inputTextBox);
            this.inputGroupBox.Controls.Add(this.processButton);
            this.inputGroupBox.Dock = System.Windows.Forms.DockStyle.Top;
            this.inputGroupBox.Location = new System.Drawing.Point(0, 0);
            this.inputGroupBox.Name = "inputGroupBox";
            this.inputGroupBox.Size = new System.Drawing.Size(382, 113);
            this.inputGroupBox.TabIndex = 3;
            this.inputGroupBox.TabStop = false;
            this.inputGroupBox.Text = "Input";
            // 
            // listenButton
            // 
            this.listenButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.listenButton.Location = new System.Drawing.Point(301, 79);
            this.listenButton.Name = "listenButton";
            this.listenButton.Size = new System.Drawing.Size(75, 23);
            this.listenButton.TabIndex = 2;
            this.listenButton.Text = "Listen";
            this.listenButton.UseVisualStyleBackColor = true;
            this.listenButton.Click += new System.EventHandler(this.listenButton_Click);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.semanticsTextBox);
            this.groupBox1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.groupBox1.Location = new System.Drawing.Point(0, 0);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(420, 456);
            this.groupBox1.TabIndex = 3;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Semantics Output";
            // 
            // semanticsTextBox
            // 
            this.semanticsTextBox.BackColor = System.Drawing.SystemColors.Control;
            this.semanticsTextBox.Dock = System.Windows.Forms.DockStyle.Fill;
            this.semanticsTextBox.Location = new System.Drawing.Point(3, 16);
            this.semanticsTextBox.Multiline = true;
            this.semanticsTextBox.Name = "semanticsTextBox";
            this.semanticsTextBox.ReadOnly = true;
            this.semanticsTextBox.Size = new System.Drawing.Size(414, 437);
            this.semanticsTextBox.TabIndex = 0;
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(0, 0);
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.inputGroupBox);
            this.splitContainer1.Panel1.Controls.Add(this.outputGroupBox);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.groupBox1);
            this.splitContainer1.Size = new System.Drawing.Size(806, 456);
            this.splitContainer1.SplitterDistance = 382;
            this.splitContainer1.TabIndex = 4;
            // 
            // NLInputForm
            // 
            this.AcceptButton = this.processButton;
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(806, 456);
            this.Controls.Add(this.splitContainer1);
            this.Name = "NLInputForm";
            this.Text = "Natural Language Input";
            this.outputGroupBox.ResumeLayout(false);
            this.inputGroupBox.ResumeLayout(false);
            this.inputGroupBox.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            this.splitContainer1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TextBox inputTextBox;
        private System.Windows.Forms.Button processButton;
        private System.Windows.Forms.GroupBox outputGroupBox;
        private System.Windows.Forms.GroupBox inputGroupBox;
        private Netron.Lithium.LithiumControl parseTreeControl;
        private System.Windows.Forms.Button listenButton;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox semanticsTextBox;
        private System.Windows.Forms.SplitContainer splitContainer1;
    }
}

