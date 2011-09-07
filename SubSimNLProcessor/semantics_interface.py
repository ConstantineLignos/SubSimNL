
#  semantics_interface.py - Provides an interface to semantics functions.
#  Copyright (C) 2011 Constantine Lignos
#
#  This file is part of SumSimNL.
#
#  SumSimNL is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  SumSimNL is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with SumSimNL.  If not, see <http://www.gnu.org/licenses/>.

import sys

from Semantics import Knowledge, Parsing, Tree

# World knowledge object
KNOWLEDGE = Knowledge.Knowledge()


def process_tree(parsed_text, text=''):
    """Return a string summary of the semantic representations after parsing text."""
    (semantic_answer, frame_trees) = KNOWLEDGE.process_parse_tree(parsed_text, text)
    results = []
    
    if frame_trees is not None:
        #modified_trees = [str(modified_parse_tree[1]) for modified_parse_tree in frame_trees if \
        #                    len(modified_parse_tree) > 1 and \
        #                    isinstance(modified_parse_tree[1], Tree.Tree)]
        frames =  [str(frame_dict) + "\r\n" for frame_dict in [frame[0] for frame in frame_trees \
                                                        if not isinstance(frame,str)]] 
        #results.append("Modified trees: " + "\r\n".join(modified_trees))
        results.append("Frames:\r\n" + "".join(frames))

    # TODO: Turn answers back on
    #if semantic_answer:
    #    results.append("Answer: " + str(semantic_answer))
    
    # TODO: Turn watch list back on
    #if KNOWLEDGE.watch_list:
    #    results.append("Watch list: " + str(KNOWLEDGE.watch_list))
        
    if KNOWLEDGE.commander_known_entities:
        results.append("Commander known entities: " + str(KNOWLEDGE.commander_known_entities))
        
    if KNOWLEDGE.current_commands:
        results.append("Commands generated: " + str(KNOWLEDGE.current_commands))
     
    return ("\r\n\r\n".join(results) if results else "No semantic information.", KNOWLEDGE.current_commands)
    
    
def test():
    parse_tree_strings = [
        "(S (NP-SBJ-A (NNP Junior)) (, ,) (VP (VP-A (VBP drive)) (PP-CLR (TO to) (NP-A (DT the) (NN hallway)))) (. .))",
        "(S (CC And) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A-0 (PRP you)) (VP (VBP get) (VP-A (VBN flipped) (NP-A-0 (-NONE- *))))))(, ,) (NP-SBJ-A (-NONE- *)) (VP (VB call) (NP-A (PRP me)))(. .))",
        "(S(NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A  (NP (DT a) (JJ bad) (NN guy))(CC or) (NP (DT a) (NN hostage)))))))(. .))",
        "(S (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me))) (. .))",
        "((S (NP-SBJ-A (-NONE- *)) (VP (VB Go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))) ",
        "((S (NP-SBJ-A (PRP I)) (VP (VBP am) (PP-LOC-PRD (IN in) (NP-A (NN room) (CD 2)))) (. .)))",
        "(S (NP-SBJ-A (EX There)) (VP (VBP are) (NP-PRD-A (NP (CD two) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 3))))) (. .))",
        "((S (NP-SBJ-A (EX There)) (VP (VBP are) (NP-PRD-A (NP (QP (IN at) (JJS least) (CD 4)) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 1))))) (. .)))  ",
        "((SBARQ (WHADVP-0 (WRB Where)) (SQ (VP (VBP are) (NP-PRD-A (DT the) (NNS hostages)) (ADVP-0 (-NONE- *T*)))) (. ?)))",
        "((SBARQ (WHNP-0 (WP What)) (SQ (VBP are) (NP-SBJ (PRP you)) (VP (VBG doing) (NP-0 (-NONE- *T*)))) (. ?)))",
        "((SINV (VP (VBP Are) (ADVP-LOC-PRD (RB there))) (NP-SBJ (DT any) (NNS hostages)) (. ?))) ",
        "(S  (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A (DT a) (NN hostage))))))(. .))",
        "(S  (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A  (NP (DT a) (NN bomb))(CC or) (NP (DT a) (NN hostage)))))))(. .))",
        "((S (NP-SBJ-A (NNP Junior)) (, ,) (VP (VP-A (VB go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))))"]
    for tree in parse_tree_strings:
        print '*'*40
        print tree
        print process_tree(tree)
        print
        
    raw_input()

        
if __name__ == "__main__":
    test()