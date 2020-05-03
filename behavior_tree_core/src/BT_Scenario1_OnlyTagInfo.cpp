/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <behavior_tree.h>
#include <global_taginfo.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BT_Scenario1_OnlyTagInfo");
    try
    {
        int TickPeriod_milliseconds = 1000;

	BT::SequenceNodeWithMemory* sequence1 = new BT::SequenceNodeWithMemory("Seq1");

	BT::ConditionNodeEnterTargetAreas* condition1 = new BT::ConditionNodeEnterTargetAreas("Con1");
	BT::SequenceNodeWithMemory* sequence2 = new BT::SequenceNodeWithMemory("Seq2");
	BT::ConditionNodeStayStill* condition2 = new BT::ConditionNodeStayStill("Con2");
	BT::ActionNodePutObject* action1 = new BT::ActionNodePutObject("Act1");

	BT::FallbackNodeWithMemory* fallback1 = new BT::FallbackNodeWithMemory("Fal1");
	BT::FallbackNodeWithMemory* fallback2 = new BT::FallbackNodeWithMemory("Fal2");

	BT::ConditionNodeSmallBox* condition3 = new BT::ConditionNodeSmallBox("Con3");
	BT::ActionNodePickBigObject* action2 = new BT::ActionNodePickBigObject("Act2");
	BT::ConditionNodeBigBox* condition4 = new BT::ConditionNodeBigBox("Con4");
	BT::ActionNodePickSmallObject* action3 = new BT::ActionNodePickSmallObject("Act3");

	sequence1->AddChild(condition1);
	sequence1->AddChild(sequence2);
	sequence1->AddChild(condition2);
	sequence1->AddChild(action1);

	sequence2->AddChild(fallback1);
	sequence2->AddChild(fallback2);

	fallback1->AddChild(condition3);
	fallback1->AddChild(action2);

	fallback2->AddChild(condition4);
	fallback2->AddChild(action3);

        Execute(sequence1, TickPeriod_milliseconds);  // behavior_tree.cpp
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}


