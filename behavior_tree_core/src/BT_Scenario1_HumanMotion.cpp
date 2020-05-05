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
#include <global_info.h>
#include <info_subscriber.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BT_Scenario1_HumanMotion");
    try
    {
        int TickPeriod_milliseconds = 1000;

	// Start the subscribers in another thread
	BT::InfoSubscriber info_sub;
	std::thread t_sub(&BT::InfoSubscriber::subscribe, info_sub);

	// Build the behavior tree
	BT::SequenceNodeWithMemory* sequence1 = new BT::SequenceNodeWithMemory("Seq1");

	BT::ConditionNodePredictSteadyBoxSize* condition1 = new BT::ConditionNodePredictSteadyBoxSize("Con1");
	BT::SequenceNodeWithMemory* sequence2 = new BT::SequenceNodeWithMemory("Seq2");
	BT::ConditionNodePredictSteadyBoxPosition* condition2 = new BT::ConditionNodePredictSteadyBoxPosition("Con2");
	BT::SequenceNodeWithMemory* sequence3 = new BT::SequenceNodeWithMemory("Seq3");
	BT::ConditionNodeStayStill* condition3 = new BT::ConditionNodeStayStill("Con3");
	BT::ActionNodePutObject* action1 = new BT::ActionNodePutObject("Act1");

	BT::FallbackNodeWithMemory* fallback1 = new BT::FallbackNodeWithMemory("Fal1");
	BT::FallbackNodeWithMemory* fallback2 = new BT::FallbackNodeWithMemory("Fal2");
	BT::FallbackNodeWithMemory* fallback3 = new BT::FallbackNodeWithMemory("Fal3");	
	BT::FallbackNodeWithMemory* fallback4 = new BT::FallbackNodeWithMemory("Fal4");

	BT::ConditionNodePredictSmallBox* condition4 = new BT::ConditionNodePredictSmallBox("Con4");
	BT::ActionNodePickBigObject* action2 = new BT::ActionNodePickBigObject("Act2");
	BT::ConditionNodePredictBigBox* condition5 = new BT::ConditionNodePredictBigBox("Con5");
	BT::ActionNodePickSmallObject* action3 = new BT::ActionNodePickSmallObject("Act3");

	BT::ConditionNodePredictBoxOnLeft* condition6 = new BT::ConditionNodePredictBoxOnLeft("Con6");
	BT::ActionNodeRightReadyPose* action4 = new BT::ActionNodeRightReadyPose("Act4");
	BT::ConditionNodePredictBoxOnRight* condition7 = new BT::ConditionNodePredictBoxOnRight("Con7");
	BT::ActionNodeLeftReadyPose* action5 = new BT::ActionNodeLeftReadyPose("Act5");

	sequence1->AddChild(condition1);
	sequence1->AddChild(sequence2);
	sequence1->AddChild(condition2);
	sequence1->AddChild(sequence3);
	sequence1->AddChild(condition3);
	sequence1->AddChild(action1);

	sequence2->AddChild(fallback1);
	sequence2->AddChild(fallback2);
	sequence3->AddChild(fallback3);
	sequence3->AddChild(fallback4);

	fallback1->AddChild(condition4);
	fallback1->AddChild(action2);

	fallback2->AddChild(condition5);
	fallback2->AddChild(action3);

	fallback3->AddChild(condition6);
	fallback3->AddChild(action4);

	fallback4->AddChild(condition7);
	fallback4->AddChild(action5);

        Execute(sequence1, TickPeriod_milliseconds);  // behavior_tree.cpp
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}


