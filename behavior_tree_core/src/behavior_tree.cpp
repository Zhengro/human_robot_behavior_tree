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
#include <dot_bt.h>
#include <ros/ros.h>
#include <global_info.h>


void Execute(BT::ControlNode* root, int TickPeriod_milliseconds)
{
    std::cout << "Start Drawing!" << std::endl;
    // Starts in another thread the drawing of the BT
    std::thread t2(&drawTree, root);
    t2.detach();
    BT::DotBt dotbt(root);
    std::thread t(&BT::DotBt::publish, dotbt);

    root->ResetColorState();				// control_node.cpp

    while (ros::ok())
    {
        DEBUG_STDOUT("Ticking the root node!");

        // Ticking the root node
        root->Tick();					// sequence_node.cpp / ...
        // Printing its state
        // root->GetNodeState();

	BT::ReturnStatus root_status_ = root->get_status();
        if (root_status_ != BT::RUNNING)
        {
            // when the root returns a status it resets the colors of the tree
            root->ResetColorState();
        }

	if (root_status_ == BT::SUCCESS)
        {
            DEBUG_STDOUT("Root returning Success " << BT::SUCCESS << "!\n" << "Exit this behavior tree!");
	    break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
    }
}
