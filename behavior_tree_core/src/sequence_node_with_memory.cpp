
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


#include <sequence_node_with_memory.h>
#include <string>


BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string name) : ControlNode::ControlNode(name)
{
    reset_policy_ = BT::ON_SUCCESS;  
    // default: memory of a sequence node will only be reset when all its children return SUCCESS, 
    // i.e., when one child is running or failed, it won't reset the memory; when the sequence node is ticked again, it will tick from that child (as indicated by the memory).
    current_child_idx_ = 0;  // initialize the current running child
}


BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string name, int reset_policy) : ControlNode::ControlNode(name)
{
    reset_policy_ = reset_policy;
    current_child_idx_ = 0;  // initialize the current running child
}


BT::SequenceNodeWithMemory::~SequenceNodeWithMemory() {}


BT::ReturnStatus BT::SequenceNodeWithMemory::Tick()
{
    DEBUG_STDOUT(get_name() << " ticked, memory counter: "<< current_child_idx_);

    // Vector size initialization. N_of_children_ could change at runtime if you edit the tree
    N_of_children_ = children_nodes_.size();

    // Routing the ticks according to the sequence node's (with memory) logic:
    while (current_child_idx_ < N_of_children_)
    {
        /*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
                We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
                Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
                For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
        */

        if (children_nodes_[current_child_idx_]->get_type() == BT::ACTION_NODE)
        {
            // 1) if the child i is an action, read its status
            // Action nodes runs in another thread, hence you cannot retrieve the status just by calling the method Tick().
            child_i_status_ = children_nodes_[current_child_idx_]->get_status();

            if (child_i_status_ == BT::IDLE || child_i_status_ == BT::HALTED)
            {
                // 1.1) if the action status is not RUNNING, the sequence node sends a tick to it
                DEBUG_STDOUT(get_name() << " NEEDS TO TICK " << children_nodes_[current_child_idx_]->get_name());
                children_nodes_[current_child_idx_]->tick_engine.Tick();

                // wait for the tick to arrive to the child
                do
                {
                    child_i_status_ = children_nodes_[current_child_idx_]->get_status();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while (child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS
                       && child_i_status_ != BT::FAILURE);
            }
        }
        else
        {
            // 2) if it's not an action, send the tick and wait for the response
            child_i_status_ = children_nodes_[current_child_idx_]->Tick();
        }

        if (child_i_status_ != BT::SUCCESS)
        {
            if (child_i_status_ == BT::FAILURE)
            {
                children_nodes_[current_child_idx_]->set_status(BT::IDLE);  // the child goes in idle if it has returned FAILURE
            }

	    // if the child status is not SUCCESS, return the status to its parent
            DEBUG_STDOUT("The status of " << get_name() << " becomes " << child_i_status_);
            if (child_i_status_ == BT::FAILURE && (reset_policy_ == BT::ON_FAILURE || reset_policy_ == BT::ON_SUCCESS_OR_FAILURE))
            {
                current_child_idx_ = 0;
            }
            set_status(child_i_status_);
            return child_i_status_;
        }
        else if (current_child_idx_ != N_of_children_ - 1)
        {
            // if it is not the last child and it has returned SUCCESS, continue to the next child
            current_child_idx_++;
        }
        else
        {
            // if it is the last child and it has returned SUCCESS, reset the memory and return the status to its parent
	    current_child_idx_ = 0;
            set_status(child_i_status_);
            return child_i_status_;
        }
    }
    return BT::EXIT;
}


int BT::SequenceNodeWithMemory::DrawType()
{
    return BT::SEQUENCESTAR;
}


void BT::SequenceNodeWithMemory::Halt()
{
    current_child_idx_ = 0;
    BT::ControlNode::Halt();
}
