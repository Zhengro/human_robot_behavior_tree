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


#include <actions/action_node_pick_big_object.h>
#include <string>


BT::ActionNodePickBigObject::ActionNodePickBigObject(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = true;
    // time_ = 1;
    thread_ = std::thread(&ActionNodePickBigObject::WaitForTick, this);
}

BT::ActionNodePickBigObject::~ActionNodePickBigObject() {}

void BT::ActionNodePickBigObject::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();				// the action node will be ticked by the control node and then executed in another thread
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        // Perform action...
        if (get_status() != BT::HALTED)
        {
            DEBUG_STDOUT(get_name() << " Running! Thread id: " << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(2));
	    // To Do: Pick Big Object
	    // Call a function to enable robot to pick a big object, if success, boolean_value_ = true; otherwise boolean_value_ = false.
        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(get_name() << " Success!");
            }
            else
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(get_name() << " Failure!");
            }
        }
    }
}

void BT::ActionNodePickBigObject::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionNodePickBigObject::set_time(int time)
{
    time_ = time;
}


void BT::ActionNodePickBigObject::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}


