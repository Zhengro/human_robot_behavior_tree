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


#include <conditions/condition_node_stay_still.h>
#include <string>

BT::ConditionNodeStayStill::ConditionNodeStayStill(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionNodeStayStill::~ConditionNodeStayStill() {}

BT::ReturnStatus BT::ConditionNodeStayStill::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

	// To Do: implement a function to subscribe the tag info, if it stays still, set boolean_value_ = true, otherwise boolean_value_ = false.

        // Condition checking and state update
        if (boolean_value_)
        {
            set_status(BT::SUCCESS);
            std::cout << get_name() << " returning Success " << BT::SUCCESS << "!" << std::endl;
            return BT::SUCCESS;
        }
        else
        {
            set_status(BT::FAILURE);
            std::cout << get_name() << " returning Failure " << BT::FAILURE << "!" << std::endl;
            return BT::FAILURE;
        }
}


void BT::ConditionNodeStayStill::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}

