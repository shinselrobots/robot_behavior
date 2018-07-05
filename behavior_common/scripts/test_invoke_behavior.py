#! /usr/bin/env python

# Copyright 2018 Matt Curfman, Dave Shinsel
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Usage: test_invoke_behavior.py <behavior_service_name> <param1> <param2>
#

import sys
import signal
import rospy
import actionlib
import actionlib.action_client
import behavior_common.msg

def signal_handler(signal, frame):
    if( None != client):
        rospy.loginfo("Ctrl-c, canceling behavior");
        client.cancel_goal()
    sys.exit(0)

if __name__ == '__main__':
    client = None

    rospy.init_node('test_invoke_behavior')
    signal.signal(signal.SIGINT, signal_handler);

    client = actionlib.SimpleActionClient(sys.argv[1], behavior_common.msg.behaviorAction)
    client.wait_for_server()

    rospy.loginfo("Invoke behavior: %s", sys.argv[1])
    goal = behavior_common.msg.behaviorGoal(param1=sys.argv[2], param2=sys.argv[3])
    client.send_goal(goal)

    result = client.wait_for_result()
    rospy.loginfo("Behavior returned result: %d", result)