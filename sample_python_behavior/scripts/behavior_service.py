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
# For simple behaviors that can run syncronously, Python provides
# a simple way to implement this.  Add the work of your behavior
# in the execute_cb callback
#

import rospy
import actionlib
import behavior_common.msg

class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Python behavior service' % (self._action_name))
      
    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))

        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # TODO: Do your work here for behavior.  
        r = rospy.Rate(1.0)
        success = True

        for i in range(1, 5):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Behavior preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            rospy.loginfo('%s: Running behavior' % (self._action_name))
            self._feedback.running = True
            self._as.publish_feedback(self._feedback)

            r.sleep()
          
        if success:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('sample_python_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
