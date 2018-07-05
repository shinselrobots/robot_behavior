// Copyright 2018 Matt Curfman, Dave Shinsel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __BEHAVIOR_UTILS_H__
#define __BEHAVIOR_UTILS_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_common/behaviorAction.h>
#include <sys/types.h>

namespace behavior_common 
{
  /**
  ** This class defines the interface of an object which will load a Python
  ** script and execute it using the Python interpreter.  It is used
  ** for behavior plug-ins which implement their service using Python instead
  ** of C or C++
  **
  ** When the object of this class goes out of scope, the destructor will 
  ** terminate the Python script.
  **/
  class PythonScript
  {
    public:
      PythonScript();
      virtual ~PythonScript();

      void launch(const char *package, const char *fn);

    private:
      pid_t pid_;
  };


  class BehaviorActionServiceBase 
  {
    public:
      BehaviorActionServiceBase(std::string name);
      virtual ~BehaviorActionServiceBase();

      virtual void StartBehavior(const char *param1, const char *param2);
      virtual void PremptBehavior();

      void StartBehaviorCallback();
      void PremptBehaviorCallback();

      void BehaviorComplete();
      
      protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviorAction> as_;
        std::string action_name_;
        behaviorFeedback feedback_;
        behaviorResult result_;
    };
};
#endif
