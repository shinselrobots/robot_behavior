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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>


namespace behavior_plugin 
{
    class MyAsyncBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        MyAsyncBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
          // TODO: Add what you need to get your behavior ready
        }

        virtual ~MyAsyncBehaviorService()
        {
          // TODO: Add what you need to destroy any resources allocated for your behavior
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          // TODO: Add the implementation of your behavior here.  You can either
          // register additional callbacks which will be invoked while your behavior
          // is running, or you can run the entire behavior here (watching for a
          // possible request to prempt your behavior below)

          // Mark behavior complete
          BehaviorComplete();
        }

        virtual void PremptBehavior()
        {
          // TODO: Add code which prempts your running behavior here.
        }
    };

    CPP_BEHAVIOR_PLUGIN(SampleAsyncCppBehaviorPlugin, "/sample_cpp_behavior_service", "SAMPLE_CPP_BEHAVIOR", MyAsyncBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::SampleAsyncCppBehaviorPlugin, behavior_common::BehaviorPlugin);
