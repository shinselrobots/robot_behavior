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

#ifndef __BEHAVIOR_H__
#define __BEHAVIOR_H__

#include <behavior_common/behavior_common.h>
#include <memory>

namespace behavior_common 
{
  /**
  ** This class defines the pure virtual interface defining a ROS Behavior
  ** plugin. 
  **/
  class BehaviorPlugin
  {
    public:
      virtual void initialize() = 0;
      virtual void shutdown() = 0;

      // This method returns the string intended to be used to match
      // the 'intent' to the 'behavior'
      virtual const char* behaviorCommandString() = 0;

      // This method returns the string intended to be used to
      // connect the behavior with its actionlib server used
      // to run the behavior
      virtual const char *behaviorActionServiceName() = 0;

      virtual ~BehaviorPlugin(){}

    protected:
      BehaviorPlugin(){}
  };
};

#define CPP_BEHAVIOR_PLUGIN(W,X,Y,Z) \
class W : public behavior_common::BehaviorPlugin \
{ \
    public: \
      W() : \
        service_name_(X), \
        intent_name_(Y) \
      {} \
\
      virtual void initialize() \
      { \
        service_.reset(new Z(service_name_)); \
      } \
\
      virtual void shutdown() \
      { \
        service_.reset(); \
      } \
\
      const char* behaviorActionServiceName() \
      { \
        return service_name_.c_str(); \
      } \
\
      const char* behaviorCommandString() \
      { \
        return intent_name_.c_str(); \
      } \
\
    private: \
      std::shared_ptr<Z> service_; \
      std::string service_name_; \
      std::string intent_name_; \
  };


#define PYTHON_BEHAVIOR_PLUGIN(NAME, INTENT) \
class NAME : public behavior_common::BehaviorPlugin \
{ \
    public: \
      NAME() : \
        service_name_("/" #NAME), \
        intent_name_(INTENT) \
      {} \
\
      virtual void initialize() \
      { \
        script_.reset(new behavior_common::PythonScript()); \
        script_->launch(#NAME, "behavior_service.py"); \
      } \
\
      virtual void shutdown() \
      { \
        script_.reset(); \
      } \
\
      const char* behaviorActionServiceName() \
      { \
        return service_name_.c_str(); \
      } \
\
      const char* behaviorCommandString() \
      { \
        return intent_name_.c_str(); \
      } \
\
    private: \
      std::shared_ptr<behavior_common::PythonScript> script_; \
      std::string service_name_; \
      std::string intent_name_; \
  };

#endif
