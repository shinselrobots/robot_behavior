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

#include <Python.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <signal.h>
#include <wait.h>
#include <behavior_common/behavior_common.h>
#include <ros/package.h>

namespace behavior_common
{
    PythonScript::PythonScript() :
        pid_(-1)
    {
    }

    PythonScript::~PythonScript()
    {
        // ROS_INFO( "Shutting down python script, PID %d", pid_);

        // // Send nice request to shutdown
        // kill(pid_, SIGTERM);

        // // Wait 5 seconds for shutdown to complete, otherwise kill it
        // for (int i = 0; i < 5; ++i)
        // {
        //     int status;
        //     if (waitpid(pid_, &status, WNOHANG) == pid_) 
        //         return;

        //     sleep(1);
        // }

        ROS_INFO( "Killing python script, PID %d", pid_);
        kill(pid_, SIGKILL);
    }

    void PythonScript::launch(const char *package, const char *fn)
    {
        std::string package_path = ros::package::getPath(package);
        std::string script_path = package_path + "/scripts/" + std::string(fn);

        pid_ = fork();
        if (pid_ == 0)
        {
            char *argv[1];
            argv[0] = (char*)fn;

            // child process
            Py_Initialize();
            FILE *fp = fopen(script_path.c_str(), "r");
            if(NULL == fp )
            {
              ROS_ERROR( "When trying to load the behavior plugin '%s', could not find python script %s to execute.  Was it installed correctly by your CMakeLists.txt file??", package, script_path.c_str());
            }
            else
            {
              PySys_SetArgv(1, argv);
              PyRun_AnyFile(fp, script_path.c_str());
              fclose(fp);
            }
            Py_Finalize();

            // when script above completes, exit our forked process
            exit(0);
        }  
    }


    BehaviorActionServiceBase::BehaviorActionServiceBase(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        ROS_INFO( "%s: initializing C++ behavior service", name.c_str());

        // Register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&BehaviorActionServiceBase::StartBehaviorCallback, this));
        as_.registerPreemptCallback(boost::bind(&BehaviorActionServiceBase::PremptBehaviorCallback, this));
        as_.start();
    }

    BehaviorActionServiceBase::~BehaviorActionServiceBase()
    {
        if (as_.isActive())
        {    
            as_.setAborted(result_);
        }
    }

    void BehaviorActionServiceBase::StartBehaviorCallback()
    {
        // Only accept a single running goal at a time
        if (as_.isActive())
        {
            as_.setAborted(result_);
            return;
        }

        // accept the new goal
        behavior_common::behaviorGoalConstPtr goal = as_.acceptNewGoal();

        ROS_INFO("'%s': Started", action_name_.c_str());
        StartBehavior(goal->param1.c_str(), goal->param2.c_str());
    }

    void BehaviorActionServiceBase::StartBehavior(const char *param1, const char *param2)
    {
        // Base class does nothing
    }

    void BehaviorActionServiceBase::PremptBehaviorCallback()
    {
        // Can't pre-empty if no running goal
        if (!as_.isActive())
            return;

        ROS_INFO("'%s': Preempted", action_name_.c_str());
        PremptBehavior();
        as_.setPreempted();      
    }

    void BehaviorActionServiceBase::PremptBehavior()
    {
        // Base class does nothing
    }

    void BehaviorActionServiceBase::BehaviorComplete()
    {
        as_.setSucceeded();
    }
}
