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

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behaviorAction.h>
#include <behavior_common/CommandState.h>
#include <map>
#include <signal.h>

// TODO: Get rid of all the BOOST stuff in this module, and replace with nice/clean C++11 versions
using namespace behavior_common;

class behavior_logic_node 
{
public:
  behavior_logic_node(std::string name) :
    _name(name)
  {
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;

    behaviorLoader_.reset( new pluginlib::ClassLoader<behavior_common::BehaviorPlugin>("behavior_common", "behavior_common::BehaviorPlugin"));

    // Enable launch script to override default startup and idle behaviors
    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("robot_startup_behavior",robot_startup_behavior_,"/idle_behavior");
    nodeHandle.param<std::string>("robot_idle_behavior",robot_idle_behavior_,"/idle_behavior");
    nodeHandle.param<std::string>("robot_shutdown_behavior",robot_shutdown_behavior_,"/idle_behavior");


    ROS_INFO_STREAM("Behavior Loader: robot_startup_behavior = " << robot_startup_behavior_); 
    ROS_INFO_STREAM("Behavior Loader: robot_idle_behavior = " << robot_idle_behavior_); 
    ROS_INFO_STREAM("Behavior Loader: robot_shutdown_behavior = " << robot_shutdown_behavior_); 

    // Subscribe to global robot behavior state command
    robot_behavior_state_ = nh_.subscribe("/behavior/cmd", 1, &behavior_logic_node::behaviorStateCB, this);

    // Enumerate all available robot behavior plugins, and initialize them
    BOOST_FOREACH(const std::string& behaviorPluginName, behaviorLoader_->getDeclaredClasses()) 
    {
      ROS_INFO_STREAM("Found behavior plugin: " << behaviorLoader_->getClassPackage(behaviorPluginName)); 
      try 
      {
        boost::shared_ptr<behavior_common::BehaviorPlugin> plugin = behaviorLoader_->createInstance(behaviorPluginName);

        plugin->initialize();
        
        availableBehaviors_[plugin->behaviorCommandString()] = plugin;

        std::string behaviorActionServiceName = plugin->behaviorActionServiceName();
        behaviorActionClient_[behaviorActionServiceName] = new actionlib::SimpleActionClient<behaviorAction>(behaviorActionServiceName);
      }
      catch (const pluginlib::LibraryLoadException& e) 
      {
       ROS_ERROR_STREAM("LibraryLoadException generated for behavior plugin: " + behaviorPluginName);
      }
      catch (const pluginlib::CreateClassException& e) 
      {
       ROS_ERROR_STREAM("CreateClassException generated for behavior plugin: " + behaviorPluginName);;
      }
    }

    // Wait for client server registrations above to complete in parallel
    ROS_WARN("Connecting to behavior plugin actions ...");
    int retries = 0;
    do
    {
      ros::spinOnce();

      initialized = true;
      for(auto& x: behaviorActionClient_)
      {
        if(!x.second->isServerConnected())
        {
          ROS_WARN_STREAM("  still waiting for behavior plugin action " + x.first);
          initialized = false;
        }
      }

      if(initialized)
        break;

      ros::Duration(0.5).sleep();
    }
    while(retries++<20);

    if(!initialized)
      ROS_ERROR( "%s: Initializing failed", _name.c_str());
    else
    {
      ROS_INFO("%s: Initializing complete", _name.c_str());
      pushState(robot_startup_behavior_);
    }
  }

  ~behavior_logic_node()
  {
    clearStateStack();

    ROS_INFO("behavior_loader shutting down");
    for(auto& x : availableBehaviors_)
    {
      x.second->shutdown();
    }
  }

  void behaviorStateCB(const behavior_common::CommandState::ConstPtr& msg)
  {
    auto behavior = availableBehaviors_.find(msg->commandState);

    // If requested to become idle (or requested to perform an unknown behavior),
    // start the designated idle behavior.
    if((availableBehaviors_.end() == behavior) || ("IDLE" == msg->commandState))
    {
      if(availableBehaviors_.end() == behavior)
      {
        ROS_ERROR("Warning, received request for behavior that does not exist, "
        "defaulting to idle state instead. "
        "Behavior requested: %s", msg->commandState.c_str());
      }

      clearStateStack();
      idleState();
      return;
    }
    
    // Invoke the requested behavior
    ROS_INFO("Starting behavior: %s", msg->commandState.c_str());
    pushState(behavior->second->behaviorActionServiceName(), msg->param1, msg->param2);
  }

  void transitionToState(std::string newState, std::string param1, std::string param2)
  {
    if(behaviorActionClient_.end() == behaviorActionClient_.find(newState))
    {
      ROS_ERROR("No behavior plugin available for %s, ignoring", newState.c_str());
      return;
    }

    // if there is currently a state already running, cancel it first
    if(!currentState_.empty())
    {
      ROS_INFO("** %s:  deferred [%s], cancelling [%s] first", _name.c_str(), newState.c_str(), currentState_.c_str());

      // Defer next state until current state is cancelled
      nextState_ = newState;
      nextStateParam1_ = param1;
      nextStateParam2_ = param2;
      behaviorActionClient_[currentState_]->cancelGoal();
    }
    else
    {
      ROS_INFO("** %s: starting [%s]", _name.c_str(), newState.c_str());

      // Start state
      behaviorGoal goal;
      goal.param1 = param1;
      goal.param2 = param2;
      currentState_ = newState;

      behaviorActionClient_[newState]->sendGoal(goal, 
        boost::bind(&behavior_logic_node::doneCb, this, _1, _2),
        boost::bind(&behavior_logic_node::activeCb, this),
        boost::bind(&behavior_logic_node::feedbackCb, this, _1) );
    }
  }

  void pushState(std::string newState, std::string param1 = "", std::string param2 = "")
  {
    // For now, there is no stack.  Just transition to new state
    transitionToState(newState, param1, param2);
  }

  void clearStateStack()
  {
    if(!currentState_.empty())
      behaviorActionClient_[currentState_]->cancelGoal(); 
  }

  void idleState()
  {
    ROS_INFO("** %s: idleState", _name.c_str());

    // For now, there is no stack, and going back just runs the default state
    pushState(robot_idle_behavior_);
  }

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state, const behaviorResultConstPtr& result)
  {
    ROS_INFO("** %s: Finished [%s], state [%s]", _name.c_str(), currentState_.c_str(), state.toString().c_str());
    currentState_.clear();

    if(!nextState_.empty())
    {
      transitionToState(nextState_, nextStateParam1_, nextStateParam2_);
      nextState_.clear();
    }
    else
      idleState();
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("** %s: [%s] just went active", _name.c_str(), currentState_.c_str());
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const behaviorFeedbackConstPtr& feedback)
  {
    ROS_INFO("** %s: Received Goal feedback", _name.c_str());
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<behavior_common::BehaviorPlugin>> behaviorLoader_;
  std::map<std::string, actionlib::SimpleActionClient<behaviorAction>*> behaviorActionClient_;
  std::map<std::string, boost::shared_ptr<behavior_common::BehaviorPlugin> > availableBehaviors_;
  std::string _name;
  ros::NodeHandle nh_;
  ros::Subscriber robot_behavior_state_;
  bool stateActive_;
  std::string currentState_, nextState_, nextStateParam1_, nextStateParam2_;
  std::stack<std::string> stateStack_;
  std::string robot_startup_behavior_;
  std::string robot_idle_behavior_;
  std::string robot_shutdown_behavior_;
};

bool run_ = true;

void mySigintHandler(int sig)
{
    ROS_INFO("shutting down behavior_loader");
    run_ = false; 
}

// 
// The main entry point for this node.
//
int main( int argc, char *argv[] )
{
  ros::init( argc, argv, "behavior_loader" );
  ros::NodeHandle nh;

  signal(SIGINT, mySigintHandler);
  behavior_logic_node node(ros::this_node::getName());

  // Run ROS message loop at 5hz, until ROS shutdown or ctrl-c
  ros::Rate loop_rate(5); 
  while (ros::ok() && run_)
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

