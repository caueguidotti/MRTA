/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Caue Franco Guidotti
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "murdoch_rta/MurdochExceptions.h"
#include "murdoch_rta_msgs/Task.h"
#include "utils/time.h"

#include "murdoch_rta/resource/Discrete.h"
#include "murdoch_rta/resource/Float.h"
#include "murdoch_rta/resource/FloatRange.h"
#include "murdoch_rta/resource/Integer.h"
#include "murdoch_rta/resource/IntegerRange.h"
#include "murdoch_rta/resource/Resource.h"

#include "boost/smart_ptr.hpp"

namespace murdoch_rta
{
namespace task
{
class Task;
typedef boost::shared_ptr<Task> taskPtr;
typedef const boost::shared_ptr<const Task> taskConstPtr;
typedef murdoch_rta_msgs::Task TaskMsg;
typedef murdoch_rta_msgs::Task::ConstPtr TaskMsgConstPtr;

class Task
{
private:
  int _taskUID;
  int _priority;
  bool _taskTimeLimitCalculated;
  std::string _taskIdentifier;
  ros::Time _taskAuctionLimit;
  ros::Duration _taskAuctionDeadline;
  resource::ResourcesVecPtr _resources;

  resource::ResourceConstPtr getResourceFromVecByIndex(const unsigned &index) const;
  void initTask(const int &taskUID, const int &priority, const std::string &taskIdentifier,
                const ros::Duration &taskAuctionDeadline, const resource::ResourcesVec *resources,
                const bool &taskTimeLimitCalculated, const ros::Time &taskAuctionLimit);

public:
  Task(const int &taskUID, const int &priority, const std::string &taskIdentifier,
       const resource::ResourcesVec *resources, const ros::Duration &taskAuctionDeadline = ros::Duration(86400));
  Task(const Task &task);
  Task(const Task *task);
  Task(const murdoch_rta_msgs::Task::ConstPtr &taskMsgRos);
  Task(const murdoch_rta_msgs::Task &taskMsgRos);

  ~Task()
  {
  }

  int getPriority() const;
  int getTaskUID() const;
  ros::Duration getTaskAuctionDurationDeadline() const;
  ros::Time getTaskAuctionTimeDeadline() const;
  std::string getTaskIdentifier() const;
  bool getTaskTimeLimitCalculated() const;

  void setTaskAuctionDeadline(const ros::Duration &taskDuration);

  const resource::ResourcesVec *getResources() const;
  void setResources(resource::ResourcesVecPtr &resourcePtrVec);

  murdoch_rta_msgs::Task toRosMessage() const;

  static bool isTaskValid(const Task &task);
  static bool isTaskValid(const Task *&task);

  static std::string taskStatusTopic(const Task &task);
  static std::string taskStatusTopic(const Task *task);
  static std::string taskStatusTopic(const taskPtr task);
  static std::string taskStatusTopic(const TaskMsg &task);
  static std::string taskStatusTopic(TaskMsgConstPtr task);
  static std::string taskStatusTopic(const int &taskUID);
};
}
}
