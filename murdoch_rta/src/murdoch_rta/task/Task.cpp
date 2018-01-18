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

#include "murdoch_rta/task/Task.h"
#include "ros/ros.h"

namespace murdoch_rta
{
namespace task
{
Task::Task(const int &taskUID, const int &priority, const std::string &taskIdentifier,
           const resource::ResourcesVec *resources, const ros::Duration &taskAuctionDeadline)
{
  initTask(taskUID, priority, taskIdentifier, taskAuctionDeadline, resources, false, ros::Time::now());
}

Task::Task(const Task &task)
{
  initTask(task.getTaskUID(), task.getPriority(), task.getTaskIdentifier(), task.getTaskAuctionDurationDeadline(),
           task.getResources(), task.getTaskTimeLimitCalculated(), task.getTaskAuctionTimeDeadline());
}

Task::Task(const Task *task)
{
  initTask(task->getTaskUID(), task->getPriority(), task->getTaskIdentifier(), task->getTaskAuctionDurationDeadline(),
           task->getResources(), task->getTaskTimeLimitCalculated(), task->getTaskAuctionTimeDeadline());
}

Task::Task(const murdoch_rta_msgs::Task &taskMsgRos)
{
  ros::Duration taskDeadlineDuration = ros::Duration(taskMsgRos.taskAuctionDeadline);
  ros::Time taskAuctionLimit = taskMsgRos.baseMsg.messageTime + taskDeadlineDuration;

  initTask(taskMsgRos.taskUID, taskMsgRos.taskPriority, taskMsgRos.taskIdentifier, taskDeadlineDuration,
           resource::Resource::resourcesVecRosMsgToVecPtr(taskMsgRos.taskResources).get(), true, taskAuctionLimit);
}

Task::Task(const murdoch_rta_msgs::Task::ConstPtr &taskMsgRos)
{
  ros::Duration taskDeadlineDuration = ros::Duration(taskMsgRos->taskAuctionDeadline);
  ros::Time taskAuctionLimit = taskMsgRos->baseMsg.messageTime + taskDeadlineDuration;

  initTask(taskMsgRos->taskUID, taskMsgRos->taskPriority, taskMsgRos->taskIdentifier, taskDeadlineDuration,
           resource::Resource::resourcesVecRosMsgToVecPtr(taskMsgRos->taskResources).get(), true, taskAuctionLimit);
}

void Task::initTask(const int &taskUID, const int &priority, const std::string &taskIdentifier,
                    const ros::Duration &taskAuctionDeadline, const resource::ResourcesVec *resources,
                    const bool &taskTimeLimitCalculated, const ros::Time &taskAuctionLimit)
{
  // Set up task attributes
  _taskUID = taskUID;
  _priority = priority;
  _taskIdentifier = taskIdentifier;
  _taskAuctionDeadline = taskAuctionDeadline;

  if (taskTimeLimitCalculated)
    _taskAuctionLimit = taskAuctionLimit;
  else
    _taskAuctionLimit = ros::Time::now() + taskAuctionDeadline;
  _taskTimeLimitCalculated = true;

  // Set up resources
  _resources.reset(new resource::ResourcesVec);
  // Iterate through each resource from parameter Resources and push back into my Resources
  resource::ResourcesVec::const_iterator it = resources->begin();
  for (; it != resources->end(); it++)
  {
    switch ((*it)->getResourceType())
    {
      case resource::DiscreteResource:
      {
        boost::shared_ptr<resource::Discrete> typedRes = boost::dynamic_pointer_cast<resource::Discrete>(*it);
        _resources->push_back(boost::make_shared<resource::Discrete>(*(typedRes.get())));
        break;
      }
      case resource::IntegerResource:
      {
        boost::shared_ptr<resource::Integer> typedRes = boost::dynamic_pointer_cast<resource::Integer>(*it);
        _resources->push_back(boost::make_shared<resource::Integer>(*(typedRes.get())));
        break;
      }
      case resource::IntegerRangeResource:
      {
        boost::shared_ptr<resource::IntegerRange> typedRes = boost::dynamic_pointer_cast<resource::IntegerRange>(*it);
        _resources->push_back(boost::make_shared<resource::IntegerRange>(*(typedRes.get())));
        break;
      }
      case resource::FloatResource:
      {
        boost::shared_ptr<resource::Float> typedRes = boost::dynamic_pointer_cast<resource::Float>(*it);
        _resources->push_back(boost::make_shared<resource::Float>(*(typedRes.get())));
        break;
      }
      case resource::FloatRangeResource:
      {
        boost::shared_ptr<resource::FloatRange> typedRes = boost::dynamic_pointer_cast<resource::FloatRange>(*it);
        _resources->push_back(boost::make_shared<resource::FloatRange>(*(typedRes.get())));
        break;
        ;
      }
      default:
        throw CastFailed("Resource within Task does not have a valid type.");
    }
  }
}

int Task::getPriority() const
{
  return _priority;
}

int Task::getTaskUID() const
{
  return _taskUID;
}

ros::Duration Task::getTaskAuctionDurationDeadline() const
{
  return _taskAuctionDeadline;
}

std::string Task::getTaskIdentifier() const
{
  return _taskIdentifier;
}

void Task::setTaskAuctionDeadline(const ros::Duration &taskDuration)
{
  _taskAuctionDeadline = taskDuration;
  _taskAuctionLimit = ros::Time::now() + _taskAuctionDeadline;
}

ros::Time Task::getTaskAuctionTimeDeadline() const
{
  return _taskAuctionLimit;
}

resource::ResourcesVec const *Task::getResources() const
{
  return _resources.get();
}

void Task::setResources(resource::ResourcesVecPtr &resources)
{
  _resources = resources;
}

bool Task::isTaskValid(const Task &task)
{
  const Task *tempTask = &task;
  Task::isTaskValid(tempTask);
}

bool Task::isTaskValid(const Task *&task)
{
  if (!task)  // task is NULL
    return false;

  if (!task->getResources())  // pointer to resource vector is null
    return false;

  if (task->getPriority() < 0 || task->getTaskIdentifier().empty() || task->getTaskUID() < 0 ||
      task->getResources()->size() == 0)
  {
    return false;
  }

  for (unsigned i = 0; i < task->getResources()->size(); i++)
  {
    resource::Resource *res = task->getResources()->at(i).get();
    if (!res)  // a resource in vector is null
      return false;

    if (res->getResourceIdentifier().empty())
      return false;

    resource::Resource *typedResource;
    switch (res->getResourceType())  // For prevision
    {
      case resource::DiscreteResource:
      {
        typedResource = dynamic_cast<resource::Discrete *>(res);
        if (!typedResource)
          return false;
        break;
      }
      case resource::IntegerResource:
      {
        typedResource = dynamic_cast<resource::Integer *>(res);
        if (!typedResource)
          return false;
        break;
      }
      case resource::IntegerRangeResource:
      {
        typedResource = dynamic_cast<resource::IntegerRange *>(res);
        if (!typedResource)
          return false;
        if (dynamic_cast<resource::IntegerRange *>(res)->getResourceUpperBound() <
            dynamic_cast<resource::IntegerRange *>(res)->getResourceLowerBound())
          return false;
        break;
      }
      case resource::FloatResource:
      {
        typedResource = dynamic_cast<resource::Float *>(res);
        if (!typedResource)
          return false;
        break;
      }
      case resource::FloatRangeResource:
      {
        typedResource = dynamic_cast<resource::FloatRange *>(res);
        if (!typedResource)
          return false;
        if (dynamic_cast<resource::FloatRange *>(res)->getResourceUpperBound() <
            dynamic_cast<resource::FloatRange *>(res)->getResourceLowerBound())
          return false;
        break;
      }
      default:
      {
        return false;
      }
    }
  }
  return true;
}

std::string Task::taskStatusTopic(const Task &task)
{
  Task::taskStatusTopic(task.getTaskUID());
}

std::string Task::taskStatusTopic(const Task *task)
{
  Task::taskStatusTopic(task->getTaskUID());
}

std::string Task::taskStatusTopic(const taskPtr task)
{
  Task::taskStatusTopic(task->getTaskUID());
}

std::string Task::taskStatusTopic(const TaskMsg &task)
{
  Task::taskStatusTopic(task.taskUID);
}

std::string Task::taskStatusTopic(TaskMsgConstPtr task)
{
  Task::taskStatusTopic(task->taskUID);
}

std::string Task::taskStatusTopic(const int &taskUID)
{
  std::stringstream srvTopicSS;
  srvTopicSS << "/" << taskUID << "/getTaskStatus";
  return srvTopicSS.str();
}

murdoch_rta_msgs::Task Task::toRosMessage() const
{
  murdoch_rta_msgs::Task taskMsg;

  taskMsg.taskUID = _taskUID;
  taskMsg.taskPriority = _priority;
  taskMsg.taskAuctionDeadline = _taskAuctionDeadline.toSec();
  taskMsg.taskIdentifier = _taskIdentifier;

  taskMsg.baseMsg.messageTime = ros::Time::now();

  if (!_resources)
    throw murdoch_rta::CastFailed("Cannot cast Task to ROS Message, pointer to resources vector is NULL");

  for (unsigned i = 0; i < _resources->size(); ++i)
  {
    if (!(_resources->at(i)))
      throw murdoch_rta::CastFailed("Cannot cast Task to ROS Message, a resource is NULL");
    taskMsg.taskResources.push_back(_resources->at(i).get()->toRosMessage());
  }

  return taskMsg;
}

resource::ResourceConstPtr Task::getResourceFromVecByIndex(const unsigned &index) const
{
  _resources->at(index);
}

bool Task::getTaskTimeLimitCalculated() const
{
  return _taskTimeLimitCalculated;
}
}
}
