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

#include "utils/robot/BaseRobot.h"

BaseRobot::BaseRobot(ros::NodeHandle *nh) : _nh(nh)
{
  _currentTask.reset();
  _timeToWaitTaskRenewal = ros::Time::now();

  initTopics();  // calls subs/pubs/srvs initialization
}

BaseRobot::~BaseRobot()
{
  _getResourcesSrvServer.shutdown();
  _setTaskSrvServer.shutdown();
}

void BaseRobot::initTopics()
{
  // murdoch_rta will request robot resources from this service
  _getResourcesSrvServer = _nh->advertiseService("getResources", &BaseRobot::respondResourcesRequest, this);

  // murdoch_rta will request robot to set its task from this service
  _setTaskSrvServer = _nh->advertiseService("setTask", &BaseRobot::respondSetTaskRequest, this);
}

void BaseRobot::clearTask()
{
  _currentTask.reset();
}

void BaseRobot::processTask()
{
  if (_currentTask)  // if has task, call fn to execute it
  {
    if (ros::Time::now() > _timeToWaitTaskRenewal)
    {
      clearTask();
    }
    else
    {
      executeTask();
    }
  }
}

bool BaseRobot::respondSetTaskRequest(murdoch_rta_msgs::setTask::Request &req, murdoch_rta_msgs::setTask::Response &res)
{
  // FIXME check if requested to cancel task!
  // if (isCancelTaskRequest)
  if (_currentTask)
  {
    if (_currentTask->getTaskUID() == req.task.taskUID)
    {
      if (boost::iequals(_currentTask->getTaskIdentifier(), req.task.taskIdentifier))
      {
        // here I'm sure tasks are identifical, so I will just add more time to the task
        _timeToWaitTaskRenewal = req.taskRenewalBeforeTime;

        res.isTaskSet = true;  // inform bidder task was set correctly
        return true;
      }
    }
    res.isTaskSet = false;
    _timeToWaitTaskRenewal = ros::Time::now();  // this should stop task execution
  }
  else
  {
    // robot does not have a task yet
    _currentTask = boost::make_shared<murdoch_rta::task::Task>(req.task);
    _timeToWaitTaskRenewal = req.taskRenewalBeforeTime;
    res.isTaskSet = true;  // inform bidder task was set correctly
  }

  return true;
}

bool BaseRobot::respondResourcesRequest(murdoch_rta_msgs::getResources::Request &req,
                                        murdoch_rta_msgs::getResources::Response &res)
{
  std::vector<murdoch_rta_msgs::Resource> myResources;

  if (_robotMurdochResources.size() > 0)
  {
    murdoch_rta::resource::ResourcesVec::iterator it = _robotMurdochResources.begin();
    for (; it != _robotMurdochResources.end(); it++)
    {
      if ((*it))
        myResources.push_back((*it)->toRosMessage());
    }
  }

  res.robotResources = myResources;
  return true;
}
