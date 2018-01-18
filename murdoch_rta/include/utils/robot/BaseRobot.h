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

#include "ros/ros.h"

#include "boost/algorithm/string.hpp"  // for case-insentive string comparasion

#include "murdoch_rta_msgs/Task.h"
#include "murdoch_rta_msgs/getResources.h"
#include "murdoch_rta_msgs/getTaskStatus.h"
#include "murdoch_rta_msgs/setTask.h"

#include "murdoch_rta/communicator/messages/BooleanMessage.h"
#include "murdoch_rta/resource/Discrete.h"
#include "murdoch_rta/resource/Float.h"
#include "murdoch_rta/resource/FloatRange.h"
#include "murdoch_rta/resource/Integer.h"
#include "murdoch_rta/resource/IntegerRange.h"

#include "murdoch_rta/task/Task.h"

class BaseRobot
{
public:
  BaseRobot(ros::NodeHandle *nh);
  virtual ~BaseRobot();

  virtual void processTask();

protected:
  ros::NodeHandle *_nh;
  murdoch_rta::task::taskPtr _currentTask;
  murdoch_rta::resource::ResourcesVec _robotMurdochResources;

private:
  void initTopics();
  void clearTask();

  ros::Time _timeToWaitTaskRenewal;

  ros::ServiceServer _getResourcesSrvServer;
  ros::ServiceServer _setTaskSrvServer;

  bool respondResourcesRequest(murdoch_rta_msgs::getResources::Request &req,
                               murdoch_rta_msgs::getResources::Response &res);
  bool respondSetTaskRequest(murdoch_rta_msgs::setTask::Request &req, murdoch_rta_msgs::setTask::Response &res);

  virtual void executeTask() = 0;
};
