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

#include "murdoch_rta/communicator/Talker.h"

namespace murdoch_rta
{
namespace communicator
{
Talker::Talker(ros::NodeHandle *nh, int robotID, bool isAuctioner)
  : _nh(nh), _robotID(robotID), _isAuctioner(isAuctioner)
{
  initPublisher();
  initServiceClient();
}

Talker::~Talker()
{
  _boolMgsPub.shutdown();
  _auctionsPub.shutdown();
  _bidPub.shutdown();
  _requestResourcesSrvClient.shutdown();
  _requestTaskStatusSrvClient.shutdown();
  _requestSetRobotTaskSrvClient.shutdown();
}

void Talker::initPublisher()
{
  if (_isAuctioner)
    _auctionsPub = _nh->advertise<murdoch_rta_msgs::Auction>("/auctions", 10);
  else
    _bidPub = _nh->advertise<murdoch_rta_msgs::Bid>("/bids", 10);
  _boolMgsPub = _nh->advertise<murdoch_rta_msgs::Boolean>("/boolMsgs", 10);
}

void Talker::initServiceClient()
{
  if (!_isAuctioner)
  {
    _requestResourcesSrvClient = _nh->serviceClient<murdoch_rta_msgs::getResources>("getResources");
    _requestSetRobotTaskSrvClient = _nh->serviceClient<murdoch_rta_msgs::setTask>("setTask");
  }
}

void Talker::publishAuction(const murdoch_rta_msgs::Auction &auction) const
{
  _auctionsPub.publish(auction);
}

void Talker::publishBooleanMsg(const murdoch_rta_msgs::Boolean &boolMsg) const
{
  _boolMgsPub.publish(boolMsg);
}

void Talker::publishBidMsg(const murdoch_rta_msgs::Bid &bidRosMsg) const
{
  _bidPub.publish(bidRosMsg);
}

void Talker::publishBidMsg(const messages::BidMessage &bidMsg) const
{
  publishBidMsg(bidMsg.toRosMessage());
}

void Talker::publishBidMsg(const float &bidValue, const int &auctionerID, const int &taskUID) const
{
  messages::BidMessage bidMsg(_robotID, auctionerID, bidValue, taskUID);
  publishBidMsg(bidMsg);
}

bool Talker::requestResourcesFromRobot(std::vector<murdoch_rta_msgs::Resource> &robotResources)
{
  murdoch_rta_msgs::getResources serviceRequestMsg;

  if (!_requestResourcesSrvClient.call(serviceRequestMsg))
  {
    return false;  // could not retrieve resources
  }
  robotResources = serviceRequestMsg.response.robotResources;
  return true;
}

bool Talker::requestTaskStatus(const task::Task *const task, messages::BooleanMessage &taskStatus)
{
  murdoch_rta_msgs::Task taskRosMsg = task->toRosMessage();
  int taskUID = task->getTaskUID();

  // shutdown any previous service;
  _requestTaskStatusSrvClient.shutdown();

  // set up service based on task UID - this is safe since ROS only allows a single service
  // define topic name
  std::string srvTopicStr = task::Task::taskStatusTopic(taskUID);

  // now initialize topic
  _requestTaskStatusSrvClient = _nh->serviceClient<murdoch_rta_msgs::getTaskStatus>(srvTopicStr);

  murdoch_rta_msgs::getTaskStatus requestTaskStatusMsg;
  requestTaskStatusMsg.request.task = taskRosMsg;
  requestTaskStatusMsg.request.task.baseMsg.robotSenderId = _robotID;
  if (!_requestTaskStatusSrvClient.call(requestTaskStatusMsg))
  {
    return false;
  }

  taskStatus = messages::BooleanMessage(requestTaskStatusMsg.response.taskStatus);
  return true;
}

bool Talker::requestSetRobotTask(const task::Task *const task, const ros::Time &renewalBeforeTime)
{
  murdoch_rta_msgs::setTask serviceRequestSetTask;
  serviceRequestSetTask.request.task = task->toRosMessage();
  serviceRequestSetTask.request.taskRenewalBeforeTime = renewalBeforeTime;

  if (!_requestSetRobotTaskSrvClient.call(serviceRequestSetTask))
    return false;

  return serviceRequestSetTask.response.isTaskSet;
}
}
}
