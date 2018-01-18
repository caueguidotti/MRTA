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

#include "murdoch_rta_msgs/Auction.h"
#include "murdoch_rta_msgs/Boolean.h"
#include "murdoch_rta_msgs/getResources.h"
#include "murdoch_rta_msgs/getTaskStatus.h"

#include "murdoch_rta/communicator/messages/AuctionMessage.h"
#include "murdoch_rta/communicator/messages/BooleanMessage.h"
#include "murdoch_rta/task/Task.h"

#include <vector>

typedef murdoch_rta::communicator::messages::BooleanMessage BooleanMessage;
typedef murdoch_rta::communicator::messages::AuctionMessage AuctionMessage;
typedef murdoch_rta::task::Task Task;

typedef boost::shared_ptr<BooleanMessage> BooleanMessagePtr;
typedef boost::shared_ptr<AuctionMessage> AuctionMessagePtr;

struct TaskStatusService
{
  int taskUID;
  ros::ServiceServer getTaskStatusSrvServer;
};

struct TaskStatus
{
  int taskUID;
  murdoch_rta::communicator::messages::BooleanMessagesType taskStatus;
};

class BaseTaskMonitor
{
public:
  BaseTaskMonitor(ros::NodeHandle *nh);
  virtual ~BaseTaskMonitor();

  void initSubscribers();
  void processTaskMonitor();

protected:
  ros::NodeHandle *_nh;  // Node handle from caller node

  int getTaskRobotID(const int &taskUID) const;
  std::vector<AuctionMessagePtr> getAuctionMessagesProc() const;
  TaskStatus getTaskStatus(const int &taskUID) const;

  virtual void onAddingAuction(const int &taskUID);
  virtual void onAddedAuction(const int &taskUID);
  virtual void onDeletingAuction(const int &taskUID);
  virtual void onDeletedAuction(const int &taskUID);

private:
  ros::Subscriber _subBoolMsg;                            // Subscriber that will listen to winning messages
  ros::Subscriber _subAuctionMsg;                         // Subscriber that will listen to auction messages
  std::vector<BooleanMessagePtr> _wonAuctionMessages;     // vector to store incoming won auction messages
  std::vector<BooleanMessagePtr> _endTaskMessages;        // vector to store completed cancelled tasks messages
  std::vector<AuctionMessagePtr> _auctionMessagesUnproc;  // vector to store all incoming auctions
  std::vector<AuctionMessagePtr> _auctionMessagesProc;    // vector to store all auction which received a won msg
  std::vector<TaskStatus> _tasksStatus;                   // Vector containing current tasks being monitored status
  std::vector<TaskStatusService> _tasksStatusSvrs;  // Vector of service servers where murdoch_rta will obtain task
                                                    // status
  //  Type is actually a struct containing the information of the taskUID
  //  and a ros::ServiceServer.

  virtual TaskStatus evaluateTaskProgress(const Task &monitoringTask) = 0;
  void processUnprocAuctions();
  void processProcAuctions();
  void addProcAuction(AuctionMessagePtr auction);
  bool hasWonAuctionMessageByTaskUID(const int &taskUID, BooleanMessage &wonAuctionMessage);
  bool hasEndTaskMessageByTaskUID(const int &taskUID, const int &auctionerID);
  void clearMsgsByTaskUID(const int &taskUID);
  bool setTaskStatus(const int &taskUID, const TaskStatus &taskStatus);
  void removeTaskStatus(const int &taskUID);
  bool isTaskStatusStopTaskMsg(const TaskStatus &taskStatus) const;
  bool stopAuctionTaskMonitoring(const int &taskUID);
  bool stopTaskStatusService(const int &taskUID);

  void auctionMsgSubCallback(const murdoch_rta_msgs::Auction::ConstPtr &murdochAuctionMsg);  // Callback of auction
                                                                                             // messages
  void wonAuctionMsgSubCallback(const murdoch_rta_msgs::Boolean::ConstPtr &murdochBoolMsg);  // Callback of bool
                                                                                             // messages
                                                                                             // subscriber
  bool respondTaskStatusRequest(murdoch_rta_msgs::getTaskStatus::Request &req,
                                murdoch_rta_msgs::getTaskStatus::Response &res);
};
