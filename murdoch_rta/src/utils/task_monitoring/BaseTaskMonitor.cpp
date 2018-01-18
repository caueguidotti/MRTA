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

#include "utils/task_monitoring/BaseTaskMonitor.h"

BaseTaskMonitor::BaseTaskMonitor(ros::NodeHandle *nh) : _nh(nh)
{
  initSubscribers();
}

BaseTaskMonitor::~BaseTaskMonitor()
{
}

void BaseTaskMonitor::initSubscribers()
{
  _subBoolMsg = _nh->subscribe("/boolMsgs", 100, &BaseTaskMonitor::wonAuctionMsgSubCallback, this);
  _subAuctionMsg = _nh->subscribe("/auctions", 10, &BaseTaskMonitor::auctionMsgSubCallback, this);
}

void BaseTaskMonitor::processTaskMonitor()
{
  processUnprocAuctions();
  processProcAuctions();
}

void BaseTaskMonitor::processUnprocAuctions()
{
  // FIXME : Need to receive cancel task message or renew message in case task is cancelled by auctioner

  // iterate through received auctions and check if has received any WonAuction associated with it or if it has expired
  std::vector<AuctionMessagePtr>::iterator auction_it = _auctionMessagesUnproc.begin();

  for (; auction_it != _auctionMessagesUnproc.end();)
  {
    // first check if auction still lives - if doesnt delete an continue (auction_it++) after living longer than 1 sec -
    // since won msg might be delayed
    if ((*auction_it)->getAuctioningTask()->getTaskAuctionTimeDeadline() + ros::Duration(1.0) < ros::Time::now())
    {
      (*auction_it).reset();
      auction_it = _auctionMessagesUnproc.erase(auction_it);
    }
    else
    {
      // Try to obtain a won auction based on task UID of auction
      int auctionTaskUID = (*auction_it)->getAuctioningTask()->getTaskUID();
      BooleanMessage wonAuctionMessage(-1, -1, -1, murdoch_rta::communicator::messages::WonAuction);
      if (hasWonAuctionMessageByTaskUID(auctionTaskUID, wonAuctionMessage))
      {
        // set auction winner - so we know which robot to monitor
        (*auction_it)->setAuctionWinnerID(wonAuctionMessage.getIdReceiver());
        // Raise "event"
        onAddingAuction(auctionTaskUID);

        // Since auction had a winner bidder add this auction to another vector - which tasks will be monitored
        addProcAuction(*auction_it);

        // Remove auction from Unproc vector
        (*auction_it).reset();
        auction_it = _auctionMessagesUnproc.erase(auction_it);

        // Raise "event"
        onAddedAuction(auctionTaskUID);
      }
      else
      {
        auction_it++;
      }
    }
  }
}

void BaseTaskMonitor::processProcAuctions()
{
  // iterate through finished auctions and proccess task status function
  std::vector<AuctionMessagePtr>::iterator auction_it = _auctionMessagesProc.begin();

  for (; auction_it != _auctionMessagesProc.end(); auction_it++)
  {
    int taskUID = (*auction_it)->getAuctioningTask()->getTaskUID();
    TaskStatus taskStatus = evaluateTaskProgress((*auction_it)->getAuctioningTask());
    if (hasEndTaskMessageByTaskUID(taskUID, (*auction_it)->getCreatorID()))
      taskStatus.taskStatus = murdoch_rta::communicator::messages::TaskCancelled;

    setTaskStatus(taskUID, taskStatus);
  }
}

void BaseTaskMonitor::addProcAuction(AuctionMessagePtr auction)
{
  int robotID = auction->getAuctionWinnerID();
  int taskUID = auction->getAuctioningTask()->getTaskUID();

  // clean up variables for TaskUID
  removeTaskStatus(taskUID);
  stopTaskStatusService(taskUID);
  clearMsgsByTaskUID(taskUID);

  // push back task to taskStatus vector - all tasks starts as task continuing as default
  TaskStatus taskStatus;
  taskStatus.taskUID = taskUID;
  taskStatus.taskStatus = murdoch_rta::communicator::messages::TaskContinuing;
  _tasksStatus.push_back(taskStatus);

  // creates service server for auction with winning bidder
  //   defines service topic
  std::string srvTopicStr = murdoch_rta::task::Task::taskStatusTopic(taskUID);
  //   creates a service structure
  TaskStatusService taskStatusSrv;
  taskStatusSrv.taskUID = taskUID;
  taskStatusSrv.getTaskStatusSrvServer =
      _nh->advertiseService(srvTopicStr, &BaseTaskMonitor::respondTaskStatusRequest, this);
  //  push back service to vector
  _tasksStatusSvrs.push_back(taskStatusSrv);

  // Now add auction to Proc vector
  _auctionMessagesProc.push_back(boost::make_shared<AuctionMessage>(auction));
}

TaskStatus BaseTaskMonitor::getTaskStatus(const int &taskUID) const
{
  TaskStatus taskStatus;
  taskStatus.taskUID = taskUID;
  taskStatus.taskStatus = murdoch_rta::communicator::messages::TaskCancelled;

  std::vector<TaskStatus>::const_iterator status_it = _tasksStatus.begin();

  for (; status_it != _tasksStatus.end(); status_it++)
  {
    if ((*status_it).taskUID == taskUID)
    {
      return (*status_it);
    }
  }

  return taskStatus;
}

bool BaseTaskMonitor::setTaskStatus(const int &taskUID, const TaskStatus &taskStatus)
{
  std::vector<TaskStatus>::iterator status_it = _tasksStatus.begin();

  for (; status_it != _tasksStatus.end(); status_it++)
  {
    if ((*status_it).taskUID == taskUID)
    {
      *status_it = taskStatus;
      return true;
    }
  }

  return false;
}

void BaseTaskMonitor::removeTaskStatus(const int &taskUID)
{
  std::vector<TaskStatus>::iterator status_it = _tasksStatus.begin();

  for (; status_it != _tasksStatus.end();)
  {
    if ((*status_it).taskUID == taskUID)
    {
      status_it = _tasksStatus.erase(status_it);
    }
    else
      status_it++;
  }
}

bool BaseTaskMonitor::isTaskStatusStopTaskMsg(const TaskStatus &taskStatus) const
{
  return taskStatus.taskStatus != murdoch_rta::communicator::messages::TaskContinuing;
}

bool BaseTaskMonitor::stopAuctionTaskMonitoring(const int &taskUID)
{
  // find auction with same taskUID and delete it
  std::vector<AuctionMessagePtr>::iterator auction_it = _auctionMessagesProc.begin();
  for (; auction_it != _auctionMessagesProc.end(); auction_it++)
  {
    if ((*auction_it)->getAuctioningTask()->getTaskUID() == taskUID)
    {
      (*auction_it).reset();
      _auctionMessagesProc.erase(auction_it);
      return true;
    }
  }
  return false;
}

bool BaseTaskMonitor::stopTaskStatusService(const int &taskUID)
{
  bool foundStatus = false;

  // find task status by task UID and delete it
  std::vector<TaskStatus>::iterator status_it = _tasksStatus.begin();
  for (; status_it != _tasksStatus.end(); status_it++)
  {
    if ((*status_it).taskUID == taskUID)
    {
      _tasksStatus.erase(status_it);
      foundStatus = true;
      break;
    }
  }

  // find task status service by task UID and terminate it
  std::vector<TaskStatusService>::iterator srvs_it = _tasksStatusSvrs.begin();
  for (; srvs_it != _tasksStatusSvrs.end(); srvs_it++)
  {
    if ((*srvs_it).taskUID == taskUID)
    {
      (*srvs_it).getTaskStatusSrvServer.shutdown();
      _tasksStatusSvrs.erase(srvs_it);
      return foundStatus && true;
    }
  }

  return false;
}

int BaseTaskMonitor::getTaskRobotID(const int &taskUID) const
{
  // find auction with same taskUID and get the robot that won the auction
  std::vector<AuctionMessagePtr>::const_iterator auction_it = _auctionMessagesProc.begin();

  for (; auction_it != _auctionMessagesProc.end(); auction_it++)
  {
    if (taskUID == (*auction_it)->getAuctioningTask()->getTaskUID())
    {
      return (*auction_it)->getAuctionWinnerID();
    }
  }

  return -1;
}

std::vector<AuctionMessagePtr> BaseTaskMonitor::getAuctionMessagesProc() const
{
  return _auctionMessagesProc;
}

bool BaseTaskMonitor::hasWonAuctionMessageByTaskUID(const int &taskUID, BooleanMessage &wonAuctionMessage)
{
  std::vector<BooleanMessagePtr>::iterator boolMsg_it = _wonAuctionMessages.begin();

  for (; boolMsg_it != _wonAuctionMessages.end(); boolMsg_it++)
  {
    if ((*boolMsg_it)->getTaskUID() == taskUID)
    {
      wonAuctionMessage = BooleanMessage((*boolMsg_it).get());
      (*boolMsg_it).reset();
      _wonAuctionMessages.erase(boolMsg_it);
      return true;
    }
  }
  return false;
}

bool BaseTaskMonitor::hasEndTaskMessageByTaskUID(const int &taskUID, const int &auctionerID)
{
  std::vector<BooleanMessagePtr>::iterator boolMsg_it = _endTaskMessages.begin();

  for (; boolMsg_it != _endTaskMessages.end(); boolMsg_it++)
  {
    if ((*boolMsg_it)->getTaskUID() == taskUID)
    {
      if ((*boolMsg_it)->getIdSender() == auctionerID)
      {
        (*boolMsg_it).reset();
        _endTaskMessages.erase(boolMsg_it);
        return true;
      }
    }
  }
  return false;
}

void BaseTaskMonitor::clearMsgsByTaskUID(const int &taskUID)
{
  std::vector<BooleanMessagePtr>::iterator boolMsg_it = _endTaskMessages.begin();

  for (; boolMsg_it != _endTaskMessages.end();)
  {
    if ((*boolMsg_it)->getTaskUID() == taskUID)
    {
      (*boolMsg_it).reset();
      boolMsg_it = _endTaskMessages.erase(boolMsg_it);
    }
    else
      boolMsg_it++;
  }
}

void BaseTaskMonitor::auctionMsgSubCallback(const murdoch_rta_msgs::Auction::ConstPtr &murdochAuctionMsg)
{
  // push back any incoming auction
  _auctionMessagesUnproc.push_back(boost::make_shared<AuctionMessage>(murdochAuctionMsg));
}

void BaseTaskMonitor::wonAuctionMsgSubCallback(const murdoch_rta_msgs::Boolean::ConstPtr &murdochBoolMsg)
{
  // check if incoming bool msgs is of won auction type
  if (murdochBoolMsg->msgType == murdoch_rta::communicator::messages::WonAuction)
  {
    // TODO : think of a way to delete old bool messages
    // TODO : Check if has an auction with a task with same UID of this message before pushing back to vector
    _wonAuctionMessages.push_back(boost::make_shared<BooleanMessage>(murdochBoolMsg));
  }
  else if (murdochBoolMsg->msgType == murdoch_rta::communicator::messages::TaskCancelled ||
           murdochBoolMsg->msgType == murdoch_rta::communicator::messages::TaskCompleted)
  {
    // this messages come from auctioner - which may cancel the task by itself, so we need to be aware of it and stop
    // monitoring
    // the task
    _endTaskMessages.push_back(boost::make_shared<BooleanMessage>(murdochBoolMsg));
  }
}

bool BaseTaskMonitor::respondTaskStatusRequest(murdoch_rta_msgs::getTaskStatus::Request &req,
                                               murdoch_rta_msgs::getTaskStatus::Response &res)
{
  int auctionerID = req.task.baseMsg.robotSenderId;
  int taskUID = req.task.taskUID;
  std::string taskStrID = req.task.taskIdentifier;

  // Retrieve info from analyzed task status
  //   Set a task type
  TaskStatus taskStatus = getTaskStatus(taskUID);

  murdoch_rta::communicator::messages::BooleanMessagesType msgType = taskStatus.taskStatus;
  //   Define the Response msg
  murdoch_rta::communicator::messages::BooleanMessage responseMsg(-1, auctionerID, taskUID, msgType);

  res.taskStatus = responseMsg.toRosMessage();

  // end communication and monitoring if it was a stop task message
  if (isTaskStatusStopTaskMsg(taskStatus))
  {
    onDeletingAuction(taskUID);
    removeTaskStatus(taskUID);
    stopAuctionTaskMonitoring(taskUID);
    onDeletedAuction(taskUID);
  }

  return true;
}

void BaseTaskMonitor::onAddingAuction(const int &taskUID)
{
}

void BaseTaskMonitor::onAddedAuction(const int &taskUID)
{
}

void BaseTaskMonitor::onDeletingAuction(const int &taskUID)
{
}

void BaseTaskMonitor::onDeletedAuction(const int &taskUID)
{
}
