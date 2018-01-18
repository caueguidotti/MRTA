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

#include "utils/task_generator/BaseTaskGenerator.h"

BaseTaskGenerator::BaseTaskGenerator(ros::NodeHandle *nh) : _nh(nh)
{
  initSubscribers();
}

BaseTaskGenerator::~BaseTaskGenerator()
{
}

void BaseTaskGenerator::initSubscribers()
{
  _subBoolMsg = _nh->subscribe("/boolMsgs", 100, &BaseTaskGenerator::wonAuctionMsgSubCallback, this);
  _subAuctionMsg = _nh->subscribe("/auctions", 10, &BaseTaskGenerator::auctionMsgSubCallback, this);
}

void BaseTaskGenerator::processTaskGenerator()
{
  processUnprocAuctions();
  processProcAuctions();
  generateTask();
}

void BaseTaskGenerator::processUnprocAuctions()
{
  // a aux vector with tasks that are being auctioned:
  std::vector<ExecutingTasksInfo> auctioningTasks;

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

        addProcAuction(*auction_it);

        // Remove auction from Unproc vector
        (*auction_it).reset();
        auction_it = _auctionMessagesUnproc.erase(auction_it);

        // Raise "event"
        onAddedAuction(auctionTaskUID);
      }
      else
      {
        // under auction task
        ExecutingTasksInfo taskInfo;
        taskInfo.taskUID = (*auction_it)->getAuctioningTask()->getTaskUID();
        taskInfo.taskName = (*auction_it)->getAuctioningTask()->getTaskIdentifier();
        auctioningTasks.push_back(taskInfo);

        auction_it++;
      }
    }
  }

  _auctioningTasks = auctioningTasks;
}

void BaseTaskGenerator::processProcAuctions()
{
  // a aux vector with tasks that are being executed:
  std::vector<ExecutingTasksInfo> executingTasks;

  // iterate through finished auctions and find if tasks have finished or not
  std::vector<AuctionMessagePtr>::iterator auction_it = _auctionMessagesProc.begin();
  for (; auction_it != _auctionMessagesProc.end();)
  {
    int taskUID = (*auction_it)->getAuctioningTask()->getTaskUID();

    if (hasEndTaskMessageByTaskUID(taskUID, (*auction_it)->getCreatorID()))
    {
      auction_it = _auctionMessagesProc.erase(auction_it);
    }
    else
    {
      // undergoing task
      ExecutingTasksInfo taskInfo;
      taskInfo.taskUID = (*auction_it)->getAuctioningTask()->getTaskUID();
      taskInfo.robotUID = (*auction_it)->getAuctionWinnerID();
      taskInfo.taskName = (*auction_it)->getAuctioningTask()->getTaskIdentifier();
      executingTasks.push_back(taskInfo);
      auction_it++;
    }
  }

  _executingTasks = executingTasks;  // renew executing tasks vector
}

void BaseTaskGenerator::addProcAuction(AuctionMessagePtr auction)
{
  int robotID = auction->getAuctionWinnerID();
  int taskUID = auction->getAuctioningTask()->getTaskUID();

  // clean up variables for TaskUID
  clearMsgsByTaskUID(taskUID);

  // Now add auction to Proc vector
  _auctionMessagesProc.push_back(boost::make_shared<AuctionMessage>(auction));
}

int BaseTaskGenerator::getTaskRobotID(const int &taskUID) const
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

bool BaseTaskGenerator::hasWonAuctionMessageByTaskUID(const int &taskUID, BooleanMessage &wonAuctionMessage)
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

bool BaseTaskGenerator::hasEndTaskMessageByTaskUID(const int &taskUID, const int &auctionerID)
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

void BaseTaskGenerator::clearMsgsByTaskUID(const int &taskUID)
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

void BaseTaskGenerator::auctionMsgSubCallback(const murdoch_rta_msgs::Auction::ConstPtr &murdochAuctionMsg)
{
  // push back any incoming auction
  _auctionMessagesUnproc.push_back(boost::make_shared<AuctionMessage>(murdochAuctionMsg));
}

void BaseTaskGenerator::wonAuctionMsgSubCallback(const murdoch_rta_msgs::Boolean::ConstPtr &murdochBoolMsg)
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

void BaseTaskGenerator::onAddingAuction(const int &taskUID)
{
}

void BaseTaskGenerator::onAddedAuction(const int &taskUID)
{
}

void BaseTaskGenerator::onDeletingAuction(const int &taskUID)
{
}

void BaseTaskGenerator::onDeletedAuction(const int &taskUID)
{
}

std::vector<ExecutingTasksInfo> BaseTaskGenerator::getAuctioningTasks() const
{
  return _auctioningTasks;
}

std::vector<ExecutingTasksInfo> BaseTaskGenerator::getExecutingTasks() const
{
  return _executingTasks;
}
