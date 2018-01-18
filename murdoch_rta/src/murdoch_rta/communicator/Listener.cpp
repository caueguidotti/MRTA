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

#include "murdoch_rta/communicator/Listener.h"

namespace murdoch_rta
{
namespace communicator
{
Listener::Listener(ros::NodeHandle *nh, const int &robotId, const bool &isAuctioner, const ros::Duration &replyTimeOut)
  : _robotID(robotId)
  , _nh(nh)
  , _isAuctioner(isAuctioner)
  , _replyTimeout(replyTimeOut)
  , _bidBuffer(new std::vector<messages::bidPtr>)
  , _taskBuffer(new std::vector<task::taskPtr>)
  , _auctionBuffer(new std::vector<messages::auctionPtr>)
  , _boolMsgBuffer(new std::vector<messages::BooleanMessagePtr>)
{
  // TODO make it a parameter or based on timeout for reply
  //  currently timer is set at a specific at a fixed rate of 1 minute
  int timerTriggerInSecs = 60;
  _cleanUpTimer = _nh->createTimer(ros::Duration(timerTriggerInSecs), &Listener::cleanUpMessages, this);
  initSubscribers();
}

Listener::~Listener()
{
  _subAuction.shutdown();
  _subTask.shutdown();
  _subBid.shutdown();
  _subBoolMsg.shutdown();
}

void Listener::initSubscribers()
{
  if (_isAuctioner)
  {
    _subTask = _nh->subscribe("/tasks", 10, &Listener::taskSubCallback, this);
    _subBid = _nh->subscribe("/bids", 100, &Listener::bidSubCallback, this);
  }
  else
  {
    _subAuction = _nh->subscribe("/auctions", 10, &Listener::auctionSubCallback, this);
  }
  _subBoolMsg = _nh->subscribe("/boolMsgs", 100, &Listener::boolMsgSubCallback, this);
}

void Listener::auctionSubCallback(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos)
{
  int receiverId = auctionMsgRos->baseMsg.robotReceiverId;
  if (receiverId == -1 || receiverId == _robotID)
  {
    _auctionBuffer->push_back(boost::make_shared<messages::AuctionMessage>(auctionMsgRos));
  }
}

void Listener::taskSubCallback(const murdoch_rta_msgs::Task::ConstPtr &taskMsgRos)
{
  int receiverId = taskMsgRos->baseMsg.robotReceiverId;
  if (receiverId == -1 || receiverId == _robotID)
  {
    _taskBuffer->push_back(boost::make_shared<task::Task>(taskMsgRos));
  }
}

void Listener::bidSubCallback(const murdoch_rta_msgs::Bid::ConstPtr &bidMsgRos)
{
  int receiverId = bidMsgRos->baseMsg.robotReceiverId;
  if (receiverId == _robotID)
  {
    _bidBuffer->push_back(boost::make_shared<messages::BidMessage>(bidMsgRos));
  }
}

void Listener::boolMsgSubCallback(const murdoch_rta_msgs::Boolean::ConstPtr &boolMsgRos)
{
  int receiverId = boolMsgRos->baseMsg.robotReceiverId;
  if (receiverId == _robotID)
    _boolMsgBuffer->push_back(boost::make_shared<messages::BooleanMessage>(boolMsgRos));
}

void Listener::cleanUpMessages(const ros::TimerEvent &timerEvent)
{
  double elapsedTime = timerEvent.current_expected.toSec() - timerEvent.last_expected.toSec();

  if (_bidBuffer->size())
  {
    // bids are deleted when they live longer than one timer cycle
    std::vector<messages::bidPtr>::iterator it = _bidBuffer->begin();
    for (; it != _bidBuffer->end();)
    {
      if (ros::Time::now() > ((*it)->getMessageSentTime() + ros::Duration(elapsedTime)))
      {
        (*it).reset();
        it = _bidBuffer->erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  if (_auctionBuffer->size())
  {
    // auctions are deleted when they live longer than auction deadline
    std::vector<messages::auctionPtr>::iterator it = _auctionBuffer->begin();
    for (; it != _auctionBuffer->end();)
    {
      if (ros::Time::now() > (*it)->getAuctionTaskAuctionTimeDeadline())
      {
        (*it).reset();
        it = _auctionBuffer->erase(it);
      }
      else
        ++it;
    }
  }

  if (_taskBuffer->size())
  {
    // tasks are deleted when they live longer than expected auction deadline
    std::vector<task::taskPtr>::iterator it = _taskBuffer->begin();
    for (; it != _taskBuffer->end();)
    {
      if (ros::Time::now() > (*it)->getTaskAuctionTimeDeadline())
      {
        (*it).reset();
        it = _taskBuffer->erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  if (_boolMsgBuffer->size())
  {
    std::vector<messages::BooleanMessagePtr>::iterator it = _boolMsgBuffer->begin();
    for (; it != _boolMsgBuffer->end();)
    {
      if (ros::Time::now() > ((*it)->getMessageSentTime() + _replyTimeout))
      {
        (*it).reset();
        it = _boolMsgBuffer->erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
}

bool Listener::hasTaskInBuffer() const
{
  return _taskBuffer->size() > 0;
}

bool Listener::hasAuctionInBuffer() const
{
  return _auctionBuffer->size() > 0;
}

int Listener::getRobotID() const
{
  return _robotID;
}

ros::NodeHandle *Listener::getNh() const
{
  return _nh;
}

void Listener::setNh(ros::NodeHandle *nh)
{
  _nh = nh;
}

messages::auctionPtr Listener::retrieveAuction()
{
  messages::auctionPtr auctionToCaller;
  auctionToCaller.reset();

  if (!_auctionBuffer)
    return auctionToCaller;
  else if (_auctionBuffer->size() == 0)
    return auctionToCaller;

  auctionToCaller = boost::make_shared<messages::AuctionMessage>((*_auctionBuffer->begin()));

  (*_auctionBuffer->begin()).reset();
  _auctionBuffer->erase(_auctionBuffer->begin());

  return auctionToCaller;
}

// to change bool to pointer, copy pointer is always better, let boost handle everything for me !!!!!
task::Task Listener::retrieveHighestPriorityTask()
{
  if (!_taskBuffer)
    throw UndefinedBehavior("Cannot retrieve a task from listener if buffer was not initialized!");
  if (_taskBuffer->empty())
    throw UndefinedBehavior("Cannot retrieve a task if there is no task within buffer. Run hasTaskInBuffer() first!");

  std::vector<task::taskPtr>::iterator it = _taskBuffer->begin();
  std::vector<task::taskPtr>::iterator highestTaskIt = it;

  while (it != _taskBuffer->end())
  {
    if ((*it)->getPriority() > (*highestTaskIt)->getPriority())
      highestTaskIt = it;
    ++it;
  }

  task::Task highestTask = *((*highestTaskIt).get());
  (*highestTaskIt).reset();
  _taskBuffer->erase(highestTaskIt);

  return highestTask;
}

bool Listener::hasBids() const
{
  if (!_bidBuffer)
    throw UndefinedBehavior("Cannot retrieve a bid from listener if buffer was not initialized!");

  return !_bidBuffer->empty();
}

messages::BidMessage Listener::retrieveBidByTaskUID(const int &taskUID)
{
  messages::BidMessage bid(-1, -1, -1, -1);

  std::vector<messages::bidPtr>::iterator it_bidBuffer = _bidBuffer->begin();
  while (it_bidBuffer != _bidBuffer->end())
  {
    if ((*it_bidBuffer)->getTaskUID() == taskUID)  // has a bid with refered task UID
    {
      bid = *((*it_bidBuffer).get());   // copies bid
      (*it_bidBuffer).reset();          // reset bid pointer
      _bidBuffer->erase(it_bidBuffer);  // removes bid pointer from vector
      break;
    }
    else
      it_bidBuffer++;
  }

  return bid;
}

bool Listener::retrieveAcknowlegmentMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID)
{
  return retrieveBoolMsg(acknowledgmentMsgPredicate, boolMsg, senderID, taskUID);
}

bool Listener::retrieveTaskProgressMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID)
{
  return retrieveBoolMsg(taskProgressMsgPredicate, boolMsg, senderID, taskUID);
}

bool Listener::retrieveWonAuctionMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID)
{
  return retrieveBoolMsg(wonAuctionMsgPredicate, boolMsg, senderID, taskUID);
}

void Listener::deleteBoolMsgsByTaskUID(const int &taskUID)
{
  std::vector<messages::BooleanMessagePtr>::iterator boolMsgs_it = _boolMsgBuffer->begin();

  for (; boolMsgs_it != _boolMsgBuffer->end();)
  {
    if ((*boolMsgs_it)->getTaskUID() == taskUID)
    {
      (*boolMsgs_it).reset();
      boolMsgs_it = _boolMsgBuffer->erase(boolMsgs_it);
    }
    else
      boolMsgs_it++;
  }
}

void Listener::deleteBidMsgsByTaskUID(const int &taskUID)
{
  std::vector<messages::bidPtr>::iterator bids_it = _bidBuffer->begin();

  for (; bids_it != _bidBuffer->end();)
  {
    if ((*bids_it)->getTaskUID() == taskUID)
    {
      (*bids_it).reset();
      bids_it = _bidBuffer->erase(bids_it);
    }
    else
      bids_it++;
  }
}

void Listener::deleteAllAuctions()
{
  if (_auctionBuffer->size())
  {
    // auctions are deleted when they live longer than auction deadline
    std::vector<messages::auctionPtr>::iterator it = _auctionBuffer->begin();
    for (; it != _auctionBuffer->end();)
    {
      (*it).reset();
      it = _auctionBuffer->erase(it);
    }
  }
}

bool Listener::generalMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                   const int &taskUID, const messages::BooleanMessagesType &msgType)
{
  return boolMsg->getMessageType() == msgType && (boolMsg->getIdSender() == senderID || senderID == -1) &&
         boolMsg->getTaskUID() == taskUID;
}

bool Listener::acknowledgmentMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                          const int &taskUID)
{
  return generalMsgPredicate(boolMsg, senderID, taskUID, messages::Acknowledge);
}

bool Listener::wonAuctionMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                      const int &taskUID)
{
  return generalMsgPredicate(boolMsg, senderID, taskUID, messages::WonAuction);
}

bool Listener::taskProgressMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                        const int &taskUID)
{
  return generalMsgPredicate(boolMsg, senderID, taskUID, messages::TaskCancelled) ||
         generalMsgPredicate(boolMsg, senderID, taskUID, messages::TaskCompleted) ||
         generalMsgPredicate(boolMsg, senderID, taskUID, messages::TaskContinuing);
}
}
}
