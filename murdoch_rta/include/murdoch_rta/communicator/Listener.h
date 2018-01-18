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

#include "murdoch_rta_msgs/Auction.h"
#include "murdoch_rta_msgs/BaseMessage.h"
#include "murdoch_rta_msgs/Bid.h"
#include "murdoch_rta_msgs/Boolean.h"
#include "murdoch_rta_msgs/Resource.h"
#include "murdoch_rta_msgs/Task.h"

#include "murdoch_rta/communicator/messages/AuctionMessage.h"
#include "murdoch_rta/communicator/messages/BidMessage.h"
#include "murdoch_rta/communicator/messages/BooleanMessage.h"

#include "murdoch_rta/task/Task.h"
#include "ros/ros.h"

#include <boost/smart_ptr/scoped_ptr.hpp>

namespace murdoch_rta
{
namespace communicator
{
class Listener;
typedef boost::scoped_ptr<Listener> listenerPtr;

class Listener
{
private:
  ros::NodeHandle *_nh;
  const int _robotID;
  const bool _isAuctioner;
  const ros::Duration _replyTimeout;

  boost::shared_ptr<std::vector<task::taskPtr> > _taskBuffer;
  boost::shared_ptr<std::vector<messages::auctionPtr> > _auctionBuffer;
  boost::shared_ptr<std::vector<messages::bidPtr> > _bidBuffer;
  boost::shared_ptr<std::vector<messages::BooleanMessagePtr> > _boolMsgBuffer;

  ros::Subscriber _subAuction;
  ros::Subscriber _subTask;
  ros::Subscriber _subBid;
  ros::Subscriber _subBoolMsg;

  // specifies a timer to cleanup unnused old messages
  ros::Timer _cleanUpTimer;

  void auctionSubCallback(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos);
  void taskSubCallback(const murdoch_rta_msgs::Task::ConstPtr &taskMsgRos);
  void bidSubCallback(const murdoch_rta_msgs::Bid::ConstPtr &bidMsgRos);
  void boolMsgSubCallback(const murdoch_rta_msgs::Boolean::ConstPtr &boolMsgRos);

  void cleanUpMessages(const ros::TimerEvent &timerEvent);

  template <typename BoolPredicate>
  bool retrieveBoolMsg(const BoolPredicate &checker, messages::BooleanMessage &boolMsg, const int &senderID,
                       const int &taskUID);

  static bool acknowledgmentMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                         const int &taskUID);
  static bool wonAuctionMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                     const int &taskUID);
  static bool taskProgressMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                       const int &taskUID);
  static bool generalMsgPredicate(const messages::BooleanMessage *const boolMsg, const int &senderID,
                                  const int &taskUID, const messages::BooleanMessagesType &msgType);

public:
  Listener(ros::NodeHandle *nh, const int &robotId, const bool &isAuctioner, const ros::Duration &replyTimeOut);
  ~Listener();

  void initSubscribers();

  int getRobotID() const;
  ros::NodeHandle *getNh() const;
  void setNh(ros::NodeHandle *nh);

  bool hasTaskInBuffer() const;
  bool hasAuctionInBuffer() const;

  messages::auctionPtr retrieveAuction();

  task::Task retrieveHighestPriorityTask();

  bool hasBids() const;
  messages::BidMessage retrieveBidByIndex(int bufferInd);
  messages::BidMessage retrieveBidByTaskUID(const int &taskUID);

  bool retrieveAcknowlegmentMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID);
  bool retrieveTaskProgressMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID);
  bool retrieveWonAuctionMsg(messages::BooleanMessage &boolMsg, int senderID, int taskUID);
  void deleteBoolMsgsByTaskUID(const int &taskUID);
  void deleteBidMsgsByTaskUID(const int &taskUID);
  void deleteAllAuctions();
};

template <typename BoolPredicate>
bool Listener::retrieveBoolMsg(const BoolPredicate &checker, messages::BooleanMessage &boolMsg, const int &senderID,
                               const int &taskUID)
{
  if (!_boolMsgBuffer)
    throw UndefinedBehavior("Cannot retrieve a Bool Message from listener if buffer was not initialized!");

  std::vector<messages::BooleanMessagePtr>::iterator it = _boolMsgBuffer->begin();
  for (; it != _boolMsgBuffer->end();)
  {
    if (!(*it).get())
      it = _boolMsgBuffer->erase(it);
    else
    {
      if (checker((*it).get(), senderID, taskUID))
      {
        boolMsg = messages::BooleanMessage((*it).get());
        (*it).reset();
        _boolMsgBuffer->erase(it);
        return true;
      }
      it++;
    }
  }
  return false;
}
}
}
