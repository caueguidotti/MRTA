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

#include "murdoch_rta/communicator/messages/BidMessage.h"

namespace murdoch_rta
{
namespace communicator
{
namespace messages
{
BidMessage::BidMessage(int bidderID, int AuctioneerID, float bidValue, int taskUID, ros::Time bidTime)
  : Message(bidderID, AuctioneerID, bidTime), _bidValue(bidValue), _taskUID(taskUID)
{
}

BidMessage::BidMessage(const BidMessage &bid)
  : Message(bid.getIdSender(), bid.getIdReceiver(), bid.getMessageSentTime())
  , _bidValue(bid.getBidValue())
  , _taskUID(bid.getTaskUID())
{
}

BidMessage::BidMessage(const murdoch_rta_msgs::Bid::ConstPtr &bidMsgRos)
  : Message(bidMsgRos->baseMsg), _bidValue(bidMsgRos->bidValue), _taskUID(bidMsgRos->taskUID)
{
}

BidMessage::~BidMessage()
{
}

float BidMessage::getBidValue() const
{
  return _bidValue;
}

int BidMessage::getBidderID() const
{
  return Message::getIdSender();
}

int BidMessage::getAuctioneerID() const
{
  return Message::getIdReceiver();
}

ros::Time BidMessage::getBidTime() const
{
  return Message::getMessageSentTime();
}

int BidMessage::getTaskUID() const
{
  return _taskUID;
}

murdoch_rta_msgs::Bid BidMessage::toRosMessage() const
{
  murdoch_rta_msgs::Bid bidMsg;

  bidMsg.baseMsg = Message::toRosMessage();
  bidMsg.bidValue = _bidValue;
  bidMsg.taskUID = _taskUID;

  return bidMsg;
}
}
}
}
