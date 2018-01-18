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

#include "murdoch_rta/communicator/messages/AuctionMessage.h"

namespace murdoch_rta
{
namespace communicator
{
namespace messages
{
void AuctionMessage::auctionInit(const int &creatorID, const int &auctionWinnerID, const ros::Time &auctionStartTime,
                                 const task::Task *auctioningTask, const std::vector<bidPtr> *auctionBids)
{
  _creatorID = creatorID;
  _auctionWinnerID = auctionWinnerID;
  _auctionStartTime = auctionStartTime;
  _auctioningTask = boost::make_shared<task::Task>(auctioningTask);

  // since bids are scoped_ptr, need to copy one by one
  _auctionBids.reset(new std::vector<bidPtr>);
  std::vector<bidPtr>::const_iterator bid_it = auctionBids->begin();
  for (; bid_it != auctionBids->end(); bid_it++)
  {
    _auctionBids->push_back(boost::make_shared<BidMessage>(*(*bid_it)));
  }
}

AuctionMessage::AuctionMessage(const AuctionMessage &auction) : Message(auction.getCreatorID(), -1)
{
  auctionInit(auction.getCreatorID(), auction.getAuctionWinnerID(), auction.getAuctionStartTime(),
              auction.getAuctioningTask(), auction.getAuctionBids());
}

AuctionMessage::AuctionMessage(const AuctionMessage *auction) : Message(auction->getCreatorID(), -1)
{
  auctionInit(auction->getCreatorID(), auction->getAuctionWinnerID(), auction->getAuctionStartTime(),
              auction->getAuctioningTask(), auction->getAuctionBids());
}

AuctionMessage::AuctionMessage(const auctionPtr auction) : Message(auction->getCreatorID(), -1)
{
  auctionInit(auction->getCreatorID(), auction->getAuctionWinnerID(), auction->getAuctionStartTime(),
              auction->getAuctioningTask(), auction->getAuctionBids());
}

AuctionMessage::AuctionMessage(const int &creatorID, const task::taskPtr auctioningTask,
                               const ros::Time &auctionStartTime)
  : Message(creatorID, -1), _creatorID(creatorID), _auctionStartTime(auctionStartTime)
{
  _auctionBids.reset(new std::vector<bidPtr>);
  _auctioningTask = boost::make_shared<task::Task>(auctioningTask.get());
}

AuctionMessage::AuctionMessage(const int &creatorID, const task::Task *auctioningTask,
                               const ros::Time &auctionStartTime)
  : Message(creatorID, -1), _creatorID(creatorID), _auctionStartTime(auctionStartTime)
{
  _auctionBids.reset(new std::vector<bidPtr>);
  _auctioningTask = boost::make_shared<task::Task>(auctioningTask);
}

AuctionMessage::AuctionMessage(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos)
  : Message(auctionMsgRos->baseMsg)
  , _creatorID(auctionMsgRos->baseMsg.robotSenderId)
  , _auctionStartTime(auctionMsgRos->baseMsg.messageTime)
{
  _auctionBids.reset(new std::vector<bidPtr>);
  _auctioningTask = boost::make_shared<task::Task>(auctionMsgRos->auctionTask);
}

AuctionMessage::AuctionMessage(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos,
                               const ros::Time auctionStarTime)
  : Message(auctionMsgRos->baseMsg)
  , _creatorID(auctionMsgRos->baseMsg.robotSenderId)
  , _auctionStartTime(auctionStarTime)
{
  _auctionBids.reset(new std::vector<bidPtr>);
  _auctioningTask = boost::make_shared<task::Task>(auctionMsgRos->auctionTask);
}

AuctionMessage::~AuctionMessage()
{
}

const task::Task *AuctionMessage::getAuctioningTask() const
{
  return _auctioningTask.get();
}

murdoch_rta_msgs::Auction AuctionMessage::toRosMessage() const
{
  murdoch_rta_msgs::Auction auctionMsg;

  auctionMsg.baseMsg = Message::toRosMessage();

  if (!_auctioningTask)
    throw murdoch_rta::CastFailed("Cannot cast Auction to ROS Message, pointer to task is NULL");
  auctionMsg.auctionTask = _auctioningTask->toRosMessage();

  return auctionMsg;
}

void AuctionMessage::addBidToAuction(const BidMessage &bid)
{
  std::vector<bidPtr>::iterator it = _auctionBids->begin();
  for (; it != _auctionBids->end();)
  {
    if ((*it)->getBidderID() == bid.getBidderID())
    {
      std::stringstream ss;
      ss << "Received another bid from Robot ID " << bid.getBidderID() << " for Task UID " << bid.getTaskUID()
         << ". Auctioner can't decide winner!"
         << " Do you have two robots with equal robot ID?";
      throw UndefinedAuctionBehavior(ss.str());
    }
    ++it;
  }
  _auctionBids->push_back(boost::make_shared<BidMessage>(bid));
}

// Receives a BidMessage as arguemnt passed by referece (highestBid), this argument will be filled with
//  the highest bid found within this Auction Message.
//  If no bid is found within this Auction Message, this function returns false.
//  If a bid is found, returns True, and fills highestBid with the highest bid value Bid Message
bool AuctionMessage::getHighestBid(BidMessage &highestBid) const
{
  if (_auctionBids->size() == 0)
    return false;

  unsigned highestBidValueIndex = 0;
  for (unsigned i = i; i < _auctionBids->size(); ++i)
  {
    if (_auctionBids->at(highestBidValueIndex)->getBidValue() < _auctionBids->at(i)->getBidValue())
      highestBidValueIndex = i;
  }

  highestBid = BidMessage(*(_auctionBids->at(highestBidValueIndex).get()));
  return true;
}

void AuctionMessage::clearBids()
{
}

ros::Time AuctionMessage::getAuctionStartTime() const
{
  return _auctionStartTime;
}

void AuctionMessage::setAuctionStartTime(const ros::Time &auctionStartTime)
{
  _auctionStartTime = auctionStartTime;
}

int AuctionMessage::getCreatorID() const
{
  return _creatorID;
}

ros::Time AuctionMessage::getAuctionTaskAuctionTimeDeadline() const
{
  return _auctioningTask->getTaskAuctionTimeDeadline();
}

int AuctionMessage::getAuctionWinnerID() const
{
  return _auctionWinnerID;
}

void AuctionMessage::setAuctionWinnerID(const int &auctionWinnerID)
{
  _auctionWinnerID = auctionWinnerID;
}

std::vector<bidPtr> *AuctionMessage::getAuctionBids() const
{
  return _auctionBids.get();
}
}
}
}
