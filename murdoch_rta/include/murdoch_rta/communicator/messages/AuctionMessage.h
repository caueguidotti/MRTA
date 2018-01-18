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

#include "murdoch_rta/communicator/messages/BidMessage.h"
#include "murdoch_rta/communicator/messages/Message.h"
#include "murdoch_rta/task/Task.h"

#include "murdoch_rta_msgs/Auction.h"

namespace murdoch_rta
{
namespace communicator
{
namespace messages
{
class AuctionMessage;  // forward decl
typedef boost::shared_ptr<AuctionMessage> auctionPtr;
typedef const boost::shared_ptr<AuctionMessage> auctionConstPtr;

class AuctionMessage : public Message
{
private:
  int _creatorID;
  int _auctionWinnerID;
  ros::Time _auctionStartTime;

  task::taskPtr _auctioningTask;
  boost::scoped_ptr<std::vector<bidPtr> > _auctionBids;

  void auctionInit(const int &creatorID, const int &auctionWinnerID, const ros::Time &auctionStartTime,
                   const task::Task *auctioningTask, const std::vector<bidPtr> *auctionBids);

public:
  AuctionMessage(const AuctionMessage &auction);

  AuctionMessage(const AuctionMessage *auction);

  AuctionMessage(const auctionPtr auction);

  AuctionMessage(const int &creatorID, const task::taskPtr auctioningTask,
                 const ros::Time &auctionStartTime = ros::Time::now());

  AuctionMessage(const int &creatorID, const task::Task *auctioningTask,
                 const ros::Time &auctionStartTime = ros::Time::now());

  AuctionMessage(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos);

  AuctionMessage(const murdoch_rta_msgs::Auction::ConstPtr &auctionMsgRos, const ros::Time auctionStarTime);

  ~AuctionMessage();

  ros::Time getAuctionStartTime() const;
  void setAuctionStartTime(const ros::Time &auctionStartTime);

  const task::Task *getAuctioningTask() const;
  int getCreatorID() const;
  ros::Time getAuctionTaskAuctionTimeDeadline() const;

  murdoch_rta_msgs::Auction toRosMessage() const;

  void addBidToAuction(const BidMessage &bid);
  bool getHighestBid(BidMessage &highestBid) const;
  void clearBids();

  int getAuctionWinnerID() const;
  void setAuctionWinnerID(const int &auctionWinnerID);
  std::vector<bidPtr> *getAuctionBids() const;
};
}
}
}
