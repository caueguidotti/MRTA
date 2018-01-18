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

#include "murdoch_rta/auctioneer/states/AuctioningTask.h"
#include "murdoch_rta/auctioneer/StateController.h"

namespace murdoch_rta
{
namespace auctioneer
{
namespace states
{
AuctioningTask::AuctioningTask(StateController *stateController) : State(stateController)
{
}

AuctioningTask::~AuctioningTask()
{
}

bool AuctioningTask::process()
{
  if (!checkForBids())
  {
    communicator::messages::BidMessage auxBid(-1, -1, -1, -1);
    if (getStateController()->getAuction()->getHighestBid(auxBid))
    {
      int auctioneerID = getStateController()->getAuction()->getCreatorID();
      int winnerBidderID = auxBid.getBidderID();
      int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();

      ROS_INFO_STREAM(getStateController()->getAuctioneer()->getLogNameStr()
                      << "Task=" << taskUID << " - Winner Robot=" << winnerBidderID
                      << " - Bid Value=" << auxBid.getBidValue());

      communicator::messages::BooleanMessage wonAuctionBoolMsg(auctioneerID, winnerBidderID, taskUID,
                                                               communicator::messages::WonAuction);

      getStateController()->getAuction()->setAuctionWinnerID(winnerBidderID);
      getStateController()->getAuctioneer()->getTalker()->publishBooleanMsg(wonAuctionBoolMsg.toRosMessage());
      return transitionToWaitingAcknowledgment();
    }
    else
    {
      ROS_INFO_STREAM(getStateController()->getAuctioneer()->getLogNameStr()
                      << "Task=" << getStateController()->getAuction()->getAuctioningTask()->getTaskUID()
                      << " - no bids");
      return false;
    }
  }

  return true;
}

bool AuctioningTask::checkForBids()
{
  if (getStateController()->getAuction()->getAuctioningTask()->getTaskAuctionTimeDeadline() > ros::Time::now())
  {
    if (getStateController()->getAuctioneer()->getListener()->hasBids())
    {
      communicator::messages::BidMessage auxBid =
          getStateController()->getAuctioneer()->getListener()->retrieveBidByTaskUID(
              getStateController()->getAuctioningTask()->getTaskUID());
      if (auxBid.getAuctioneerID() != -1 || auxBid.getBidderID() != -1 || auxBid.getBidValue() != -1 ||
          auxBid.getTaskUID() != -1)
      {
        // since we are sure bid is valid, we append it to the auction
        getStateController()->getAuction()->addBidToAuction(auxBid);
        ROS_INFO_STREAM(getStateController()->getAuctioneer()->getLogNameStr()
                        << "Task=" << auxBid.getTaskUID() << " - Received Bid Robot=" << auxBid.getBidderID()
                        << " - Bid Value=" << auxBid.getBidValue());
      }
    }
    return true;
  }
  return false;
}

StateType AuctioningTask::getStateType() const
{
  return AuctionTaskState;
}
}
}
}
