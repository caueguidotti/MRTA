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

#include "murdoch_rta/auctioneer/states/WaitingAcknowledgment.h"
#include "murdoch_rta/auctioneer/StateController.h"

namespace murdoch_rta
{
namespace auctioneer
{
namespace states
{
WaitingAcknowledgment::WaitingAcknowledgment(StateController *stateController) : State(stateController)
{
}

WaitingAcknowledgment::~WaitingAcknowledgment()
{
}

bool WaitingAcknowledgment::process()
{
  communicator::messages::BooleanMessage ackMsg(-1, -1, -1, communicator::messages::Acknowledge);
  int winnerID = getStateController()->getAuction()->getAuctionWinnerID();
  int auctioneerID = getStateController()->getAuction()->getCreatorID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();

  if (getStateController()->getAuctioneer()->getListener()->retrieveAcknowlegmentMsg(ackMsg, winnerID, taskUID))
  {
    return transitionToMonitoringTask();
  }
  else if (ros::Time::now() > (getStateTransitionTime() + getStateController()->getAuctioneer()->getReplyTimeout()))
  {
    ROS_WARN_STREAM(getStateController()->getAuctioneer()->getLogNameStr()
                    << "Task=" << taskUID << " - No Acknowledge from Robot=" << winnerID);

    communicator::messages::BooleanMessage taskCancelled(auctioneerID, winnerID, taskUID,
                                                         communicator::messages::TaskCancelled);
    getStateController()->getAuctioneer()->getTalker()->publishBooleanMsg(taskCancelled.toRosMessage());
    return false;
  }
  return true;
}

StateType WaitingAcknowledgment::getStateType() const
{
  return WaitAcknowledgmentState;
}
}
}
}
