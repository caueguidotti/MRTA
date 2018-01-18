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

#include "murdoch_rta/bidder/states/RenewingTask.h"
#include "murdoch_rta/bidder/StateController.h"

namespace murdoch_rta
{
namespace bidder
{
namespace states
{
RenewingTask::RenewingTask(StateController *stateController) : State(stateController)
{
}

RenewingTask::~RenewingTask()
{
}

bool RenewingTask::process()
{
  int bidderID = getStateController()->getBidder()->getRobotID();
  int auctionerID = getStateController()->getAuction()->getCreatorID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();
  std::string taskName = getStateController()->getAuction()->getAuctioningTask()->getTaskIdentifier();

  const task::Task *const task = getStateController()->getAuction()->getAuctioningTask();
  ros::Time timeToWaitBeforeRenewal = ros::Time::now() + getStateController()->getBidder()->getReplyTimeout();
  ros::Time maxTimeToSetTask = getTransitionTime() + getStateController()->getBidder()->getReplyTimeout();

  if (getStateController()->getBidder()->getTalker()->requestSetRobotTask(task, timeToWaitBeforeRenewal))
  {
    getStateController()->setSentAcknowledgeMsgFlag(true);
    communicator::messages::BooleanMessage ackMsg(bidderID, auctionerID, taskUID, communicator::messages::Acknowledge);
    getStateController()->getBidder()->getTalker()->publishBooleanMsg(ackMsg.toRosMessage());
    return transitionToExecutingTask();
  }
  else if (ros::Time::now() > maxTimeToSetTask)
  {
    int trials = getStateController()->getConnectionTrials();
    trials++;
    ROS_WARN_STREAM(getStateController()->getBidder()->getLogNameStr()
                    << "Could not contact robot to set Task. Tried " << trials << " times to establish communication.");
    return false;
  }
  int trials = getStateController()->getConnectionTrials();
  trials++;
  getStateController()->setConnectionTrials(trials);
  return true;
}

StateType RenewingTask::getStateType() const
{
  return RenewTaskState;
}
}
}
}
