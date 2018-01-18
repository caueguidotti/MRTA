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

#include "murdoch_rta/bidder/states/ExecutingTask.h"
#include "murdoch_rta/bidder/StateController.h"

namespace murdoch_rta
{
namespace bidder
{
namespace states
{
ExecutingTask::ExecutingTask(StateController *stateController) : State(stateController)
{
}

ExecutingTask::~ExecutingTask()
{
}

bool ExecutingTask::process()
{
  communicator::messages::BooleanMessage taskProgressMsg(-1, -1, -1, communicator::messages::TaskContinuing);

  int auctionerID = getStateController()->getAuction()->getCreatorID();
  int bidderID = getStateController()->getBidder()->getRobotID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();

  if (getStateController()->getBidder()->getListener()->retrieveTaskProgressMsg(taskProgressMsg, auctionerID, taskUID))
  {
    getStateController()->setReceivedRenewalMsgFlag(true);
    return processTaskProgressMsg(taskProgressMsg);
  }
  else if (ros::Time::now() > (getTransitionTime() + getStateController()->getBidder()->getReplyTimeout()))
  {
    ROS_WARN_STREAM(getStateController()->getBidder()->getLogNameStr()
                    << "Task=" << taskProgressMsg.getTaskUID()
                    << " - No Task Progress - Stopping Acknowledge messages!");
    return false;
  }
  return true;
}

bool ExecutingTask::processTaskProgressMsg(const communicator::messages::BooleanMessage &taskProgressMsg)
{
  int bidderID = getStateController()->getBidder()->getRobotID();
  int auctionerID = getStateController()->getAuction()->getCreatorID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();
  std::string taskName = getStateController()->getAuction()->getAuctioningTask()->getTaskIdentifier();

  switch (taskProgressMsg.getMessageType())
  {
    case communicator::messages::TaskCancelled:
      ROS_INFO_STREAM(getStateController()->getBidder()->getLogNameStr() << "Task=" << taskProgressMsg.getTaskUID()
                                                                         << " - Cancelled");
      return false;
      break;
    case communicator::messages::TaskCompleted:
      ROS_INFO_STREAM(getStateController()->getBidder()->getLogNameStr() << "Task=" << taskProgressMsg.getTaskUID()
                                                                         << " - Completed");
      return false;
      break;
    case communicator::messages::TaskContinuing:
      return transitionToRenewingTask();
      break;
    default:
    {
      std::stringstream errSS;
      errSS << "Received a Task Progress message of unexpected type: " << taskProgressMsg.getMessageType();
      throw std::logic_error(errSS.str());
      break;
    }
  }
}

StateType ExecutingTask::getStateType() const
{
  return ExecuteTaskState;
}
}
}
}
