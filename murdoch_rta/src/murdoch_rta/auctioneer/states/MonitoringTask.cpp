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

#include "murdoch_rta/auctioneer/states/MonitoringTask.h"
#include "murdoch_rta/auctioneer/StateController.h"

namespace murdoch_rta
{
namespace auctioneer
{
namespace states
{
MonitoringTask::MonitoringTask(StateController *stateController) : State(stateController)
{
}

MonitoringTask::~MonitoringTask()
{
}

bool MonitoringTask::process()
{
  int auctioneerID = getStateController()->getAuction()->getCreatorID();
  int winnerID = getStateController()->getAuction()->getAuctionWinnerID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();
  ros::Time maxTimeToRetrieveTaskStatus =
      getStateTransitionTime() + getStateController()->getAuctioneer()->getReplyTimeout();

  communicator::messages::BooleanMessage taskStatus(winnerID, auctioneerID, taskUID,
                                                    communicator::messages::TaskCancelled);
  if (getStateController()->getAuctioneer()->getTalker()->requestTaskStatus(
          getStateController()->getAuction()->getAuctioningTask(), taskStatus))
  {
    return processTaskProgressMsg(taskStatus);
  }
  else if (ros::Time::now() > maxTimeToRetrieveTaskStatus)
  {
    int trials = getStateController()->getConnectionTrials();
    trials++;
    ROS_WARN_STREAM(getStateController()->getAuctioneer()->getLogNameStr()
                    << "Could not contact Monitor to retrieve Task Status. Tried " << trials
                    << " times to establish communication."
                    << " Topic: " << task::Task::taskStatusTopic(taskUID));
    return false;
  }
  int trials = getStateController()->getConnectionTrials();
  trials++;
  getStateController()->setConnectionTrials(trials);
  return true;
}

bool MonitoringTask::processTaskProgressMsg(const communicator::messages::BooleanMessage &taskProgressMsg)
{
  int auctioneerID = getStateController()->getAuction()->getCreatorID();
  int winnerID = getStateController()->getAuction()->getAuctionWinnerID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();

  switch (taskProgressMsg.getMessageType())
  {
    case communicator::messages::TaskCancelled:
    {
      ROS_INFO_STREAM(getStateController()->getAuctioneer()->getLogNameStr() << "Task=" << taskUID << " - Cancelled");
      communicator::messages::BooleanMessage bidderInformTaskStatus(auctioneerID, winnerID, taskUID,
                                                                    communicator::messages::TaskCancelled);
      getStateController()->getAuctioneer()->getTalker()->publishBooleanMsg(bidderInformTaskStatus.toRosMessage());
      return false;
    }
    case communicator::messages::TaskCompleted:
    {
      ROS_INFO_STREAM(getStateController()->getAuctioneer()->getLogNameStr() << "Task=" << taskUID << " - Completed");
      communicator::messages::BooleanMessage bidderInformTaskStatus(auctioneerID, winnerID, taskUID,
                                                                    communicator::messages::TaskCompleted);
      getStateController()->getAuctioneer()->getTalker()->publishBooleanMsg(bidderInformTaskStatus.toRosMessage());
      return false;
    }
    case communicator::messages::TaskContinuing:
    {
      communicator::messages::BooleanMessage bidderInformTaskStatus(auctioneerID, winnerID, taskUID,
                                                                    communicator::messages::TaskContinuing);
      getStateController()->getAuctioneer()->getTalker()->publishBooleanMsg(bidderInformTaskStatus.toRosMessage());
      return transitionToWaitingAcknowledgment();
    }
    default:
    {
      std::stringstream errSS;
      errSS << "Received a Task Progress message of unexpected type: " << taskProgressMsg.getMessageType();
      throw std::logic_error(errSS.str());
    }
  }
}

StateType MonitoringTask::getStateType() const
{
  return MonitorTaskState;
}
}
}
}
