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

#include "murdoch_rta/bidder/states/ProcessingAuction.h"
#include "murdoch_rta/bidder/StateController.h"

namespace murdoch_rta
{
namespace bidder
{
namespace states
{
ProcessingAuction::ProcessingAuction(StateController *stateController) : State(stateController)
{
}

ProcessingAuction::~ProcessingAuction()
{
}

bool ProcessingAuction::process()
{
  ros::Time maxTimeToRetrieveResources = getTransitionTime() + getStateController()->getBidder()->getReplyTimeout();

  int auctionerID = getStateController()->getAuction()->getCreatorID();
  int taskUID = getStateController()->getAuction()->getAuctioningTask()->getTaskUID();
  std::string taskName = getStateController()->getAuction()->getAuctioningTask()->getTaskIdentifier();

  if (retrieveRobotResources())
  {
    const task::Task *auctioningTask = getStateController()->getAuction()->getAuctioningTask();
    if (getStateController()->getBidder()->hasRequiredResources(auctioningTask->getResources()))
    {
      float bidValue = getStateController()->getBidder()->getUtilityCalculator()->calculateUtility(auctioningTask);

      getStateController()->getBidder()->getTalker()->publishBidMsg(bidValue, auctionerID, taskUID);
      getStateController()->setMadeBidFlag(true);

      // before transitioning to waiting results state, clean all previous boolean msgs in case there are any "trash"
      // from previous auctions
      getStateController()->getBidder()->getListener()->deleteBoolMsgsByTaskUID(taskUID);

      return transitionToWaitingResults();
    }
    else
    {
      ROS_INFO_STREAM(getStateController()->getBidder()->getLogNameStr()
                      << "Task=" << auctioningTask->getTaskUID() << " - Resources insufficient - Cannot bid on task.");
      return false;
    }
  }
  else if (ros::Time::now() > maxTimeToRetrieveResources)
  {
    int trials = getStateController()->getConnectionTrials();
    trials++;
    ROS_WARN_STREAM(getStateController()->getBidder()->getLogNameStr()
                    << "Could not contact robot to get Resources. Tried " << trials
                    << " times to establish communication.");
    return false;
  }
  int trials = getStateController()->getConnectionTrials();
  trials++;
  getStateController()->setConnectionTrials(trials);
  return true;
}

bool ProcessingAuction::retrieveRobotResources()
{
  // A container to obtain resources:
  std::vector<murdoch_rta_msgs::Resource> resourcesDefault;
  resourcesDefault.clear();

  // Request robot resources
  if (getStateController()->getBidder()->getTalker()->requestResourcesFromRobot(resourcesDefault))
  {
    // Convert resource messages to murdoch_rta resources
    resource::ResourcesVecPtr robotResources;
    robotResources.reset();
    robotResources =
        boost::shared_ptr<resource::ResourcesVec>(resource::Resource::resourcesVecRosMsgToVecPtr(resourcesDefault));
    // Update bidder resources
    getStateController()->setBidderResources(robotResources);
    return true;
  }
  return false;
}

StateType ProcessingAuction::getStateType() const
{
  return ProcessAuctionState;
}
}
}
}
