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

#include "murdoch_rta/bidder/State.h"
#include "murdoch_rta/bidder/StateController.h"

namespace murdoch_rta
{
namespace bidder
{
ros::Time State::getTransitionTime() const
{
  return _transitionTime;
}

State::State(StateController *stateController) : _stateController(stateController)
{
}

State::~State()
{
}

StateController *State::getStateController() const
{
  return _stateController;
}

bool State::transitionToExecutingTask()
{
  if (getStateController()->getCurrentStateType() != states::RenewTaskState)
  {
    throw InvalidStateTransition();
  }
  else if (!getStateController()->getSentAcknowledgeMsgFlag())
  {
    throw InvalidStateTransition();
    return false;
  }

  _stateController->setCurrentState(_stateController->getExecuteTaskState());
  return true;
}

bool State::transitionToRenewingTask()
{
  if (getStateController()->getCurrentStateType() != states::WaitResultState &&
      getStateController()->getCurrentStateType() != states::ExecuteTaskState)
  {
    throw InvalidStateTransition();
  }
  else if (getStateController()->getCurrentStateType() == states::WaitResultState &&
           !getStateController()->getGotWonAuctionMsgFlag())
  {
    throw InvalidStateTransition();
    return false;
  }
  else if (getStateController()->getCurrentStateType() == states::ExecuteTaskState &&
           !getStateController()->getReceivedRenewalMsgFlag())
  {
    throw InvalidStateTransition();
    return false;
  }

  _stateController->setCurrentState(_stateController->getRenewTaskState());
  return true;
}

bool State::transitionToWaitingResults()
{
  if (getStateController()->getCurrentStateType() != states::ProcessAuctionState)
  {
    throw InvalidStateTransition();
  }
  else if (!getStateController()->getMadeBidFlag())
  {
    throw InvalidStateTransition();
    return false;
  }

  _stateController->setCurrentState(_stateController->getWaitResultState());
  return true;
}

void State::updateTransitionTime(const ros::Time &time)
{
  _transitionTime = time;
}
}
}
