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

#include "murdoch_rta/auctioneer/State.h"
#include "murdoch_rta/auctioneer/StateController.h"

namespace murdoch_rta
{
namespace auctioneer
{
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

bool State::transitionToInitialState()
{
}

bool State::transitionToProcessingTask()
{
  if (_stateController->getCurrentState()->getStateType() != states::InitialState)
  {
    throw InvalidStateTransition();
  }

  if (_stateController->getAuctioningTask())
  {
    _stateController->setCurrentState(_stateController->getProcessTaskState());
    return true;
  }
  else
  {
    throw InvalidStateTransition();
    return false;
  }
}

bool State::transitionToAuctioningTask()
{
  if (_stateController->getCurrentState()->getStateType() != states::ProcessTaskState)
  {
    throw InvalidStateTransition();
  }

  if (!_stateController->getAuction())
  {
    throw InvalidStateTransition();
    return false;
  }

  if (!_stateController->getAuction()->getAuctioningTask())
  {
    throw InvalidStateTransition();
    return false;
  }

  _stateController->setCurrentState(_stateController->getAuctionTaskState());
  return true;
}

bool State::transitionToWaitingAcknowledgment()
{
  // TODO think about something to distinguish - further check - these state for correct transition
  if (_stateController->getCurrentState()->getStateType() != states::AuctionTaskState &&
      _stateController->getCurrentState()->getStateType() != states::MonitorTaskState)
  {
    throw InvalidStateTransition();
  }

  if (_stateController->getAuction()->getAuctionWinnerID() == -1)
  {
    throw InvalidStateTransition();
  }

  _stateController->setCurrentState(_stateController->getWaitAcknowledgmentState());
  return true;
}

bool State::transitionToMonitoringTask()
{
  if (_stateController->getCurrentState()->getStateType() != states::WaitAcknowledgmentState)
  {
    throw InvalidStateTransition();
  }

  _stateController->setCurrentState(_stateController->getMonitorTaskState());
  return true;
}

ros::Time State::getStateTransitionTime() const
{
  return stateTransitionTime;
}

void State::setStateTransitionTime(const ros::Time &value)
{
  stateTransitionTime = value;
}
}
}
