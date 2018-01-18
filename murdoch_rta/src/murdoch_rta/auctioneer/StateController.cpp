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

#include "murdoch_rta/auctioneer/StateController.h"
#include "murdoch_rta/auctioneer/states/AuctioningTask.h"
#include "murdoch_rta/auctioneer/states/Initial.h"
#include "murdoch_rta/auctioneer/states/MonitoringTask.h"
#include "murdoch_rta/auctioneer/states/ProcessingTask.h"
#include "murdoch_rta/auctioneer/states/WaitingAcknowledgment.h"

namespace murdoch_rta
{
namespace auctioneer
{
StateController::StateController(Auctioneer *auctioneer) : _auctioneer(auctioneer)
{
  _initialState.reset(new states::Initial(this));
  _processTaskState.reset(new states::ProcessingTask(this));
  _auctionTaskState.reset(new states::AuctioningTask(this));
  _waitAcknowledgmentState.reset(new states::WaitingAcknowledgment(this));
  _monitorTaskState.reset(new states::MonitoringTask(this));

  setCurrentState(_initialState.get());
  _auction.reset();
}

StateController::~StateController()
{
}

State *StateController::getCurrentState() const
{
  return _currentState;
}

void StateController::setCurrentState(State *currentState)
{
  _connectionTrials = 0;
  _currentState = currentState;
  _currentState->setStateTransitionTime(ros::Time::now());
}

bool StateController::processState() const
{
  return _currentState->process();
}

Auctioneer *StateController::getAuctioneer() const
{
  return _auctioneer;
}

State *StateController::getInitialState() const
{
  return _initialState.get();
}

State *StateController::getProcessTaskState() const
{
  return _processTaskState.get();
}

State *StateController::getAuctionTaskState() const
{
  return _auctionTaskState.get();
}

State *StateController::getWaitAcknowledgmentState() const
{
  return _waitAcknowledgmentState.get();
}

State *StateController::getMonitorTaskState() const
{
  return _monitorTaskState.get();
}

void StateController::setAuctioningTask(task::taskPtr &auctioningTask)
{
  if (_auctioningTask)
  {
    throw AuctioningTaskRedefinition();
  }
  _auctioningTask = auctioningTask;
}

task::taskConstPtr StateController::getAuctioningTask() const
{
  return _auctioningTask;
}

communicator::messages::AuctionMessage *StateController::getAuction() const
{
  return _auction.get();
}

void StateController::setAuction(const communicator::messages::AuctionMessage &auction)
{
  if (_currentState->getStateType() != states::ProcessTaskState)
    throw UnallowedAuctionDefinition();
  if (_auction)
    throw AuctionRedefinition();

  _auction.reset(new communicator::messages::AuctionMessage(auction));
}

states::StateType StateController::getCurrentStateType() const
{
  return _currentState->getStateType();
}

int StateController::getConnectionTrials() const
{
  return _connectionTrials;
}

void StateController::setConnectionTrials(int connectionTrials)
{
  _connectionTrials = connectionTrials;
}
}
}
