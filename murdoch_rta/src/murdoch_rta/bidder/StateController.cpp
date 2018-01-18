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

#include "murdoch_rta/bidder/StateController.h"
#include "murdoch_rta/bidder/states/ExecutingTask.h"
#include "murdoch_rta/bidder/states/ProcessingAuction.h"
#include "murdoch_rta/bidder/states/RenewingTask.h"
#include "murdoch_rta/bidder/states/WaitingResult.h"

namespace murdoch_rta
{
namespace bidder
{
StateController::StateController(Bidder *bidder, communicator::messages::auctionPtr auction,
                                 boost::function<void(resource::ResourcesVecPtr)> setResources)
  : _bidder(bidder), _setResources(setResources)
{
  if (!_auction || !_auction.get())
    UndefinedBehavior("Auction received in bidder state controller is NULL");

  _receivedRenewalMsgFlag = false;
  _gotWonAuctionMsgFlag = false;
  _madeBidFlag = false;
  _auction = boost::make_shared<communicator::messages::AuctionMessage>(auction);

  _processAuctionState.reset(new states::ProcessingAuction(this));
  _waitResultState.reset(new states::WaitingResult(this));
  _executeTaskState.reset(new states::ExecutingTask(this));
  _renewTaskState.reset(new states::RenewingTask(this));

  setCurrentState(_processAuctionState.get());
}

StateController::~StateController()
{
}

states::StateType StateController::getCurrentStateType() const
{
  return _currentState->getStateType();
}

void StateController::setCurrentState(State *currentState)
{
  _receivedRenewalMsgFlag = false;
  _sentAcknowledgeMsgFlag = false;
  _connectionTrials = 0;
  _currentState = currentState;
  _currentState->updateTransitionTime(ros::Time::now());
}

void StateController::setBidderResources(resource::ResourcesVecPtr resources)
{
  // TODO check current state
  _setResources(resources);
}

bool StateController::processState() const
{
  return _currentState->process();
}

Bidder *StateController::getBidder() const
{
  return _bidder;
}

communicator::messages::AuctionMessage *StateController::getAuction() const
{
  return _auction.get();
}

bool StateController::getMadeBidFlag() const
{
  return _madeBidFlag;
}

void StateController::setMadeBidFlag(bool madeBidFlag)
{
  _madeBidFlag = madeBidFlag;
}

void StateController::setGotWonAuctionMsgFlag(bool gotWonAuctionMsgFlag)
{
  _gotWonAuctionMsgFlag = gotWonAuctionMsgFlag;
}

void StateController::setSentAcknowledgeMsgFlag(bool sentAcknowledgeMsgFlag)
{
  _sentAcknowledgeMsgFlag = sentAcknowledgeMsgFlag;
}

void StateController::setReceivedRenewalMsgFlag(bool receivedRenewalMsgFlag)
{
  _receivedRenewalMsgFlag = receivedRenewalMsgFlag;
}

bool StateController::getSentAcknowledgeMsgFlag() const
{
  return _sentAcknowledgeMsgFlag;
}

bool StateController::getReceivedRenewalMsgFlag() const
{
  return _receivedRenewalMsgFlag;
}

State *StateController::getProcessAuctionState() const
{
  return _processAuctionState.get();
}

State *StateController::getWaitResultState() const
{
  return _waitResultState.get();
}

State *StateController::getExecuteTaskState() const
{
  return _executeTaskState.get();
}

State *StateController::getRenewTaskState() const
{
  return _renewTaskState.get();
}

bool StateController::getGotWonAuctionMsgFlag() const
{
  return _gotWonAuctionMsgFlag;
}

int StateController::getConnectionTrials() const
{
  return _connectionTrials;
}

void StateController::setConnectionTrials(const int &connectionTrials)
{
  _connectionTrials = connectionTrials;
}
}
}
