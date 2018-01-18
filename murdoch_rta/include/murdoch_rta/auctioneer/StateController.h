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

#pragma once

#include "murdoch_rta/auctioneer/State.h"

namespace murdoch_rta
{
namespace auctioneer
{
class StateController
{
private:
  Auctioneer *_auctioneer;  // reference to auctioneer object
  State *_currentState;
  int _connectionTrials;

  statePtr _initialState;
  statePtr _processTaskState;
  statePtr _auctionTaskState;
  statePtr _waitAcknowledgmentState;
  statePtr _monitorTaskState;

  task::taskPtr _auctioningTask;
  communicator::messages::auctionPtr _auction;

public:
  StateController(Auctioneer *auctioneer);
  ~StateController();
  Auctioneer *getAuctioneer() const;

  bool processState() const;

  State *getCurrentState() const;
  void setCurrentState(State *currentState);

  State *getInitialState() const;
  State *getProcessTaskState() const;
  State *getAuctionTaskState() const;
  State *getWaitAcknowledgmentState() const;
  State *getMonitorTaskState() const;

  void setAuctioningTask(task::taskPtr &auctioningTask);
  task::taskConstPtr getAuctioningTask() const;
  communicator::messages::AuctionMessage *getAuction() const;
  void setAuction(const communicator::messages::AuctionMessage &auction);
  states::StateType getCurrentStateType() const;
  int getConnectionTrials() const;
  void setConnectionTrials(int connectionTrials);
};

typedef boost::shared_ptr<StateController> stateControllerPtr;
typedef const stateControllerPtr stateControllerConstPtr;
typedef boost::scoped_ptr<std::vector<stateControllerPtr> > stateControllervecPtr;
}
}
