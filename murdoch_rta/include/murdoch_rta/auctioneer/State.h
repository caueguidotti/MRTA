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

#include "murdoch_rta/auctioneer/Auctioneer.h"
#include "murdoch_rta/auctioneer/states/StateType.h"

namespace murdoch_rta
{
namespace auctioneer
{
// forward declaration
class StateController;

class State
{
private:
  StateController *_stateController;

protected:
  State(StateController *stateController);

  // transitionToInitialState TODO: move transition functions to statecontroller
  bool transitionToInitialState();
  bool transitionToProcessingTask();
  bool transitionToAuctioningTask();
  bool transitionToMonitoringTask();
  bool transitionToWaitingAcknowledgment();

  ros::Time stateTransitionTime;

public:
  ~State();

  virtual bool process() = 0;
  StateController *getStateController() const;

  virtual states::StateType getStateType() const = 0;

  void setStateTransitionTime(const ros::Time &value);
  ros::Time getStateTransitionTime() const;
};

// TODO make this StatePtr instead of statePtr
typedef boost::scoped_ptr<State> statePtr;
}
}
