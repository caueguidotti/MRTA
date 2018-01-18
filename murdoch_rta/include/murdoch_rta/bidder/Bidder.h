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

#include "boost/functional.hpp"
#include "pluginlib/class_loader.h"

#include "murdoch_rta/BaseUtilityCalculator.h"
#include "murdoch_rta/Robot.h"

#include "states/StateType.h"

namespace murdoch_rta
{
namespace bidder
{
class StateController;
typedef boost::scoped_ptr<StateController> StateControllerPtr;

class Bidder : public Robot
{
private:
  StateControllerPtr _stateController;
  resource::ResourcesVecPtr _resources;

  boost::shared_ptr<pluginlib::ClassLoader<murdoch_rta::BaseUtilityCalculator> > _utility_calculator_loader;
  boost::shared_ptr<murdoch_rta::BaseUtilityCalculator> _utilityCalculator;

  void processStateController();
  bool startStateController();
  void setResources(resource::ResourcesVecPtr resources);

public:
  Bidder(ros::NodeHandle *nh, int robotID, std::string utilityPlugin, double replyTimeout);
  ~Bidder();

  void mainLoop();

  bool hasRequiredResources(const resource::ResourcesVec *resourcesNeeded) const;

  const resource::ResourcesVec *getResources() const;
  const resource::ResourcesVecPtr *getPointerToResources() const;

  boost::shared_ptr<murdoch_rta::BaseUtilityCalculator> getUtilityCalculator() const;
};
}
}
