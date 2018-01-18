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

#include "murdoch_rta/bidder/Bidder.h"
#include "murdoch_rta/bidder/StateController.h"

namespace murdoch_rta
{
namespace bidder
{
Bidder::Bidder(ros::NodeHandle *nh, int robotID, std::string utilityPlugin, double replyTimeout)
  : Robot(nh, robotID, false, replyTimeout)
{
  _resources.reset(new std::vector<resource::ResourcePtr>);
  _stateController.reset();

  _utility_calculator_loader = boost::make_shared<pluginlib::ClassLoader<murdoch_rta::BaseUtilityCalculator> >(
      "murdoch_rta", "murdoch_rta::BaseUtilityCalculator");

  _utilityCalculator = _utility_calculator_loader->createInstance(utilityPlugin);
  _utilityCalculator->initialize(this);
}

Bidder::~Bidder()
{
}

void Bidder::mainLoop()
{
  if (!_stateController)
    startStateController();
  else
    processStateController();
}

boost::shared_ptr<murdoch_rta::BaseUtilityCalculator> Bidder::getUtilityCalculator() const
{
  return _utilityCalculator;
}

void Bidder::processStateController()
{
  if (_stateController)
  {
    if (!_stateController->processState())
    {
      getListener()->deleteAllAuctions();
      _stateController.reset();
    }
  }
}

bool Bidder::startStateController()
{
  if (!_stateController)
  {
    if (_listener->hasAuctionInBuffer())
    {
      _stateController.reset(
          new StateController(this, _listener->retrieveAuction(), boost::bind(&Bidder::setResources, this, _1)));
      return true;
    }
  }
  return false;
}

bool Bidder::hasRequiredResources(const resource::ResourcesVec *resourcesNeeded) const
{
  if (!_resources.get())
    return false;

  if (_resources->size() == 0)
    return false;

  std::vector<resource::ResourcePtr>::const_iterator it_neededRes = resourcesNeeded->begin();
  for (; it_neededRes != resourcesNeeded->end(); ++it_neededRes)
  {
    bool foundRes = false;

    std::vector<resource::ResourcePtr>::iterator it_myRes = _resources->begin();
    for (; it_myRes != _resources->end(); ++it_myRes)
    {
      if (!(*it_myRes).get())
        throw std::logic_error("Trying to access a resource that was not initialized");

      std::string myResID = (*it_myRes).get()->getResourceIdentifier();
      resource::ResourceType myResType = (*it_myRes).get()->getResourceType();

      if (boost::iequals((*it_neededRes)->getResourceIdentifier(), myResID))
      {
        if (myResType == (*it_neededRes)->getResourceType())
        {
          switch (myResType)
          {
            case resource::DiscreteResource:
              foundRes = true;
              break;
            case resource::IntegerResource:
              foundRes = resource::Integer::resCompare((*it_myRes), (*it_neededRes)) >= 0;
              break;
            case resource::FloatResource:
              foundRes = resource::Float::resCompare((*it_myRes), (*it_neededRes)) >= 0;
              break;
            case resource::IntegerRangeResource:
              foundRes = resource::IntegerRange::resCompare((*it_myRes), (*it_neededRes)) >= 0;
              break;
            case resource::FloatRangeResource:
              foundRes = resource::FloatRange::resCompare((*it_myRes), (*it_neededRes)) >= 0;
              break;
            default:
              std::stringstream ss;
              ss << "Type of resource with ID " << myResID << " in robot(" << getRobotID() << ") is invalid";
              throw UndefinedBehavior(ss.str());
          }
        }
        else
        {
          std::stringstream ss;
          ss << "Resource with ID " << myResID << " in robot(" << getRobotID() << ") is of type " << myResType
             << " while comparing resource with same ID is of type " << (*it_neededRes)->getResourceType();
          throw UndefinedBehavior(ss.str());
        }
      }
    }
    if (!foundRes)
    {
      return false;
    }
  }

  return true;
}

resource::ResourcesVec const *Bidder::getResources() const
{
  return _resources.get();
}

void Bidder::setResources(resource::ResourcesVecPtr resources)
{
  _resources = resources;
}
}
}
