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

#include "murdoch_rta/resource/FloatRange.h"

namespace murdoch_rta
{
namespace resource
{
FloatRange::FloatRange(float lowerBoundValue, float upperBoundValue, std::string resourceIdentifier, bool isIddle)
  : Resource(FloatRangeResource, resourceIdentifier, isIddle)
  , _resourceLowerBound(lowerBoundValue)
  , _resourceUpperBound(upperBoundValue)
{
}

FloatRange::FloatRange(const murdoch_rta_msgs::Resource &resourceMsgRos) : Resource(resourceMsgRos)
{
  _resourceLowerBound = resourceMsgRos.doubleResourceValue1;
  _resourceUpperBound = resourceMsgRos.doubleResourceValue2;
}

FloatRange::FloatRange(const FloatRange &resource)
  : Resource(resource.getResourceType(), resource.getResourceIdentifier(), resource.getIsIddle())
  , _resourceLowerBound(resource.getResourceLowerBound())
  , _resourceUpperBound(resource.getResourceUpperBound())
{
}

FloatRange::~FloatRange()
{
}

float FloatRange::getResourceUpperBound() const
{
  return _resourceUpperBound;
}

void FloatRange::setResourceUpperBound(float value)
{
  _resourceUpperBound = value;
}

float FloatRange::getResourceLowerBound() const
{
  return _resourceLowerBound;
}

void FloatRange::setResourceLowerBound(float value)
{
  _resourceLowerBound = value;
}

murdoch_rta_msgs::Resource FloatRange::toRosMessage() const
{
  murdoch_rta_msgs::Resource res;

  res.baseMsg.messageTime = ros::Time::now();
  res.type = _resourceType;
  res.resourceID = _resourceIdentifier;

  res.doubleResourceValue1 = _resourceLowerBound;
  res.doubleResourceValue2 = _resourceUpperBound;

  return res;
}

float FloatRange::resCompareUpperBound(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::FloatRange> res1Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res1);
  boost::shared_ptr<resource::FloatRange> res2Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res2);

  return FloatRange::resCompareUpperBound(res1Typed, res2Typed);
}

float FloatRange::resCompareUpperBound(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2)
{
  return res1->getResourceUpperBound() - res2->getResourceUpperBound();
}

float FloatRange::resCompareLowerBound(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::FloatRange> res1Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res1);
  boost::shared_ptr<resource::FloatRange> res2Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res2);

  return FloatRange::resCompareLowerBound(res1Typed, res2Typed);
}

float FloatRange::resCompareLowerBound(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2)
{
  return res1->getResourceLowerBound() - res2->getResourceLowerBound();
}

int FloatRange::resCompare(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::FloatRange> res1Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res1);
  boost::shared_ptr<resource::FloatRange> res2Typed = boost::dynamic_pointer_cast<resource::FloatRange>(res2);

  return FloatRange::resCompare(res1Typed, res2Typed);
}

int FloatRange::resCompare(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2)
{
  float lowerCompare = FloatRange::resCompareLowerBound(res1, res2);
  float upperCompare = FloatRange::resCompareUpperBound(res1, res2);

  if (lowerCompare < 0 && upperCompare > 0)
    return 1;
  if (lowerCompare == 0 && upperCompare == 0)
    return 0;
  return -1;
}

void FloatRange::printResource(std::ostream &out) const
{
  out << "\n\tID:\t" << getResourceIdentifier() << "\n\tValue:\t(" << getResourceLowerBound() << ", "
      << getResourceUpperBound() << ")"
      << "\n\tType:\tFloatRange";
}
}
}
