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

#include "murdoch_rta/resource/IntegerRange.h"

namespace murdoch_rta
{
namespace resource
{
IntegerRange::IntegerRange(int lowerBoundValue, int upperBoundValue, std::string resourceIdentifier, bool isIddle)
  : Resource(IntegerRangeResource, resourceIdentifier, isIddle)
  , _resourceLowerBound(lowerBoundValue)
  , _resourceUpperBound(upperBoundValue)
{
}

IntegerRange::IntegerRange(const murdoch_rta_msgs::Resource &resourceMsgRos) : Resource(resourceMsgRos)
{
  _resourceLowerBound = resourceMsgRos.integerResourceValue1;
  _resourceUpperBound = resourceMsgRos.integerResourceValue2;
}

IntegerRange::IntegerRange(const IntegerRange &resource)
  : Resource(resource.getResourceType(), resource.getResourceIdentifier(), resource.getIsIddle())
  , _resourceLowerBound(resource.getResourceLowerBound())
  , _resourceUpperBound(resource.getResourceUpperBound())
{
}

IntegerRange::~IntegerRange()
{
}

int IntegerRange::getResourceUpperBound() const
{
  return _resourceUpperBound;
}

void IntegerRange::setResourceUpperBound(int value)
{
  _resourceUpperBound = value;
}

int IntegerRange::getResourceLowerBound() const
{
  return _resourceLowerBound;
}

void IntegerRange::setResourceLowerBound(int value)
{
  _resourceLowerBound = value;
}

murdoch_rta_msgs::Resource IntegerRange::toRosMessage() const
{
  murdoch_rta_msgs::Resource res;

  res.baseMsg.messageTime = ros::Time::now();
  res.type = _resourceType;
  res.resourceID = _resourceIdentifier;

  res.integerResourceValue1 = _resourceLowerBound;
  res.integerResourceValue2 = _resourceUpperBound;

  return res;
}

int IntegerRange::resCompareUpperBound(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::IntegerRange> res1Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res1);
  boost::shared_ptr<resource::IntegerRange> res2Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res2);

  return IntegerRange::resCompareUpperBound(res1Typed, res2Typed);
}

int IntegerRange::resCompareUpperBound(boost::shared_ptr<IntegerRange> res1, boost::shared_ptr<IntegerRange> res2)
{
  return res1->getResourceUpperBound() - res2->getResourceUpperBound();
}

int IntegerRange::resCompareLowerBound(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::IntegerRange> res1Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res1);
  boost::shared_ptr<resource::IntegerRange> res2Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res2);

  return IntegerRange::resCompareLowerBound(res1Typed, res2Typed);
}

int IntegerRange::resCompareLowerBound(boost::shared_ptr<IntegerRange> res1, boost::shared_ptr<IntegerRange> res2)
{
  return res1->getResourceLowerBound() - res2->getResourceLowerBound();
}

int IntegerRange::resCompare(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::IntegerRange> res1Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res1);
  boost::shared_ptr<resource::IntegerRange> res2Typed = boost::dynamic_pointer_cast<resource::IntegerRange>(res2);

  return IntegerRange::resCompare(res1Typed, res2Typed);
}

int IntegerRange::resCompare(boost::shared_ptr<IntegerRange> res1, boost::shared_ptr<IntegerRange> res2)
{
  float lowerCompare = IntegerRange::resCompareLowerBound(res1, res2);
  float upperCompare = IntegerRange::resCompareUpperBound(res1, res2);

  if (lowerCompare < 0 && upperCompare > 0)
    return 1;
  if (lowerCompare == 0 && upperCompare == 0)
    return 0;
  return -1;
}

void IntegerRange::printResource(std::ostream &out) const
{
  out << "\n\tID:\t" << getResourceIdentifier() << "\n\tValue:\t(" << getResourceLowerBound() << ", "
      << getResourceUpperBound() << ")"
      << "\n\tType:\tFloatRange";
}
}
}
