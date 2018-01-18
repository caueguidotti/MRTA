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

#include "murdoch_rta/resource/Float.h"

namespace murdoch_rta
{
namespace resource
{
Float::Float(float value, std::string resourceIdentifier, bool isIddle)
  : Resource(FloatResource, resourceIdentifier, isIddle), _resourceValue(value)
{
}

Float::Float(const murdoch_rta_msgs::Resource &resourceMsgRos) : Resource(resourceMsgRos)
{
  _resourceValue = resourceMsgRos.doubleResourceValue1;
}

Float::Float(const Float &resource)
  : Resource(resource.getResourceType(), resource.getResourceIdentifier(), resource.getIsIddle())
  , _resourceValue(resource.getResourceValue())
{
}

Float::~Float()
{
}

float Float::getResourceValue() const
{
  return _resourceValue;
}

void Float::setResourceValue(float value)
{
  _resourceValue = value;
}

murdoch_rta_msgs::Resource Float::toRosMessage() const
{
  murdoch_rta_msgs::Resource res;

  res.baseMsg.messageTime = ros::Time::now();
  res.type = _resourceType;
  res.resourceID = _resourceIdentifier;

  res.doubleResourceValue1 = _resourceValue;

  return res;
}

float Float::resCompare(ResourcePtr res1, ResourcePtr res2)
{
  boost::shared_ptr<resource::Float> res1Typed = boost::dynamic_pointer_cast<resource::Float>(res1);
  boost::shared_ptr<resource::Float> res2Typed = boost::dynamic_pointer_cast<resource::Float>(res2);

  return Float::resCompare(res1Typed, res2Typed);
}

float Float::resCompare(boost::shared_ptr<Float> res1, boost::shared_ptr<Float> res2)
{
  return res1->getResourceValue() - res2->getResourceValue();
}

void Float::printResource(std::ostream &out) const
{
  out << "\n\tID:\t" << getResourceIdentifier() << "\n\tValue:\t" << getResourceValue() << "\n\tType:\tFloat";
}
}
}
