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

#include "murdoch_rta/resource/Resource.h"

namespace murdoch_rta
{
namespace resource
{
class FloatRange : public Resource
{
public:
  FloatRange(float lowerBoundValue, float upperBoundValue, std::string resourceIdentifier, bool isIddle = true);
  FloatRange(const murdoch_rta_msgs::Resource &resourceMsgRos);
  FloatRange(const FloatRange &resource);
  ~FloatRange();

  float getResourceUpperBound() const;
  void setResourceUpperBound(float value);

  float getResourceLowerBound() const;
  void setResourceLowerBound(float value);

  murdoch_rta_msgs::Resource toRosMessage() const;

  static float resCompareUpperBound(ResourcePtr res1, ResourcePtr res2);
  static float resCompareUpperBound(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2);

  static float resCompareLowerBound(ResourcePtr res1, ResourcePtr res2);
  static float resCompareLowerBound(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2);

  static int resCompare(ResourcePtr res1, ResourcePtr res2);
  static int resCompare(boost::shared_ptr<FloatRange> res1, boost::shared_ptr<FloatRange> res2);

protected:
  void printResource(std::ostream &out) const;

private:
  float _resourceUpperBound;
  float _resourceLowerBound;
};
}
}