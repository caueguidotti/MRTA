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

#include "murdoch_rta/MurdochExceptions.h"
#include "murdoch_rta/resource/Type.h"
#include "murdoch_rta_msgs/Resource.h"
#include "ros/ros.h"

#include "boost/algorithm/string.hpp"

namespace murdoch_rta
{
namespace resource
{
class Resource;
// TODO : check if can be a scoped ptr, probably can
typedef boost::shared_ptr<resource::Resource> ResourcePtr;
typedef boost::shared_ptr<const resource::Resource> ResourceConstPtr;
typedef std::vector<ResourcePtr> ResourcesVec;
typedef std::vector<ResourceConstPtr> ConstResourcesVec;
typedef boost::shared_ptr<ResourcesVec> ResourcesVecPtr;

class Resource
{
public:
  Resource(const murdoch_rta_msgs::Resource &resourceMsgRos);
  virtual ~Resource() = 0;

  ResourceType getResourceType() const;

  std::string getResourceIdentifier() const;

  bool getIsIddle() const;
  void setIsIddle(const bool &value);

  virtual murdoch_rta_msgs::Resource toRosMessage() const = 0;
  static ResourcesVecPtr resourcesVecRosMsgToVecPtr(const std::vector<murdoch_rta_msgs::Resource> &resources);
  static ResourcePtr resourceFromConfig(const std::string &configLoc, const ros::NodeHandle &privateNh);
  static ResourcesVecPtr resourcesFromConfig(const std::string &configLoc, const ros::NodeHandle &privateNh);

  friend std::ostream &operator<<(std::ostream &out, const Resource &resource)
  {
    resource.printResource(out);
    return out;
  }

  friend std::ostream &operator<<(std::ostream &out, const ResourcesVecPtr &resources)
  {
    out << resources.get();
    return out;
  }

  friend std::ostream &operator<<(std::ostream &out, const resource::ResourcesVec *resources)
  {
    if (resources)
    {
      int resCounter = 0;
      int resCount = resources->size();
      ResourcesVec::const_iterator it = resources->begin();
      for (; it != resources->end(); it++)
      {
        out << "  Resource " << ++resCounter << "/" << resCount;
        (*it)->printResource(out);
        out << "\n";
      }
    }
    return out;
  }

  static bool areCompatible(ResourcePtr res1, ResourcePtr res2);

protected:
  Resource(ResourceType resourceType, std::string resourceIdentifier, bool isIddle = true);
  ResourceType _resourceType;
  std::string _resourceIdentifier;
  bool _isIddle;

  virtual void printResource(std::ostream &out) const = 0;

private:
  bool operator==(const Resource &res)
  {
    return (boost::iequals(_resourceIdentifier, res.getResourceIdentifier()) && _resourceType == res.getResourceType());
  }
};
}
}
