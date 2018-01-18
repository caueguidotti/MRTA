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

#include "murdoch_rta/resource/Resource.h"
#include "murdoch_rta/resource/Discrete.h"
#include "murdoch_rta/resource/Float.h"
#include "murdoch_rta/resource/FloatRange.h"
#include "murdoch_rta/resource/Integer.h"
#include "murdoch_rta/resource/IntegerRange.h"

namespace murdoch_rta
{
namespace resource
{
Resource::Resource(ResourceType resourceType, std::string resourceIdentifier, bool isIddle)
  : _resourceType(resourceType), _resourceIdentifier(resourceIdentifier), _isIddle(isIddle)
{
}

Resource::Resource(const murdoch_rta_msgs::Resource &resourceMsgRos)
{
  int resourceTypeNum = resourceMsgRos.type;
  _resourceType = static_cast<ResourceType>(resourceTypeNum);
  _resourceIdentifier = resourceMsgRos.resourceID;
}

Resource::~Resource()
{
}

std::string Resource::getResourceIdentifier() const
{
  return _resourceIdentifier;
}

ResourceType Resource::getResourceType() const
{
  return _resourceType;
}

bool Resource::getIsIddle() const
{
  return _isIddle;
}

void Resource::setIsIddle(const bool &value)
{
  _isIddle = value;
}

ResourcesVecPtr Resource::resourcesVecRosMsgToVecPtr(const std::vector<murdoch_rta_msgs::Resource> &resources)
{
  ResourcesVecPtr resourcesVecPtr;
  resourcesVecPtr.reset(new ResourcesVec);

  for (unsigned i = 0; i != resources.size(); i++)
  {
    switch (resources[i].type)
    {
      case resource::DiscreteResource:
      {
        resource::Discrete res(resources[i]);
        resourcesVecPtr->push_back(boost::make_shared<resource::Discrete>(res));
        break;
      }
      case resource::IntegerResource:
      {
        resource::Integer res(resources[i]);
        resourcesVecPtr->push_back(boost::make_shared<resource::Integer>(res));
        break;
      }
      case resource::FloatResource:
      {
        resource::Float res(resources[i]);
        resourcesVecPtr->push_back(boost::make_shared<resource::Float>(res));
        break;
      }
      case resource::IntegerRangeResource:
      {
        resource::IntegerRange res(resources[i]);
        resourcesVecPtr->push_back(boost::make_shared<resource::IntegerRange>(res));
        break;
      }
      case resource::FloatRangeResource:
      {
        resource::FloatRange res(resources[i]);
        resourcesVecPtr->push_back(boost::make_shared<resource::FloatRange>(res));
        break;
      }
      default:
      {
        std::stringstream errMsg;
        errMsg << "Resource (AKA '" << resources[i].resourceID << "') "
               << "in received message has a invalid type number: '" << resources[i].type << "'";
        throw UnrecognizedMessage(errMsg.str());
        break;
      }
    }
  }

  return resourcesVecPtr;
}

ResourcePtr Resource::resourceFromConfig(const std::string &configLoc, const ros::NodeHandle &privateNh)
{
  // returns null pointer is resource is invalid:
  ResourcePtr readResource;
  readResource.reset();

  if (!privateNh.hasParam(configLoc))
    return readResource;

  // stringstream to parameter key
  std::stringstream paramSS;
  // resource variables
  std::string resourceType, resourceName;
  float resourceValue1(0), resourceValue2(0);

  // key to retrieve resource type
  paramSS << configLoc << "/"
          << "type";
  // check if key is present
  if (!privateNh.hasParam(paramSS.str()))
  {
    ROS_WARN_STREAM(configLoc << " found - Resource Type (" << paramSS.str() << ") not informed. Skipping resource!");
    return readResource;
  }
  // retrieve resource type
  privateNh.param<std::string>(paramSS.str(), resourceType, "");
  // check if resource is valid
  if (!(boost::iequals("discrete", resourceType) || boost::iequals("integer", resourceType) ||
        boost::iequals("float", resourceType) || boost::iequals("integerrange", resourceType) ||
        boost::iequals("floatrange", resourceType)))
  {
    ROS_WARN_STREAM(configLoc << " found - Resource Type (" << resourceType << ") is invalid. Skipping resource!");
    return readResource;
  }

  // key to resource name
  paramSS.str("");
  paramSS << configLoc << "/"
          << "name";
  // check if key is present
  if (!privateNh.hasParam(paramSS.str()))
  {
    ROS_WARN_STREAM(configLoc << " found - Resource Name (" << paramSS.str() << ") not informed. Skipping resource!");
    return readResource;
  }
  // retrieve resource name
  privateNh.param<std::string>(paramSS.str(), resourceName, "");

  // if resource is type discrete
  if (boost::iequals(resourceType, "discrete"))
  {
    readResource = boost::make_shared<Discrete>(Discrete(true, resourceName, false));
    return readResource;
  }

  // decide if resource has one or two values
  if (boost::iequals("integer", resourceType) || boost::iequals("float", resourceType))
  {
    // 1 value
    // key value checking
    paramSS.str("");
    paramSS << configLoc << "/"
            << "value";
    if (!privateNh.hasParam(paramSS.str()))
    {
      paramSS.str("");
      paramSS << configLoc << "/"
              << "value1";
      if (!privateNh.hasParam(paramSS.str()))
      {
        paramSS.str("");
        paramSS << configLoc << "/"
                << "value";
        ROS_WARN_STREAM(configLoc << " found - Resource Value (" << paramSS.str() << ") not informed. Skipping "
                                                                                     "resource!");
        return readResource;
      }
    }
    // retrieve value
    privateNh.param<float>(paramSS.str(), resourceValue1, 0.0);
    // define either resources
    if (boost::iequals("integer", resourceType))
      readResource = boost::make_shared<Integer>(Integer(resourceValue1, resourceName, false));
    else  // resource type is Float
      readResource = boost::make_shared<Float>(Float(resourceValue1, resourceName, false));
    return readResource;
  }

  // resource is either integer range or float range
  // key value1 checking
  paramSS.str("");
  paramSS << configLoc << "/"
          << "value1";
  if (!privateNh.hasParam(paramSS.str()))
  {
    ROS_WARN_STREAM(configLoc << " found - Resource Value (" << paramSS.str() << ") not informed. Skipping resource!");
    return readResource;
  }
  // retrieve value1
  privateNh.param<float>(paramSS.str(), resourceValue1, 0.0);

  // key value2 checking
  paramSS.str("");
  paramSS << configLoc << "/"
          << "value2";
  if (!privateNh.hasParam(paramSS.str()))
  {
    ROS_WARN_STREAM(configLoc << " found - Resource Value (" << paramSS.str() << ") not informed. Skipping resource!");
    return readResource;
  }
  // retrieve value2
  privateNh.param<float>(paramSS.str(), resourceValue2, 0.0);

  // define either resourceS
  if (boost::iequals("integerrange", resourceType))
    readResource = boost::make_shared<IntegerRange>(IntegerRange(resourceValue1, resourceValue2, resourceName, false));
  else  // resource type is Float
    readResource = boost::make_shared<FloatRange>(FloatRange(resourceValue1, resourceValue2, resourceName, false));
  return readResource;
}

ResourcesVecPtr Resource::resourcesFromConfig(const std::string &configLoc, const ros::NodeHandle &privateNh)
{
  ResourcesVecPtr resourcesVecPtr;
  resourcesVecPtr.reset(new ResourcesVec);

  int counter(0);
  std::string resourceParamStr = configLoc + "0";

  while (privateNh.hasParam(resourceParamStr))
  {
    ResourcePtr resource = resourceFromConfig(resourceParamStr, privateNh);
    if (resource.get())
    {
      switch (resource->getResourceType())
      {
        case resource::DiscreteResource:
        {
          resourcesVecPtr->push_back(
              boost::make_shared<resource::Discrete>(*boost::dynamic_pointer_cast<resource::Discrete>(resource)));
          break;
        }
        case resource::IntegerResource:
        {
          resourcesVecPtr->push_back(
              boost::make_shared<resource::Integer>(*boost::dynamic_pointer_cast<resource::Integer>(resource)));
          break;
        }
        case resource::FloatResource:
        {
          resourcesVecPtr->push_back(
              boost::make_shared<resource::Float>(*boost::dynamic_pointer_cast<resource::Float>(resource)));
          break;
        }
        case resource::IntegerRangeResource:
        {
          resourcesVecPtr->push_back(boost::make_shared<resource::IntegerRange>(
              *boost::dynamic_pointer_cast<resource::IntegerRange>(resource)));
          break;
        }
        case resource::FloatRangeResource:
        {
          resourcesVecPtr->push_back(
              boost::make_shared<resource::FloatRange>(*boost::dynamic_pointer_cast<resource::FloatRange>(resource)));
          break;
        }
      }
    }
    resourceParamStr = configLoc + boost::lexical_cast<std::string>(++counter);
    if (!privateNh.hasParam(resourceParamStr))
      break;
  }

  return resourcesVecPtr;
}

bool Resource::areCompatible(ResourcePtr res1, ResourcePtr res2)
{
  // true if has same resource name and type - overloaded '='
  return *(res1.get()) == *(res2.get());
}
}
}
