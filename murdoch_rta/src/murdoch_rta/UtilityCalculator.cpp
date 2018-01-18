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

#include "murdoch_rta/UtilityCalculator.h"
#include "murdoch_rta/BaseUtilityCalculator.h"
#include "murdoch_rta/bidder/Bidder.h"
#include "pluginlib/class_list_macros.h"

void murdoch_rta::DefaultUtilityCalculator::initialize(murdoch_rta::bidder::Bidder const* bidder)
{
  BaseUtilityCalculator::initialize(bidder);
}

double murdoch_rta::DefaultUtilityCalculator::calculateUtility(const murdoch_rta::task::Task* task)
{
  double calculatedUtility = 0;

  const murdoch_rta::resource::ResourcesVec* taskResources = task->getResources();
  const murdoch_rta::resource::ResourcesVec* robotResources = _bidder->getResources();

  murdoch_rta::resource::ResourcesVec::const_iterator it_robot_res = robotResources->begin();
  for (; it_robot_res != robotResources->end(); it_robot_res++)
  {
    murdoch_rta::resource::ResourcesVec::const_iterator it_task_res = taskResources->begin();
    for (; it_task_res != taskResources->end(); it_task_res++)
    {
      if (murdoch_rta::resource::Resource::areCompatible(*it_task_res, *it_robot_res))
      {
        switch ((*it_task_res)->getResourceType())
        {
          case murdoch_rta::resource::DiscreteResource:
            calculatedUtility += 1.0;
            break;

          case murdoch_rta::resource::IntegerResource:
            calculatedUtility += murdoch_rta::resource::Integer::resCompare((*it_robot_res), (*it_task_res));
            break;

          case murdoch_rta::resource::FloatResource:
            calculatedUtility += murdoch_rta::resource::Float::resCompare((*it_robot_res), (*it_task_res));
            break;

          case murdoch_rta::resource::IntegerRangeResource:
            calculatedUtility -=
                murdoch_rta::resource::IntegerRange::resCompareLowerBound((*it_robot_res), (*it_task_res));
            calculatedUtility +=
                murdoch_rta::resource::IntegerRange::resCompareUpperBound((*it_robot_res), (*it_task_res));
            break;

          case murdoch_rta::resource::FloatRangeResource:
            calculatedUtility -=
                murdoch_rta::resource::FloatRange::resCompareLowerBound((*it_robot_res), (*it_task_res));
            calculatedUtility +=
                murdoch_rta::resource::FloatRange::resCompareUpperBound((*it_robot_res), (*it_task_res));
            break;
        }
      }
    }
  }

  calculatedUtility /= taskResources->size();

  return calculatedUtility;
}

PLUGINLIB_DECLARE_CLASS(murdoch_rta, default_utility_calculator, murdoch_rta::DefaultUtilityCalculator,
                        murdoch_rta::BaseUtilityCalculator)
