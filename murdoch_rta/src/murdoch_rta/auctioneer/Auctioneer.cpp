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

#include "murdoch_rta/auctioneer/Auctioneer.h"
#include "murdoch_rta/auctioneer/StateController.h"

#include <boost/move/utility.hpp>

namespace murdoch_rta
{
namespace auctioneer
{
Auctioneer::Auctioneer(ros::NodeHandle *nh, int robotID, double replyTimeout) : Robot(nh, robotID, true, replyTimeout)
{
  _stateController.reset(new std::vector<stateControllerPtr>);
}

Auctioneer::~Auctioneer()
{
}

void Auctioneer::mainLoop()
{
  queueTask();
  processController();
}

bool Auctioneer::processController()
{
  std::vector<unsigned> auxAuctioningTasksUID;
  if (_stateController->size() > 0)
  {
    std::vector<stateControllerPtr>::iterator it = _stateController->begin();
    for (; it != _stateController->end();)
    {
      // if processState returns False, delete it
      if ((*it)->processState())  // TODO make sure each transition returns true or false, process returns transitions
      {
        auxAuctioningTasksUID.push_back((*it)->getAuctioningTask()->getTaskUID());
        ++it;
      }
      else
      {
        (*it).reset();
        it = _stateController->erase(it);
      }
    }
    this->_auctioningTasksUID = auxAuctioningTasksUID;
    return true;
  }
  this->_auctioningTasksUID.clear();
  return false;
}

void Auctioneer::retrieveQueueTask(task::taskPtr &retrievedTask)
{
  // transfer ownership to state machine from auctioneer
  retrievedTask = boost::make_shared<task::Task>(*(_tasksQueued.get()));
  _tasksQueued.reset();
}

bool Auctioneer::queueTask()
{
  // TODO : check if task belongs to this auctioner auction scope
  if (_listener->hasTaskInBuffer())
  {
    // define a task to retrieve from listener
    task::Task taskToAuction = _listener->retrieveHighestPriorityTask();
    // make sure task is safe for usage
    if (task::Task::isTaskValid(taskToAuction))
    {
      if (std::find(_auctioningTasksUID.begin(), _auctioningTasksUID.end(), taskToAuction.getTaskUID()) ==
          _auctioningTasksUID.end())
      {
        _tasksQueued.reset(new task::Task(taskToAuction));
        if (_tasksQueued)
        {
          stateControllerPtr newController;
          newController.reset(new StateController(this));
          _stateController->push_back(newController);
          return true;
        }
        else
        {
          std::stringstream errMsg;
          errMsg << "A task was retrieved from listener but was not initialized!";
          throw UndefinedAuctionBehavior(errMsg.str());
        }
      }
      else
      {
        std::stringstream errMsg;
        errMsg << "Trying to auction a Task while a Task with the same UID is being auctioned.";
        throw UndefinedAuctionBehavior(errMsg.str());
      }
    }
    std::stringstream errMsg;
    errMsg << "Highest priority task received for auction is invalid. Cannot start auction!";
    throw UndefinedAuctionBehavior(errMsg.str());
  }
  return false;
}

std::vector<murdoch_rta_msgs::Task> Auctioneer::getTasksUnderExecutionAsRosMsg()
{
  std::vector<murdoch_rta_msgs::Task> tasksVec;

  std::vector<stateControllerPtr>::iterator it = _stateController->begin();
  for (; it != _stateController->end();)
  {
    if ((*it)->getCurrentStateType() == states::MonitorTaskState ||
        (*it)->getCurrentStateType() == states::WaitAcknowledgmentState)
    {
      tasksVec.push_back((*it)->getAuctioningTask()->toRosMessage());
    }
  }

  return tasksVec;
}
}
}
