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

#include "murdoch_rta_msgs/Auction.h"
#include "murdoch_rta_msgs/BaseMessage.h"
#include "murdoch_rta_msgs/Boolean.h"
#include "murdoch_rta_msgs/Resource.h"
#include "murdoch_rta_msgs/Task.h"
#include "murdoch_rta_msgs/getResources.h"
#include "murdoch_rta_msgs/getTaskStatus.h"
#include "murdoch_rta_msgs/setTask.h"

#include "messages/AuctionMessage.h"
#include "messages/BooleanMessage.h"

#include "murdoch_rta/task/Task.h"

#include "ros/ros.h"

namespace murdoch_rta
{
namespace communicator
{
class Talker
{
private:
  int _robotID;
  bool _isAuctioner;
  ros::NodeHandle *_nh;

  ros::Publisher _auctionsPub;
  ros::Publisher _bidPub;
  ros::Publisher _boolMgsPub;
  ros::ServiceClient _requestResourcesSrvClient;
  ros::ServiceClient _requestTaskStatusSrvClient;
  ros::ServiceClient _requestSetRobotTaskSrvClient;

public:
  Talker(ros::NodeHandle *nh, int robotID, bool isAuctioner);
  ~Talker();

  void initPublisher();
  void initServiceClient();

  void publishAuction(const murdoch_rta_msgs::Auction &auction) const;
  void publishBooleanMsg(const murdoch_rta_msgs::Boolean &boolMsg) const;
  void publishBidMsg(const murdoch_rta_msgs::Bid &bidRosMsg) const;
  void publishBidMsg(const messages::BidMessage &bidMsg) const;
  void publishBidMsg(const float &bidValue, const int &auctionerID, const int &taskUID) const;

  bool requestResourcesFromRobot(std::vector<murdoch_rta_msgs::Resource> &robotResources);
  bool requestTaskStatus(const task::Task *const task, messages::BooleanMessage &taskStatus);
  bool requestSetRobotTask(const task::Task *const task, const ros::Time &renewalBeforeTime);
};

typedef boost::scoped_ptr<Talker> talkerPtr;
}
}
