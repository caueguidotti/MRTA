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

#include "nodes/Bidder_node.h"
#include "murdoch_rta/bidder/StateController.h"

namespace nodes
{
BidderNode::BidderNode(ros::NodeHandle *nh, float loop_rate) : Node(nh, loop_rate)
{
  ros::NodeHandle pnh("~");

  int robotID;
  double replyTimeout;
  std::string utility_plugin;

  // required parameters
  if (!pnh.hasParam("robot/id") || !pnh.hasParam("robot/replyTimeout"))
  {
    if (!pnh.hasParam("robot/id"))
      ROS_ERROR("You should specify the robot ID as a parameter. Expecting a parameter in 'robot/id'");
    if (!pnh.hasParam("robot/replyTimeout"))
      ROS_ERROR("You should specify the robot reply timeout as a parameter. Expecting a parameter in "
                "'robot/replyTimeout'");
    Node::shutdown();
  }

  // Optional parameters
  if (!pnh.hasParam("robot/utility_plugin"))
  {
    ROS_WARN("You did not specify a utility plugin, using default - you will see a warning message!");
  }

  pnh.getParam("robot/id", robotID);
  pnh.getParam("robot/replyTimeout", replyTimeout);
  pnh.param<std::string>("robot/utility_plugin", utility_plugin, "murdoch_rta/default_utility_calculator");

  _bidder = boost::make_shared<murdoch_rta::bidder::Bidder>(nh, robotID, utility_plugin, replyTimeout);
  initNode();
}

BidderNode::~BidderNode()
{
}

void BidderNode::initNode()
{
  // initialization code
}

void BidderNode::controlLoop()
{
  _bidder->mainLoop();
}
}
