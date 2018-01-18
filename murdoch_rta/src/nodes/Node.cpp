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

#include "nodes/Node.h"

/**
 * @brief Node::Node builds an Node object given a ROS NodeHandle
 * and also its desired spin rate.
 * @param nh must NOT be NULL.
 * @param loop_rate must be positive.
 */
Node::Node(ros::NodeHandle *nh, float loop_rate) : loop_rate_(loop_rate)
{
  if (!nh)
  {
    ROS_FATAL("ROS node handle must not be NULL!!!");
    ros::shutdown();
    return;
  }
  if (loop_rate <= 0)
  {
    ROS_FATAL("The node spin rate must be positive!!!");
    ros::shutdown();
    return;
  }
  nh_ = nh;
  name_ = ros::this_node::getName();
}

/**
 * @brief Node::~Node announces that this ROS node will shutdown and
 * destructs the ROS NodeHandle object properly.
 */
Node::~Node()
{
  if (nh_)
    delete nh_;
}

/**
 * @brief Node::spin loops while there is not another instance
 * of this node with this node name, or while the Ctrl+C buttons
 * is not pressed at the terminal. In addition, it periodicly updates
 * this node, as well as, controls the updates rate.
 */
void Node::spin()
{
  ros::Rate loop_rate(loop_rate_);
  ROS_INFO("%s is ON!!!", name_.c_str());
  while (nh_->ok())
  {
    controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 * @brief Node::shutdown
 */
void Node::shutdown() const
{
  ROS_WARN("%s is OFF now!!!", name_.c_str());
  nh_->shutdown();
}

void Node::controlLoop()
{
}

float Node::getLoop_rate() const
{
  return loop_rate_;
}

/**
 * @brief Node::getNodeHandle encapsulates this ROS node handle.
 * @return a pointer to an internal member that handles this node.
 */
ros::NodeHandle *Node::getNodeHandle() const
{
  return nh_;
}

/**
 * @brief Node::getName encapsulates this ROS node name.
 * @return this ROS node whole name.
 */
std::string Node::getName() const
{
  return name_;
}
