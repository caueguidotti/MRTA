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

#include "murdoch_rta/communicator/messages/BooleanMessage.h"

namespace murdoch_rta
{
namespace communicator
{
namespace messages
{
BooleanMessage::BooleanMessage(int senderID, int receiverID, int taskUID, BooleanMessagesType boolMsgType,
                               ros::Time messageSentTime)
  : Message(senderID, receiverID, messageSentTime), _messageValue(true), _messageType(boolMsgType), _taskUID(taskUID)
{
}

BooleanMessage::BooleanMessage(const murdoch_rta_msgs::Boolean::ConstPtr &boolMsgRos)
  : Message(boolMsgRos->baseMsg)
  , _messageValue(boolMsgRos->data)
  , _messageType(static_cast<BooleanMessagesType>(boolMsgRos->msgType))
  , _taskUID(boolMsgRos->taskUID)
{
}

BooleanMessage::BooleanMessage(const murdoch_rta_msgs::Boolean &boolMsgRos)
  : Message(boolMsgRos.baseMsg)
  , _messageValue(boolMsgRos.data)
  , _messageType(static_cast<BooleanMessagesType>(boolMsgRos.msgType))
  , _taskUID(boolMsgRos.taskUID)
{
}

BooleanMessage::BooleanMessage(const BooleanMessage *boolMsg)
  : Message(boolMsg->getIdSender(), boolMsg->getIdReceiver(), boolMsg->getMessageSentTime())
  , _messageValue(boolMsg->getMessageValue())
  , _messageType(boolMsg->getMessageType())
  , _taskUID(boolMsg->getTaskUID())
{
}

BooleanMessage::~BooleanMessage()
{
}

BooleanMessagesType BooleanMessage::getMessageType() const
{
  return _messageType;
}

bool BooleanMessage::isProgressTaskMsg() const
{
  return (_messageType == messages::TaskCancelled || _messageType == messages::TaskCompleted ||
          _messageType == messages::TaskContinuing);
}

bool BooleanMessage::getMessageValue() const
{
  return _messageValue;
}

int BooleanMessage::getTaskUID() const
{
  return _taskUID;
}

murdoch_rta_msgs::Boolean BooleanMessage::toRosMessage()
{
  murdoch_rta_msgs::Boolean boolMsg;

  boolMsg.baseMsg = Message::toRosMessage();
  boolMsg.data = _messageValue;
  boolMsg.msgType = _messageType;
  boolMsg.taskUID = _taskUID;

  return boolMsg;
}
}
}
}
