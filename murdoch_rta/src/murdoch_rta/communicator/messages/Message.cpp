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

#include "murdoch_rta/communicator/messages/Message.h"

namespace murdoch_rta
{
namespace communicator
{
namespace messages
{
Message::Message(int idSender, int idReceiver, ros::Time messageTime)
  : _idReceiver(idReceiver), _idSender(idSender), _messageSentTime(messageTime)
{
}

Message::Message(murdoch_rta_msgs::BaseMessage base)
  : _idReceiver(base.robotReceiverId), _idSender(base.robotSenderId), _messageSentTime(base.messageTime)
{
}

Message::~Message()
{
}

ros::Time Message::getMessageSentTime() const
{
  return _messageSentTime;
}

int Message::getIdReceiver() const
{
  return _idReceiver;
}

int Message::getIdSender() const
{
  return _idSender;
}

murdoch_rta_msgs::BaseMessage Message::toRosMessage() const
{
  murdoch_rta_msgs::BaseMessage baseMsg;

  baseMsg.messageTime = _messageSentTime;
  baseMsg.robotSenderId = _idSender;
  baseMsg.robotReceiverId = _idReceiver;

  return baseMsg;
}
}
}
}
