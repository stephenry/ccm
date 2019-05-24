//========================================================================== //
// Copyright (c) 2018, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

#include "memory.hpp"
#include "message.hpp"

namespace ccm {

Memory::Memory(const ActorOptions &opts) : CoherentActor(opts) {}

void Memory::apply(TimeStamped<Message *> ts) { }

void Memory::eval(Context &context) {
  const Epoch epoch = context.epoch();
  Cursor cursor = epoch.cursor();

  // do {
  //   const QueueEntry next; // = qmgr_.next();
  //   if (next.type() == QueueEntryType::Invalid) break;

  //   if (!epoch.in_interval(next.time())) break;

  //   cursor.set_time(std::max(time(), next.time()));
  //   set_time(cursor.time());

  //   switch (next.type()) {
  //     case QueueEntryType::Message:
  //       handle_msg(context, cursor, next.as_msg());
  //       break;
  //     default:;  // TODO: unexpected
  //   }
  //   next.consume();
  // } while (epoch.in_interval(cursor.time()));

  set_time(cursor.time());
}

void Memory::handle_msg(Context &context, Cursor &cursor,
                        TimeStamped<Message *> ts) {
  const Message *msg = ts.t();
  CCM_AGENT_ASSERT(msg->type() == MessageType::Data);
  cursor.advance(MessageType::to_cost(msg->type()));
  log_debug(cursor.time(), "Write-back to memory:", to_string(*msg));
  msg->release();
}

bool Memory::is_active() const { return false; }

}  // namespace ccm
