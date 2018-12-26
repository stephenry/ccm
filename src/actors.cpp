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

#include "actors.hpp"
#include "sim.hpp"

namespace ccm {

void Agent::add_transaction(std::size_t time, Transaction * t) {
  pending_transactions_.push(make_time_stamped(time, t));
}

bool Agent::eval(Frontier & f) {
  if (!pending_transactions_.empty()) {

    TimeStamped<Transaction *> head;
    while (pending_transactions_.pop(head)) {
      set_time(head.time());
        
      MessageBuilder b = msgd_.builder();
      b.set_type(MessageType::GetS);
      b.set_dst_id(4);
      b.set_tid(1);
      f.add_to_frontier(1 + head.time(), b.msg());
        
      std::cout << time() << " SnoopActor: Sending something\n";
    }
  }
  return is_active();
}

void SnoopFilter::apply(std::size_t t, const Message * m) {
  pending_messages_.push(make_time_stamped(t, m));
}

bool SnoopFilter::eval(Frontier & f) {
  if (!pending_messages_.empty()) {

    TimeStamped<const Message *> head;
    while (pending_messages_.pop(head)) {
      set_time(head.time());
      
      std::cout << time() << " SnoopFilter: Received something\n";
    }
  }
  return is_active();
}

} // namespace ccm
