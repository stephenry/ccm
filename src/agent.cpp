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

#include "agent.hpp"
#include "msi.hpp"

namespace ccm {

Agent::Agent(const AgentOptions & opts)
    : CoherentAgentCommandInvoker(opts), opts_(opts) {
  set_logger_scope(opts.logger_scope());
}

void Agent::add_transaction(std::size_t time, Transaction * t) {
  pending_transactions_.push(make_time_stamped(time, t));
}

bool Agent::eval(Frontier & f) {
  if (!pending_messages_.empty()) {

    for (TimeStamped<const Message *> t : pending_messages_) {
      set_time(t.time());

      CacheLine cache_line = cache_->lookup(addr_);
      const CoherentActorActions actions =
          cc_model_->get_actions(t.t(), cache_line);

      execute(f, actions, cache_line);
    }
    pending_messages_.clear();
  }
  if (!pending_transactions_.empty()) {

    TimeStamped<Transaction *> head;
    while (pending_transactions_.pop(head)) {
      const Transaction * t = head.t();
      
      set_time(head.time());

      if (!cache_->is_hit(t->addr())) {
        addr_ = t->addr();
        
        CacheLine cache_line;
        cc_model_->line_init(cache_line);
        cache_->install(t->addr(), cache_line);
      }

      CacheLine & cache_line = cache_->lookup(t->addr());
      const CoherentActorActions actions =
          cc_model_->get_actions(head.t(), cache_line);

      execute(f, actions, cache_line);
    }
  }
  return is_active();
}

} // namespace ccm
