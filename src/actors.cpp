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

Agent::Agent(const AgentOptions & opts)
    : CoherentActor(opts), opts_(opts), invoker_(opts) {
  set_logger_scope(opts.logger_scope());
  cc_model_ = coherent_agent_factory(opts);
}

void Agent::add_transaction(std::size_t time, Transaction * t) {
  pending_transactions_.push(make_time_stamped(time, t));
}

bool Agent::eval(Frontier & f) {
  if (!pending_messages_.empty()) {

    for (const Message * msg : pending_messages_) {
      const CoherentActorActions actions = cc_model_->get_actions(msg);
      const bool do_commit = false;
      if (do_commit)
        invoker_.invoke(f, actions);
    }
  }
  if (!pending_transactions_.empty()) {

    TimeStamped<Transaction *> head;
    while (pending_transactions_.pop(head)) {
      set_time(head.time());

      const CoherentActorActions actions = cc_model_->get_actions(head.t());
      const bool do_commit = true;

      if (do_commit) {
        invoker_.set_time(time());
        invoker_.invoke(f, actions);
        set_time(invoker_.time());
      }
      log_debug("SnoopActor: Sending something");
    }
  }
  return is_active();
}

SnoopFilter::SnoopFilter(const SnoopFilterOptions & opts)
    : CoherentActor(opts), opts_(opts), invoker_(opts) {
  set_logger_scope(opts.logger_scope());
  cc_model_ = snoop_filter_factory(opts);
}

void SnoopFilter::apply(std::size_t t, const Message * m) {
  pending_messages_.push(make_time_stamped(t, m));
}

bool SnoopFilter::eval(Frontier & f) {
  if (!pending_messages_.empty()) {

    TimeStamped<const Message *> head;
    while (pending_messages_.pop(head)) {
      set_time(head.time());

      DirectoryEntry dir_entry; // TODO: cache lookup
      const CoherentActorActions actions =
          cc_model_->get_actions(head.t(), dir_entry);
      const bool do_commit = true;
      if (do_commit) {
        invoker_.set_time(time());
        invoker_.invoke(actions, head.t(), f, dir_entry);
      }
      
      log_debug("SnoopFilter: Received something");
    }
  }
  return is_active();
}

} // namespace ccm
