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

#ifndef __SRC_AGENT_HPP__
#define __SRC_AGENT_HPP__

#include "coherence.hpp"
#include "message.hpp"
#include "sim.hpp"
#include <deque>

namespace ccm {

struct TransactionSource;

struct AgentOptions : CoherentAgentOptions {
  AgentOptions(std::size_t id, Protocol protocol,
               CacheOptions cache_options, Platform platform)
    : CoherentAgentOptions(id, protocol, cache_options, platform)
  {}
};

struct Agent : CoherentAgentCommandInvoker {
  Agent(const AgentOptions & opts);

  bool is_active() const override;
  Protocol protocol() const { return opts_.protocol(); }
  CacheOptions cache_options() const { return opts_.cache_options(); }

  void set_transaction_source(TransactionSource * trns) { trns_ = trns; }
  TransactionSource * transaction_source() const { return trns_; }
  
  void apply(TimeStamped<Message *> ts) override;
  void eval(Context & ctxt) override;
  
  // private:
  void fetch_transactions(std::size_t n = 10);
  
  void handle_msg(Context & ctxt, Cursor & cursor,
                  TimeStamped<Message *> ts);
  
  void handle_trn(Context & ctxt, Cursor & cursor,
                  TimeStamped<Transaction *> ts);
  
  QueueManager qmgr_;
  TransactionSource * trns_;
  const AgentOptions opts_;
};

} // namespace ccm

#endif
