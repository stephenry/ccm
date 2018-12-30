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

namespace ccm {

struct AgentOptions : ActorOptions {
  AgentOptions(std::size_t id, Protocol protocol, CacheOptions cache_options)
      : ActorOptions(id), protocol_(protocol), cache_options_(cache_options)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
};

struct Agent : CoherentAgentCommandInvoker {
  Agent(const AgentOptions & opts);

  Protocol protocol() const { return opts_.protocol(); }
  CacheOptions cache_options() const { return opts_.cache_options(); }

  
  void add_transaction(std::size_t time, Transaction * t);

  bool can_accept() const {
    return !tt_.is_full();
  }

  void apply(std::size_t t, const Message * m) override {
    pending_messages_.push_back(m);
  }
  
  bool eval(Frontier & f) override;
  
  bool is_active() const override {
    // Agent is active if there are pending transction in the
    // Transaction Table, or if there are transaction awaiting to be
    // issued.
    //
    return (!tt_.is_empty() || !pending_transactions_.empty());
  }
  
 private:
  const AgentOptions & opts_;
  TransactionTable tt_;
  Heap<TimeStamped<Transaction *> > pending_transactions_;
  std::vector<const Message *> pending_messages_;
  std::unique_ptr<CoherentAgentModel> cc_model_;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const AgentOptions & opts);

} // namespace ccm

#endif
