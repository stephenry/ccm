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
#include <deque>

namespace ccm {

struct AgentOptions : CoherentAgentOptions {
  AgentOptions(std::size_t id, Protocol protocol, CacheOptions cache_options)
      : CoherentAgentOptions(id, protocol, cache_options)
  {}
};

struct Agent : CoherentAgentCommandInvoker {
  Agent(const AgentOptions & opts);

  bool is_active() const override;
  Protocol protocol() const { return opts_.protocol(); }
  CacheOptions cache_options() const { return opts_.cache_options(); }
  bool can_accept() const { return !tt_.is_full(); }
  
  void add_transaction(std::size_t time, Transaction * t);

  void apply(std::size_t t, const Message * m) override;
  bool eval(Frontier & f) override;
  
 private:
  TransactionTable tt_;
  std::deque<TimeStamped<Transaction *> > pending_transactions_;
  std::vector<TimeStamped<const Message *> > pending_messages_;
  const AgentOptions opts_;
};

} // namespace ccm

#endif
