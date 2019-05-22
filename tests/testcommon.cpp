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

#include "testcommon.hpp"

namespace ccm::test {

BasicPlatform::BasicPlatform(Sim& sim, Protocol::type protocol, std::size_t agents_n)
    : sim_(sim), protocol_(protocol) {
  top_ = logger_.top();

  platform_ = Platform{protocol};
  for (std::size_t i = 0; i < agents_n; i++) platform_.add_agent(i);
  platform_.add_snoop_filter(4, std::make_shared<DefaultAddressRegion>());
  platform_.add_memory(5);

  for (std::size_t i = 0; i < agents_n; i++) construct_agent(i);
  construct_snoop_filter(4);
  construct_memory(platform_.memory_id());

  validator_ = coherence_protocol_validator_factory(protocol);
}

BasicPlatform::~BasicPlatform() {
  for (TransactionSource* ts : ts_) delete ts;
}

bool BasicPlatform::validate() const {
  if (!validator_) return false;

  std::unique_ptr<CacheVisitor> cache_visitor = validator_->get_cache_visitor();
  for (CoherentActor* actor : actors_)
    actor->visit_cache(cache_visitor.get());

  return validator_->validate();
}

void BasicPlatform::construct_snoop_filter(std::size_t id) {
  SnoopFilterOptions opts(4, platform_, CacheOptions());
  opts.set_logger_scope(top_->child_scope("SnoopFilter"));

  std::unique_ptr<SnoopFilter> snoop_filter =
      std::make_unique<SnoopFilter>(opts);
  snoop_filter_ = snoop_filter.get();
  actors_.push_back(snoop_filter_);
  sim_.add_actor(std::move(snoop_filter));
}

void BasicPlatform::construct_agent(std::size_t id) {
  AgentOptions opts(id, platform_, CacheOptions());

  std::stringstream ss;
  ss << "Agent" << id;
  opts.set_logger_scope(top_->child_scope(ss.str()));

  std::unique_ptr<Agent> agent = std::make_unique<Agent>(opts);
  actors_.push_back(agent.get());
  agents_.push_back(agent.get());
  sim_.add_actor(std::move(agent));
  ts_.push_back(new ProgrammaticTransactionSource());

  ss << "Src";
  ts_.back()->set_logger_scope(top_->child_scope(ss.str()));

  agents_.back()->set_transaction_source(ts_.back());
}

void BasicPlatform::construct_memory(id_t id) {
  const ActorOptions opts(id, platform_);
  std::unique_ptr<Memory> mem = std::make_unique<Memory>(opts);
  memories_.push_back(mem.get());
  sim_.add_actor(std::move(mem));
}

}  // namespace ccm::test
