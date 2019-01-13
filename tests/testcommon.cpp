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

BasicPlatform::BasicPlatform(Sim & sim, Protocol protocol, std::size_t agents_n)
  : sim_(sim), protocol_(protocol) {
  top_ = logger_.top();

  for (std::size_t i = 0; i < agents_n; i++)
    platform_.add_agent(i);
  platform_.add_snoop_filter(4, std::make_shared<DefaultAddressRegion>());

  for (std::size_t i = 0; i < agents_n; i++)
    construct_agent(i);
  construct_snoop_filter(4);
}
  
BasicPlatform::~BasicPlatform() {
  for (CoherentActor * actor : actors_)
    delete actor;
  for (TransactionSource * ts : ts_)
    delete ts;
}

void BasicPlatform::construct_snoop_filter(std::size_t id) {
  SnoopFilterOptions opts(4, protocol(), CacheOptions(), platform_);
  opts.set_logger_scope(top_->child_scope("SnoopFilter"));
    
  snoop_filter_ = new SnoopFilter(opts);
  add_actor(snoop_filter_);
}
  
void BasicPlatform::construct_agent(std::size_t id) {
  AgentOptions opts(id, protocol(), CacheOptions(), platform_);

  std::stringstream ss;
  ss << "Agent" << id;
  opts.set_logger_scope(top_->child_scope(ss.str()));

  agents_.push_back(new Agent(opts));
  ts_.push_back(new ProgrammaticTransactionSource());

  ss << "Src";
  ts_.back()->set_logger_scope(top_->child_scope(ss.str()));
  
  agents_.back()->set_transaction_source(ts_.back());
  add_actor(agents_.back());
}

void BasicPlatform::add_actor(CoherentActor * actor) {
  actors_.push_back(actor);
  sim_.add_actor(actor);
}

} // namespace ccm::test