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

#include "ccm.hpp"
#include <gtest/gtest.h>

struct BasicPlatform {
  BasicPlatform(ccm::Sim & sim, std::size_t agents_n) : sim_(sim) {
    top_ = logger_.top();

    for (std::size_t i = 0; i < agents_n; i++)
      construct_agent(i);
    construct_snoop_filter(4);
  }
  ~BasicPlatform() {
    for (ccm::CoherentActor * actor : actors_)
      delete actor;
  }

  ccm::Agent * agent(std::size_t id) { return agents_[id]; }
  ccm::SnoopFilter * snoop_filter() { return snoop_filter_; }
  
 private:
  void construct_snoop_filter(std::size_t id) {
    ccm::SnoopFilterOptions opts(4, ccm::Protocol::MSI, ccm::CacheOptions());
    opts.set_logger_scope(top_->child_scope("SnoopFilter"));
    
    snoop_filter_ = new ccm::SnoopFilter(opts);
    add_actor(snoop_filter_);
  }
  
  void construct_agent(std::size_t id) {
    ccm::AgentOptions opts(id, ccm::Protocol::MSI, ccm::CacheOptions());

    std::stringstream ss;
    ss << "Agent" << id;
    opts.set_logger_scope(top_->child_scope(ss.str()));

    ccm::Agent * agent = new ccm::Agent(opts);
    agents_.push_back(agent);
    add_actor(agent);
  }

  void add_actor(ccm::CoherentActor * actor) {
    actors_.push_back(actor);
    sim_.add_actor(actor);
  }

  ccm::Logger logger_;
  ccm::LoggerScope * top_;
  ccm::SnoopFilter * snoop_filter_;
  ccm::Sim & sim_;
  std::vector<ccm::Agent *> agents_;
  std::vector<ccm::CoherentActor *> actors_;
};

TEST(MSI, SimpleLoad) {
  const std::size_t addr = 0;

  ccm::Sim s;
  BasicPlatform p{s, 4};

  ccm::Agent * a0 = p.agent(0);
  a0->add_transaction(10, new ccm::Transaction{addr, ccm::TransactionType::Load});

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  const ccm::CacheLine cache_line = a0->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::S));

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::S));
}

TEST(MSI, SimpleStore) {
  const std::size_t addr = 0;

  ccm::Sim s;
  BasicPlatform p{s, 4};

  ccm::Agent * a0 = p.agent(0);
  a0->add_transaction(10, new ccm::Transaction{addr, ccm::TransactionType::Store});

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  const ccm::CacheLine cache_line = a0->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::M));

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::M));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
