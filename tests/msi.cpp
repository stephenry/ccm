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

TEST(MSI, Load) {
  const std::size_t addr = 0;
  
  ccm::Logger l;
  l.set_force_flush(true);
  
  ccm::LoggerScope * top = l.top();
  
  ccm::Sim s;

  std::vector<ccm::Agent *> agents_;
  ccm::SnoopFilter * snoop_filter;
  for (std::size_t i = 0; i < 4; i++) {
    ccm::AgentOptions opts(i, ccm::Protocol::MSI, ccm::CacheOptions());

    std::stringstream ss;
    ss << "Agent" << i;
    opts.set_logger_scope(top->child_scope(ss.str()));
    
    agents_.push_back(new ccm::Agent(opts));
    s.add_actor(agents_.back());
  }

  ccm::Transaction * t = new ccm::Transaction{addr, ccm::TransactionType::Load};
  agents_[0]->add_transaction(10, t);

  ccm::SnoopFilterOptions opts(4, ccm::Protocol::MSI, ccm::CacheOptions());
  opts.set_logger_scope(top->child_scope("SnoopFilter"));
  snoop_filter = new ccm::SnoopFilter(opts);
  s.add_actor(snoop_filter);
  s.run();

  const ccm::CacheLine cache_line = agents_[0]->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::S));

  const ccm::DirectoryEntry directory_entry = snoop_filter->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::S));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
