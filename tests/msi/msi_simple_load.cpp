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

#include <gtest/gtest.h>
#include "testcommon.hpp"

TEST(MSI, SimpleLoad) {
  using rnd = ccm::Random;

  // Perform a single load to one agent in the system. At the end of
  // the simulation, the line should be installed in the requestor in
  // the shared state, and installed in the directory in the shared
  // state.
  //
  rnd::set_seed(1);

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  auto ragent = rnd::UniformRandomInterval<ccm::id_t>(p.agents() - 1, 0);
  auto raddr = rnd::UniformRandomInterval<ccm::addr_t>(1 << 12);

  std::vector<std::set<ccm::addr_t> > addrs_id{p.agents()};
  for (std::size_t i = 0; i < 1; i++) {
    const ccm::Time t = (1 + i) * 1000;
    const ccm::addr_t addr = raddr();
    const ccm::id_t id = ragent();

    addrs_id[id].insert(addr);
    p.ts(id)->add_transaction(ccm::TransactionType::load, t, addr);
  }

  s.run();

  for (ccm::id_t id = 0; id < addrs_id.size(); id++) {
    for (const ccm::addr_t addr : addrs_id[id]) {
      ccm::AgentTestProxy agent{p.agent(id)};
      const ccm::CacheLine & cache_line = agent.cache_line(addr);
      EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::S);

      const ccm::DirectoryEntry directory_entry =
          p.snoop_filter()->directory_entry(addr);
      EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::S);
    }
  }
  EXPECT_TRUE(p.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
