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
  for (std::size_t i = 0; i < 1000; i++) {
    const ccm::Time t = (1 + i) * 1000;
    const ccm::addr_t addr = raddr();
    const ccm::id_t id = ragent();

    addrs_id[id].insert(addr);
    p.ts(id)->add_transaction(ccm::TransactionType::Load, t, addr);
  }

  s.run();

  for (ccm::id_t id = 0; id < addrs_id.size(); id++) {
    for (const ccm::addr_t addr : addrs_id[id]) {
      const ccm::CacheLine cache_line = p.agent(id)->cache_line(addr);
      EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::S);

      const ccm::DirectoryEntry directory_entry =
          p.snoop_filter()->directory_entry(addr);
      EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::S);
    }
  }
}

TEST(MSI, SimpleLoadPromotion) {
  // Perform a load followed by a store to the same address in the
  // same agent. Upon completion of the first transaction, the line
  // should be installed in the shared state. Upon completion of the
  // second instruction, the line should be promoted to the modified
  // state. As no other agents hold the line, no invalidation requests
  // should be passed to any other agent.
  //

  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  p.ts(0)->add_transaction(ccm::TransactionType::Load, 100, addr);
  p.ts(0)->add_transaction(ccm::TransactionType::Store, 200, addr);

  s.run();

  const ccm::CacheLine cache_line = p.agent(0)->cache_line(addr);
  EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::M);

  const ccm::DirectoryEntry directory_entry =
      p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::M);
}

TEST(MSI, SimpleStore) {
  using rnd = ccm::Random;

  // Perform a single store to one agent in the system. At the end of
  // the simulation, the line should be installed in the requester in the
  // modified state, and installed in the directory in the modified state.
  //
  rnd::set_seed(1);

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  auto ragent = rnd::UniformRandomInterval<ccm::id_t>(p.agents() - 1, 0);
  auto raddr = rnd::UniformRandomInterval<ccm::addr_t>(1 << 12);

  std::set<ccm::addr_t> addrs;
  std::vector<std::set<ccm::addr_t> > addrs_id{p.agents()};

  for (std::size_t i = 0; i < 1000; i++) {
    const ccm::Time t = (1 + i) * 1000;
    const ccm::addr_t addr = raddr();
    const ccm::id_t id = ragent();

    // In this simple test, the set of addresses issued by each agent
    // must be disjoint (we do not specifically wish to consider
    // forwarding in this case).
    //
    if (addrs.find(addr) != addrs.end()) continue;

    addrs.insert(addr);
    addrs_id[id].insert(addr);
    p.ts(id)->add_transaction(ccm::TransactionType::Store, t, addr);
  }

  s.run();

  for (ccm::id_t id = 0; id < addrs_id.size(); id++) {
    for (const ccm::addr_t addr : addrs_id[id]) {
      const ccm::CacheLine cache_line = p.agent(id)->cache_line(addr);
      EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::M);

      const ccm::DirectoryEntry directory_entry =
          p.snoop_filter()->directory_entry(addr);
      EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::M);
    }
  }
}

TEST(MSI, MultipleSharers) {
  // Each agent in the system performs a load request to the same
  // line.  Upon completion of the commands, each agent should have a
  // line installed in its cache in the shared state. The directory
  // should have the line in the shared state and each agent should
  // be present in the sharer set.
  //

  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  for (std::size_t i = 0; i < p.agents(); i++) {
    const std::size_t time = (i + 1) * 1000;

    p.ts(i)->add_transaction(ccm::TransactionType::Load, time, addr);
  }

  s.run();

  for (std::size_t i = 0; i < p.agents(); i++) {
    const ccm::CacheLine cache_line = p.agent(i)->cache_line(addr);

    EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::S);
  }

  const ccm::DirectoryEntry directory_entry =
      p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::S);
}

TEST(MSI, MultipleSharersThenPromotion) {
  // Each agent performs a load to the same line. After the loads have
  // completed, an agent performs a Store operation to the line.
  // Before the Store operation completes, each line in the other
  // agents must be invalidate and the resulting acknowlegement passed
  // to the original requesting agent. Upon completion, the storing
  // agent is the only agent with a copy of the line (in the modified
  // state).
  //

  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  for (std::size_t i = 0; i < p.agents(); i++) {
    const std::size_t time = (i + 1) * 1000;

    p.ts(i)->add_transaction(ccm::TransactionType::Load, time, addr);
  }
  p.ts(0)->add_transaction(ccm::TransactionType::Store, 10000, addr);

  s.run();

  for (std::size_t i = 0; i < p.agents(); i++) {
    const ccm::CacheLine cache_line = p.agent(i)->cache_line(addr);

    if (i == 0)
      EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::M);
    else
      EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::I);
  }

  const ccm::DirectoryEntry directory_entry =
      p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::M);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
