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

#include "common.hpp"
#include <gtest/gtest.h>

TEST(MSI, SimpleLoad) {
  // Perform a single load to one agent in the system. At the end of
  // the simulation, the line should be installed in the requestor in
  // the shared state, and installed in the directory in the shared
  // state.
  //
  
  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  ccm::Agent * a0 = p.agent(0);
  a0->add_transaction(10, new ccm::Transaction{addr, ccm::TransactionType::Load});

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  const ccm::CacheLine cache_line = a0->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::S));

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::S));
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

  ccm::Agent * a0 = p.agent(0);
  a0->add_transaction(100, new ccm::Transaction{addr, ccm::TransactionType::Load});
  a0->add_transaction(200, new ccm::Transaction{addr, ccm::TransactionType::Store});

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  const ccm::CacheLine cache_line = a0->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::M));

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::M));
}

TEST(MSI, SimpleStore) {
  // Perform a single store to one agent in the system. At the end of
  // the simulation, the line should be installed in the requester in the
  // modified state, and installed in the directory in the modified state.
  //
  
  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MSI, 4};

  ccm::Agent * a0 = p.agent(0);
  a0->add_transaction(10, new ccm::Transaction{addr, ccm::TransactionType::Store});

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  const ccm::CacheLine cache_line = a0->cache_line(addr);
  EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::M));

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::M));
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
    const std::size_t time = (i + 1) * 100;
    
    p.agent(i)->add_transaction(
        time, new ccm::Transaction{addr, ccm::TransactionType::Load});
  }

  ccm::SnoopFilter * sf = p.snoop_filter();

  s.run();

  for (std::size_t i = 0; i < p.agents(); i++) {
    const ccm::CacheLine cache_line = p.agent(i)->cache_line(addr);
    
    EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::S));
  }

  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::S));
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
    const std::size_t time = (i + 1) * 100;
    
    p.agent(i)->add_transaction(
        time, new ccm::Transaction{addr, ccm::TransactionType::Load});
  }
  p.agent(0)->add_transaction(
      1000, new ccm::Transaction{addr, ccm::TransactionType::Store});

  s.run();

  for (std::size_t i = 0; i < p.agents(); i++) {
    const ccm::CacheLine cache_line = p.agent(i)->cache_line(addr);

    if (i == 0) 
      EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::M));
    else
      EXPECT_EQ(cache_line.state(), _g(ccm::MsiAgentLineState::I));
  }

  ccm::SnoopFilter * sf = p.snoop_filter();
  const ccm::DirectoryEntry directory_entry = sf->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), _g(ccm::MsiDirectoryLineState::M));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
