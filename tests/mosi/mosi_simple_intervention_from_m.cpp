//========================================================================== //
// Copyright (c) 2019, Stephen Henry
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

TEST(MOSI, SimpleInterventionFromM) {
  // Agent 0 requests a line, the line is installed in the agents
  // cache in the exclusive state. Agent 1 attempts to load the same
  // line. Agent 0 forwards (intervenes) and sends the data to Agent
  // 1. The line remains in Agent 0's cache but is downgraded to the
  // shared state. The line is installed in Agent 1's cache in the
  // shared state. The line is shared in the directory state.
  //
  
  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MOSI, 4};

  p.ts(0)->add_transaction(ccm::TransactionType::store, 100, addr);
  p.ts(1)->add_transaction(ccm::TransactionType::load, 200, addr);

  s.run();

  ccm::AgentTestHarness agent_0{p.agent(0)};
  const ccm::CacheLine cache_line_0 = agent_0.cache_line(addr);
  EXPECT_EQ(cache_line_0.state(), ccm::MosiAgentLineState::O);

  ccm::AgentTestHarness agent_1{p.agent(1)};
  const ccm::CacheLine cache_line_1 = agent_1.cache_line(addr);
  EXPECT_EQ(cache_line_1.state(), ccm::MosiAgentLineState::S);

  const ccm::DirectoryEntry directory_entry =
    p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MosiDirectoryLineState::O);

  EXPECT_TRUE(p.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
