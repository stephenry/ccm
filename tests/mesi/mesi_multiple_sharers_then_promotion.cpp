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

TEST(MESI, MultipleSharersThenPromotion) {
  // Each agent performs a load to the same line. After the loads have
  // completed, an agent performs a Store operationp to the line.
  // Before the Store operation completes, each line in the other
  // agents must be invalidate and the resulting acknowlegement passed
  // to the original requesting agent. Upon completion, the storing
  // agent is the only agent with a copy of the line (in the modified
  // state).
  //

  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MESI, 4};

  for (std::size_t i = 0; i < p.agents(); i++) {
    const std::size_t time = (i + 1) * 1000;

    p.ts(i)->add_transaction(ccm::TransactionType::load, time, addr);
  }
  p.ts(0)->add_transaction(ccm::TransactionType::store, 10000, addr);

  s.run();

  for (std::size_t i = 0; i < p.agents(); i++) {
    ccm::AgentTestProxy agent{p.agent(i)};
    const ccm::CacheLine & cache_line = agent.cache_line(addr);

    if (i == 0)
      EXPECT_EQ(cache_line.state(), ccm::MesiAgentLineState::M);
    else
      EXPECT_EQ(cache_line.state(), ccm::MesiAgentLineState::I);
  }

  const ccm::DirectoryEntry directory_entry =
      p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MesiDirectoryLineState::M);

  EXPECT_TRUE(p.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
