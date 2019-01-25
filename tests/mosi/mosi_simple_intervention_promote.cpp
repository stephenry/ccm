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
  // Agent 0 writes to a line which is installed in its cache in the M
  // state. Agents 1-3 request the line which causes the line in Agent
  // 0 to be denoted to O, and the line to be intervened to the other
  // Agents. Agent 3 attempts to write to the line. The line in agents
  // 0-2 is invalidated and the line in Agent 3 promoted to M.
  //
  
  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MOSI, 4};

  
  p.ts(0)->add_transaction(ccm::TransactionType::Store, 100, addr);
  p.ts(1)->add_transaction(ccm::TransactionType::Load, 200, addr);
  p.ts(2)->add_transaction(ccm::TransactionType::Load, 300, addr);
  p.ts(3)->add_transaction(ccm::TransactionType::Load, 400, addr);

  p.ts(3)->add_transaction(ccm::TransactionType::Store, 500, addr);
  
  s.run();

  for (std::size_t i = 0; i < 4; i++) {
    const ccm::CacheLine cache_line = p.agent(i)->cache_line(addr);
    if (i == 3)
      EXPECT_EQ(cache_line.state(), ccm::MosiAgentLineState::M);
    else
      EXPECT_EQ(cache_line.state(), ccm::MosiAgentLineState::I);
  }

  const ccm::DirectoryEntry directory_entry =
    p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MosiDirectoryLineState::M);

  EXPECT_TRUE(p.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
