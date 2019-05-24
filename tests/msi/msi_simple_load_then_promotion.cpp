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

  p.ts(0)->add_transaction(ccm::TransactionType::load, 100, addr);
  p.ts(0)->add_transaction(ccm::TransactionType::store, 200, addr);

  s.run();

  ccm::AgentTestProxy agent{p.agent(0)};
  const ccm::CacheLine & cache_line = agent.cache_line(addr);
  EXPECT_EQ(cache_line.state(), ccm::MsiAgentLineState::M);

  const ccm::DirectoryEntry directory_entry =
    p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MsiDirectoryLineState::M);

  EXPECT_TRUE(p.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
