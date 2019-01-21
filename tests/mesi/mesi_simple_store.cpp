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

TEST(MESI, SimpleStore) {
  // Perform a single store to one agent in the system. At the end of
  // the simulation, the line should be installed in the requester in the
  // modified state, and installed in the directory in the modified state.
  //

  const std::size_t addr = 0;

  ccm::Sim s;
  ccm::test::BasicPlatform p{s, ccm::Protocol::MESI, 4};

  p.ts(0)->add_transaction(ccm::TransactionType::Store, 100, addr);

  s.run();

  const ccm::CacheLine cache_line = p.agent(0)->cache_line(addr);
  EXPECT_EQ(cache_line.state(), ccm::MesiAgentLineState::M);

  const ccm::DirectoryEntry directory_entry =
      p.snoop_filter()->directory_entry(addr);
  EXPECT_EQ(directory_entry.state(), ccm::MesiDirectoryLineState::M);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
