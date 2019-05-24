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
#include <vector>
#include <algorithm>
#include "sim.hpp"

TEST(Heap, BasicMinHeap) {
  using namespace ccm;
  
  MinHeap<int> mh;

  mh.push(1000);
  mh.push(5000);
  mh.push(2000);
  mh.push(3000);
  mh.push(4000);

  std::vector<int> out;
  while (!mh.empty()) {
    out.push_back(mh.top());
    mh.pop();
  }
  EXPECT_EQ(out.size(), 5);
  EXPECT_TRUE(std::is_sorted(out.begin(), out.end()));
}

TEST(Heap, BasicMinHeapTS) {
  using namespace ccm;
  
  struct Foo;
  MinHeap<TimeStamped<Foo *> > mh;

  mh.push(TimeStamped<Foo *>{1000, nullptr});
  mh.push(TimeStamped<Foo *>{5000, nullptr});
  mh.push(TimeStamped<Foo *>{2000, nullptr});
  mh.push(TimeStamped<Foo *>{3000, nullptr});
  mh.push(TimeStamped<Foo *>{4000, nullptr});

  std::vector<Time> out;
  while (!mh.empty()) {
    out.push_back(mh.top().time());
    mh.pop();
  }
  EXPECT_EQ(out.size(), 5);
  EXPECT_TRUE(std::is_sorted(out.begin(), out.end()));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
