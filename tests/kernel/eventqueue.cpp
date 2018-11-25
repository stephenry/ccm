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

#include "src/kernel.hpp"

#include <vector>
#include <gtest/gtest.h>

namespace {

template<typename MSG>
class EventQueueTestTop: public ccm::Module {
  struct FrontierEntry {
    std::size_t t;
    MSG msg;
  };

  struct State {
    ccm::EventQueue<MSG> eq;
    std::deque<FrontierEntry> frontier;
  } state_;

  struct P0 : public ccm::Process {
    P0(State & state) : state_(state) {}
    ccm::InvokeRsp invoke_initialization(ccm::InvokeReq const & req) override {
      n_ = state_.frontier.size();
      last_time_ = state_.frontier.back().t;
      
      ccm::InvokeRsp rsp;
      rsp.wake_on(state_.eq.event());
      return rsp;
    }
    ccm::InvokeRsp invoke_running(ccm::InvokeReq const & req) override {
      const FrontierEntry & e = state_.frontier.front();
      
      EXPECT_EQ(now(), e.t);
      EXPECT_TRUE(state_.eq.has_msg(now()));

      MSG msg;
      EXPECT_TRUE(state_.eq.get(msg, now()));
      EXPECT_EQ(msg, e.msg);

      state_.frontier.pop_front();
      count_++;
      
      ccm::InvokeRsp rsp;
      rsp.wake_on(state_.eq.event());
      return rsp;
    }
    ccm::InvokeRsp invoke_termination(ccm::InvokeReq const & req) override {
      EXPECT_EQ(state_.frontier.size(), 0);
      EXPECT_EQ(n_, count_);
      EXPECT_EQ(now(), last_time_);
    }
   private:
    State & state_;
    std::size_t n_{0}, count_{0}, last_time_{0};
  };
 public:
  EventQueueTestTop(std::string name) : ccm::Module(name) {
    for (std::size_t i = 0; i < 100; i++) {
      const std::size_t msg = i;
      const std::size_t t = i * 10;
      const FrontierEntry fe{t, msg};
      
      state_.frontier.push_back(fe);
      state_.eq.set(msg, t);
    }
    p0 = create_process<P0>(state_);
  }
private:
  ccm::Process * p0;
};
  
} // namespace

TEST(EventQueueTest, t0) {
  ccm::Scheduler sch;
  ccm::ModulePtr top = sch.construct_module<
    EventQueueTestTop<std::size_t>>("EventQueueTestTop");
  sch.set_top(std::move(top));
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
