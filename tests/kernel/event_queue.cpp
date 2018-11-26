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

#include "kernel/kernel.hpp"

#include <vector>
#include <gtest/gtest.h>

namespace {

template<typename MSG>
class EventQueueTestTop: public ccm::kernel::Module {
  struct FrontierEntry {
    std::size_t t;
    MSG msg;
  };

  struct State {
    ccm::kernel::EventQueue<MSG> * eq{nullptr};
    std::deque<FrontierEntry> frontier;
  } state_;

  struct P0 : public ccm::kernel::Process {
    P0(State & state) : state_(state) {}
    void cb__on_initialization() override {
      size_ = state_.frontier.size();
    }
    void cb__on_invoke() override {
      const FrontierEntry & e = state_.frontier.front();
      
      EXPECT_EQ(now(), e.t);
      EXPECT_TRUE(state_.eq->has_msg(now()));

      MSG msg;
      EXPECT_TRUE(state_.eq->get(msg, now()));
      EXPECT_EQ(msg, e.msg);

      state_.frontier.pop_front();
      n_++;
      last_time_ = now();
    }
    void cb__on_termination() override {
      EXPECT_EQ(state_.frontier.size(), 0);
      EXPECT_EQ(n_, size_);
      EXPECT_EQ(now(), last_time_);
    }
   private:
    State & state_;
    std::size_t size_{0}, n_{0}, last_time_{0};
  };
 public:
  EventQueueTestTop(std::string name) : ccm::kernel::Module(name) {
    p0_ = create_process<P0>(state_);
    state_.eq = create_child<ccm::kernel::EventQueue<MSG>>();
    for (std::size_t i = 1; i < 1000; i++) {
      const std::size_t msg = i;
      const std::size_t t = i * 10;
      const FrontierEntry fe{t, msg};
      
      state_.frontier.push_back(fe);
    }
  }
private:
  void cb__on_initialization() override {
    p0_->set_sensitive_on(state_.eq->event());
    for (const FrontierEntry & e : state_.frontier) {
      state_.eq->set(e.msg, e.t);
    }
  }
  ccm::kernel::Process * p0_;
};
  
} // namespace

TEST(EventQueueTest, t0) {
  ccm::kernel::Scheduler sch;
  {
    ccm::kernel::ModulePtr top = sch.construct_top<
      EventQueueTestTop<std::size_t>>("EventQueueTestTop");
    sch.set_top(std::move(top));
  }
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
