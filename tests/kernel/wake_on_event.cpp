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
#include "kernel/kernel.hpp"

namespace {

struct TestOptions {
  std::size_t sim_time{1000};
  std::size_t notify_n{1000};
  std::size_t process_n{10};
  std::size_t delay{10};
};
  
class WakeOnEventTop : public ccm::kernel::Module {
  struct SharedState {
    TestOptions opts;
    std::size_t next_process{0};
    std::size_t n{0};
    std::vector<ccm::kernel::EventHandle> es;
    std::vector<ccm::kernel::Process *> processes_;
  };
  struct WakeOnEventProcess : ccm::kernel::Process {
    WakeOnEventProcess(std::size_t id, SharedState & state)
      : id_(id), state_(state)
    {}
    void cb__on_invoke() override {
      EXPECT_EQ(state_.next_process, id_);

      state_.next_process = ccm::kernel::rand_int();
      if (--state_.n != 0) {
        state_.es[state_.next_process].notify_after(state_.opts.delay);
      }
    }
  private:
    std::size_t id_;
    SharedState & state_;
  };
 public:
  WakeOnEventTop(std::string name, TestOptions const & opts = TestOptions())
    : ccm::kernel::Module(name) {
    state_.opts = opts;
    for (std::size_t i = 0; i < state_.opts.process_n; i++)
      state_.processes_.push_back(create_process<WakeOnEventProcess>(i, state_));
  }
 private:
  void cb__on_elaboration() override {
    for (std::size_t i = 0; i < state_.opts.process_n; i++) {
      state_.es.push_back(create_event());
      state_.processes_[i]->set_sensitive_on(state_.es.back());
    }
  }
  void cb__on_initialization() override {
    state_.n = state_.opts.notify_n;
    state_.es[0].notify_on(10);
  }
  void cb__on_termination() override {
    EXPECT_EQ(state_.n, 0);
    EXPECT_EQ(now(), state_.opts.delay * state_.opts.notify_n);
  }
  SharedState state_;
};

TEST(WakeOnEvent, t0) {
  TestOptions opts;
  ccm::kernel::Scheduler sch;
  {
    ccm::kernel::ModulePtr top =
      sch.construct_top<WakeOnEventTop>("WakeOnEventTop");
    sch.set_top(std::move(top));
  }
  sch.run();
}

} // namespace

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
