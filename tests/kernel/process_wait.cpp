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
#include "src/kernel.hpp"

namespace {

class ProcessWaitOnEventTop : public ccm::Module {

  struct SharedState {
    std::size_t n, max{1000};
    ccm::EventHandle e;
  };

  struct ProcessWaitOnEvent : public ccm::Process {
    ProcessWaitOnEvent(SharedState & state)
      : state_(state)
    {}
  private:
    void cb__on_invoke() override {
      if (++state_.n != state_.max)
        state_.e.notify_after(10);
    }
    void cb__on_termination() override {
      EXPECT_EQ(state_.n, state_.max);
    }
    SharedState state_;
  };
public:
  ProcessWaitOnEventTop() : ccm::Module() {
    p_ = create_process<ProcessWaitOnEvent>(state_);
  }
private:
  void cb__on_elaboration() override {
    state_.e = create_event();
    p_->set_sensitive_on(state_.e);
    state_.e.notify_after(10);
  }
  SharedState state_;
  ccm::Process * p_;
};

class ProcessWaitForTestTop : public ccm::Module {

  struct SharedState {
    std::size_t n{1000};
    std::size_t delay{10};
    std::size_t next_time;
  };

  struct ProcessWaitFor : public ccm::Process {
    ProcessWaitFor(SharedState & state)
      : state_(state)
    {}
  private:
    void cb__on_initialization() override {
      update();
    }
    void cb__on_invoke() override {
      EXPECT_EQ(now(), state_.next_time);
      update();
    }
    void update() {
      state_.next_time += state_.delay;
      if (--state_.n != 0)
        wait_for(state_.delay);
    }
    SharedState & state_;
  };
public:
  ProcessWaitForTestTop() : ccm::Module() {
    p_ = create_process<ProcessWaitFor>(state_);
  }
private:
  void cb__on_elaboration() override {
  }
  SharedState state_;
  ccm::Process * p_;
};

class ProcessWaitUntilTestTop : public ccm::Module {

  struct SharedState {
    std::size_t n{1000};
    std::size_t delay{10};
    std::size_t next_time;
  };

  struct ProcessWaitUntil : public ccm::Process {
    ProcessWaitUntil(SharedState & state)
      : state_(state)
    {}
  private:
    void cb__on_initialization() override {
      update();
    }
    void cb__on_invoke() override {
      EXPECT_EQ(now(), state_.next_time);
      update();
    }
    void update() {
      state_.next_time += state_.delay;
      if (--state_.n != 0)
        wait_until(state_.next_time);
    }
    SharedState & state_;
  };
public:
  ProcessWaitUntilTestTop() : ccm::Module() {
    p_ = create_process<ProcessWaitUntil>(state_);
  }
private:
  void cb__on_elaboration() override {
  }
  SharedState state_;
  ccm::Process * p_;
};

} // namespace

TEST(ProcessWaitOnEventTest, t0) {
  ccm::Scheduler sch;
  sch.run();
}

TEST(ProcessWaitForTest, t0) {
  ccm::Scheduler sch;
  sch.run();
}

TEST(ProcessWaitUntilTest, t0) {
  ccm::Scheduler sch;
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
