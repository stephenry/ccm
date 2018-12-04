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

  class ProcessWaitOnEventTop : public ccm::kernel::TopModule {

    struct SharedState {
      std::size_t n{1000};
      ccm::kernel::Event e;
    };

    struct ProcessWaitOnEvent : public ccm::kernel::Process {
      ProcessWaitOnEvent(const ccm::kernel::Context & context, SharedState & state)
        : Process(context), state_(state)
      {}
    private:
      void cb__on_initialization() override {
        state_.e.notify(ctxt_.now() + 10);
      }
      void cb__on_invoke() override {
        std::cout << "now = " << ctxt_.now() << "\n";
        if (state_.n != 0) {
          state_.e.notify(ctxt_.now() + 10);
          --state_.n;
        }
      }
      void cb__on_termination() override {
        EXPECT_EQ(state_.n, 0);
        const std::size_t end_time = 10 * 100000;
        EXPECT_EQ(ctxt_.now(), end_time);
      }
      SharedState state_;
    };
  public:
    ProcessWaitOnEventTop(ccm::kernel::Scheduler & sch,
                          const std::string & instance_name = "top")
      : ccm::kernel::TopModule(std::addressof(sch), instance_name) {
      const ccm::kernel::EventBuilder b = ctxt_.event_builder();
      state_.e = b.construct_event();
    
      p_ = create_process<ProcessWaitOnEvent>("p0", state_);
      p_->set_sensitive_on(state_.e);
    }
  private:
    SharedState state_;
    ccm::kernel::Process * p_;
  };

  class ProcessWaitForTestTop : public ccm::kernel::TopModule {

    struct SharedState {
      std::size_t n{100000};
      std::size_t delay{10};
      std::size_t next_time{0};
    };

    struct ProcessWaitFor : public ccm::kernel::Process {
      ProcessWaitFor(const ccm::kernel::Context & context, SharedState & state)
        : Process(context), state_(state)
      {}
    private:
      void cb__on_initialization() override {
        update();
      }
      void cb__on_invoke() override {
        EXPECT_EQ(ctxt_.now(), state_.next_time);
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
    ProcessWaitForTestTop(ccm::kernel::Scheduler & sch,
                          const std::string & instance_name = "top")
      : ccm::kernel::TopModule(std::addressof(sch), instance_name) {
      p_ = create_process<ProcessWaitFor>("p0", state_);
    }
  private:
    void cb__on_elaboration() override {
    }
    SharedState state_;
    ccm::kernel::Process * p_;
  };

  class ProcessWaitUntilTestTop : public ccm::kernel::TopModule {

    struct SharedState {
      std::size_t n{100000};
      std::size_t delay{10};
      std::size_t next_time{0};
    };

    struct ProcessWaitUntil : public ccm::kernel::Process {
      ProcessWaitUntil(const ccm::kernel::Context & context, SharedState & state)
        : Process(context), state_(state)
      {}
    private:
      void cb__on_initialization() override {
        update();
      }
      void cb__on_invoke() override {
        EXPECT_EQ(ctxt_.now(), state_.next_time);
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
    ProcessWaitUntilTestTop(ccm::kernel::Scheduler & sch,
                            const std::string & instance_name = "top")
      : ccm::kernel::TopModule(std::addressof(sch), instance_name) {
      p_ = create_process<ProcessWaitUntil>("p0", state_);
    }
  private:
    SharedState state_;
    ccm::kernel::Process * p_;
  };

} // namespace

TEST(ProcessWaitOnEventTest, t0) {
  ccm::kernel::Scheduler sch;
  sch.set_top(new ProcessWaitOnEventTop(sch));
  sch.run();
}

TEST(ProcessWaitForTest, t0) {
  ccm::kernel::Scheduler sch;
  sch.set_top(new ProcessWaitForTestTop(sch));
  sch.run();
}

TEST(ProcessWaitUntilTest, t0) {
  ccm::kernel::Scheduler sch;
  sch.set_top(new ProcessWaitUntilTestTop(sch));
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
