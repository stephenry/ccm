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

namespace krn = ccm::kernel;

namespace {

  struct EventOrTestTop : krn::TopModule {
    struct {
      bool did_run_invoke{false};
      bool did_run_validate{false};
      void validate() {
        did_run_validate = true;
        EXPECT_TRUE(did_run_invoke);
      }
    } state_;
    
    struct P0 : krn::Process {
      P0(const krn::Context & ctxt, EventOrTestTop * p)
        : Process(ctxt), p_(p) {}
      void cb__on_invoke() override {
        p_->state_.did_run_invoke = true;
        
        EXPECT_EQ(ctxt_.now(), 10);
      }
      EventOrTestTop * p_;
    };
    EventOrTestTop(krn::Scheduler & sch)
      : TopModule(&sch, "t") {
      p0_ = create_process<P0>("P0", this);

      krn::EventBuilder b = ctxt_.event_builder();
      for (int i = 0; i < 3; i++)
        e_.push_back(b.construct_event());

      eor_ = b.construct_or_event(e_.begin(), e_.end());
      p0_->set_sensitive_on(eor_);

      e_[0].notify(10);
    }
    ~EventOrTestTop() {
      EXPECT_TRUE(state_.did_run_validate);
    }
    void cb__on_termination() override {
      state_.validate();
    }
    P0 * p0_;
    std::vector<krn::Event> e_;
    krn::Event eor_;
  };

  struct EventAndTestTop : krn::TopModule {
    EventAndTestTop(krn::Scheduler & sch)
      : TopModule(&sch, "t")
    {}
  };
  
} // namespace

TEST(EventOrTest, basic) {
  krn::Scheduler sch;
  sch.set_top(new EventOrTestTop{sch});
  sch.run();
}

TEST(EventAndTest, basic) {
  krn::Scheduler sch;
  sch.set_top(new EventAndTestTop{sch});
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
