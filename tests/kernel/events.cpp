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
#include <set>
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
    std::vector<krn::Event> e;
    std::vector<std::size_t> times;
  } state_;

  struct P0 : krn::Process {
    P0(const krn::Context & ctxt, EventOrTestTop * p)
        : Process(ctxt), p_(p) {}
    void cb__on_invoke() override {
      std::size_t t = 10;
      p_->state_.times.push_back(context().now() + t);
      const std::size_t e_id = krn::Random::uniform(p_->state_.e.size());
      p_->state_.e[e_id].notify(t);

      if (n_++ == 1000)
        kill();
    }
    EventOrTestTop * p_;
    std::size_t n_{0};
  };

  struct P1 : krn::Process {
    P1(const krn::Context & ctxt, EventOrTestTop * p)
        : Process(ctxt), p_(p) {}
    void cb__on_invoke() override {
      p_->state_.did_run_invoke = true;
      ASSERT_FALSE(p_->state_.times.empty());
        
      EXPECT_EQ(context().now(), p_->state_.times.back());
      p_->state_.times.pop_back();
    }
    EventOrTestTop * p_;
  };

  EventOrTestTop(krn::Scheduler & sch)
      : TopModule(&sch, "t") {
    p0_ = create_process<P0>("P0", this);
    p0_->set_periodic(1000);
    p1_ = create_process<P1>("P1", this);
    krn::EventBuilder b = context().event_builder();
    for (int i = 0; i < 3; i++)
      state_.e.push_back(b.construct_event());
    eor_ = b.construct_or_event(state_.e.begin(), state_.e.end(), "EOR");
    p1_->set_sensitive_on(eor_);
  }
  ~EventOrTestTop() {
    EXPECT_TRUE(state_.did_run_validate);
  }
  void cb__on_termination() override {
    state_.validate();
  }
  P0 * p0_;
  P1 * p1_;
  krn::Event eor_;
};

struct EventAndTestTop : krn::TopModule {
  struct {
    bool did_run_invoke{false};
    bool did_run_validate{false};
    void validate() {
      did_run_validate = true;
      EXPECT_TRUE(did_run_invoke);
    }
    std::vector<krn::Event> e;
    std::set<std::size_t> set;
  } state_;
  struct P0 : krn::Process {
    P0(const krn::Context & ctxt, EventAndTestTop * p)
        : Process(ctxt), p_(p) {}
    void cb__on_invoke() override {
      log_info("Invoked");
      std::size_t t = 10;
      const std::size_t e_id = krn::Random::uniform(p_->state_.e.size());
      log_info("Raise event: ", e_id);
      p_->state_.e[e_id].notify(t);
      log_info("Set size (before): ", p_->state_.set.size());
      p_->state_.set.insert(e_id);
      log_info("Set size (after): ", p_->state_.set.size());

      if (n_++ == 1000)
        kill();
    }
    EventAndTestTop * p_;
    std::size_t n_{0};
  };
  struct P1 : krn::Process {
    P1(const krn::Context & ctxt, EventAndTestTop * p)
        : Process(ctxt), p_(p) {}
    void cb__on_invoke() override {
      log_info("Invoked");
      EXPECT_EQ(p_->state_.set.size(), p_->state_.e.size());
      p_->state_.set.clear();
      p_->state_.did_run_invoke = true;
    }
    EventAndTestTop * p_;
  };
  EventAndTestTop(krn::Scheduler & sch)
      : TopModule(&sch, "t") {
    p0_ = create_process<P0>("P0", this);
    p0_->set_periodic(1000);
    p1_ = create_process<P1>("P1", this);
    krn::EventBuilder b = context().event_builder();
    for (int i = 0; i < 3; i++) {
      std::stringstream ss;
      ss << "E" << i;
      state_.e.push_back(b.construct_event(ss.str()));
    }
    eand_ = b.construct_and_event(state_.e.begin(), state_.e.end(), "EAND");
    p1_->set_sensitive_on(eand_);
  }
  ~EventAndTestTop() {
    EXPECT_TRUE(state_.did_run_validate);
  }
  void cb__on_termination() override {
    state_.validate();
  }
  P0 * p0_;
  P1 * p1_;
  krn::Event eand_;
};
  
} // namespace

TEST(EventOrTest, basic) {
  krn::Random::init(1);
  
  krn::Scheduler sch;
  std::unique_ptr<krn::Module> top = krn::TopModule::construct<EventOrTestTop>(sch);
  std::unique_ptr<krn::Logger> logger = std::make_unique<krn::Logger>();
  top->set_logger(logger.get());
  sch.set_top(top.get());
  sch.set_logger(logger.get());
  const krn::RunOptions opts(1000000);
  sch.run(opts);
}

TEST(EventAndTest, basic) {
  krn::Random::init(1);
  
  krn::Scheduler sch;
  std::unique_ptr<krn::Module> top = krn::TopModule::construct<EventAndTestTop>(sch);
  std::unique_ptr<krn::Logger> logger = std::make_unique<krn::Logger>();
  top->set_logger(logger.get());
  sch.set_top(top.get());
  sch.set_logger(logger.get());
  const krn::RunOptions opts(100000);
  sch.run(opts);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
