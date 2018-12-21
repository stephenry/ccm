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

#include "ccm.hpp"

#include <deque>
#include <gtest/gtest.h>

namespace {

namespace krn = ::ccm::kernel;
namespace agt = ::ccm::agents;
namespace itc = ::ccm::interconnects;

struct State {
  std::size_t n{10000};
  std::size_t sender_id;
  std::size_t consumer_id;
  std::deque<std::size_t> sent;
};

struct BasicTransaction : itc::FixedLatencyTransaction {
  void reset() override {
    is_valid_ = false;
  }
  bool is_valid() const { return is_valid_; }
  std::size_t value() const { return value_; }
  void set(std::size_t value) {
    is_valid_ = true;
    value_ = value;
  }
 private:
  bool is_valid_;
  std::size_t value_;
};

struct Producer : agt::BasicSourceAgent {
  CCM_BUILDABLE_COMMON(Producer);
  struct Arguments : krn::BuildableArguments {
    Arguments(std::size_t id, const std::string & instance_name)
        : BuildableArguments(id, instance_name)
    {}
    std::size_t period{16};
  };
  Producer(const krn::Context & ctxt, const Arguments & args)
      : agt::BasicSourceAgent(ctxt, args.period), args_(args)
  {}
  void set_state(State * state) { state_ = state; }
  krn::Transaction * source_transaction() override {
    if (state_->n-- == 0)
      return nullptr;

    BasicTransaction * bt = p_.alloc();
    EXPECT_EQ(args_.id_, state_->sender_id);
    bt->mid = args_.id_;
    bt->sid = state_->consumer_id;
    EXPECT_TRUE(!bt->is_valid());
    bt->set(0);
    // TODO
    //    bt->set(krn::Random::uniform());
    return bt;
  }
 private:
  Arguments args_;
  ccm::Pool<BasicTransaction> p_;
  State * state_;
};

struct Consumer : agt::BasicSinkAgent {
  CCM_BUILDABLE_COMMON(Consumer);
  struct Arguments : krn::BuildableArguments {
    Arguments(std::size_t id, const std::string & instance_name)
        : BuildableArguments(id, instance_name)
    {}
  };
  struct {
    std::size_t received_count_{0};
    bool is_valid() const {
      if (received_count_ == 0)
        return false;

      return true;
    }
  } validate_;
  
  Consumer(const krn::Context & ctxt, const Arguments & args)
      : BasicSinkAgent(ctxt), args_(args)
  {}
  void set_state(State * state) { state_ = state; }
  void sink_transaction (krn::Transaction * t) override {
    validate_.received_count_++;
    
    BasicTransaction * bt = static_cast<BasicTransaction *>(t);
    EXPECT_TRUE(bt->is_valid());
    EXPECT_EQ(bt->sid, args_.id_);
    EXPECT_EQ(bt->mid, state_->sender_id);
    EXPECT_EQ(bt->sid, state_->consumer_id);
    const std::size_t expected = state_->sent.front();
    const std::size_t actual = bt->value();
    EXPECT_EQ(expected, actual);
    state_->sent.pop_front();
    bt->release();
  }
  ~Consumer() {
    EXPECT_TRUE(validate_.is_valid());
  }
 private:
  State * state_;
  Arguments args_;
};

class Top : public krn::TopModule {
 public:
  Top(const std::string & instance_name,
      ccm::kernel::Scheduler & sch)
      : ccm::kernel::TopModule(std::addressof(sch), instance_name) {
      
    ccm::interconnects::register_interconnects(breg_);
    breg_.register_agent<Producer>();
    breg_.register_agent<Consumer>();

    Producer::Arguments pargs{0, "P"};
    producer_ = static_cast<Producer *>(breg_.construct(context(), "Producer", pargs));
    producer_->set_state(&state_);

    Consumer::Arguments cargs{1, "C"};
    consumer_ = static_cast<Consumer *>(breg_.construct(context(), "Consumer", cargs));
    producer_->set_state(&state_);

    ccm::interconnects::FixedLatency::Arguments fargs{2, "F"};
    fargs.in_ports = 1;
    fargs.out_ports = 1;
    fixed_latency_ = static_cast<
      ccm::interconnects::FixedLatency *>(breg_.construct(context(), "FixedLatency", fargs));

    producer_->out_ = fixed_latency_->ins_[0];
    fixed_latency_->outs_[0] = consumer_->in_;
  }
 private:
  krn::BuildableRegistry breg_;
  State state_;
  Producer * producer_;
  Consumer * consumer_;
  ccm::interconnects::FixedLatency * fixed_latency_;
};
  
} // namespace

TEST(Basic, t0) {
  krn::Scheduler sch;
  std::unique_ptr<krn::Module> top =
      krn::TopModule::construct<Top>("top", sch);
  std::unique_ptr<krn::Logger> logger = std::make_unique<krn::Logger>();
  top->set_logger(logger.get());
  sch.set_top(top.get());
  sch.set_logger(logger.get());
  const krn::RunOptions opts(100000);
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
