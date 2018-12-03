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

  //  namespace krn = ccm::kernel;
  namespace agt = ccm::agents;

  struct State {
    std::size_t n{10000};
    std::size_t sender_id;
    std::size_t consumer_id;
    std::deque<std::size_t> sent;
  };

  struct BasicTransaction : krn::Transaction {
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
    CCM_AGENT_COMMON(Producer);
    struct Arguments : krn::AgentArguments {
      std::size_t period{16};
    };
    
    using arg_type = Arguments;
    Producer(arg_type & arg)
      : arg_(arg), agt::BasicSourceAgent(arg.period)
    {}
    void set_state(State * state) { state_ = state; }
    krn::Transaction * source_transaction() override {
      if (state_->n-- == 0)
        return nullptr;
      
      BasicTransaction * bt = p_.alloc();
      EXPECT_EQ(arg_.id, state_->sender_id);
      bt->portid_src = arg_.id;
      bt->portid_dst = state_->consumer_id;
      EXPECT_TRUE(!bt->is_valid());
      bt->set(ccm::rand_int());
      return bt;
    }
  private:
    arg_type & arg_;
    ccm::Pool<BasicTransaction> p_;
    State * state_;
  };

  struct Consumer : agt::BasicSinkAgent {
    CCM_AGENT_COMMON(Consumer);

    struct Arguments : krn::AgentArguments {};

    using arg_type = Arguments;
    Consumer(arg_type & arg)
      : arg_(arg)
    {}
    void set_state(State * state) { state_ = state; }
    void sink_transaction (krn::Transaction * t) override {
      BasicTransaction * bt = static_cast<BasicTransaction *>(t);
      EXPECT_TRUE(bt->is_valid());
      EXPECT_EQ(bt->portid_dst, arg_.id);
      EXPECT_EQ(bt->portid_src, state_->sender_id);
      EXPECT_EQ(bt->portid_dst, state_->consumer_id);
      const std::size_t expected = state_->sent.front();
      const std::size_t actual = bt->value();
      EXPECT_EQ(expected, actual);
      state_->sent.pop_front();
      bt->release();
    }
  private:
    State * state_;
    arg_type & arg_;
  };

  class Top : public krn::Module {
  public:
    Top(std::string name) : krn::Module(name) {
      ccm::interconnects::register_interconnects(areg_);
      areg_.register_agent<Producer>();
      areg_.register_agent<Consumer>();

      pargs_.id = 0;
      pargs_.instance_name = "P";
      producer_ = static_cast<Producer *>(areg_.construct(this, "Producer", pargs_));
      producer_->set_state(&state_);

      cargs_.id = 1;
      cargs_.instance_name = "C";
      consumer_ = static_cast<Consumer *>(areg_.construct(this, "Consumer", cargs_));
      producer_->set_state(&state_);

      fargs_.id = 2;
      fargs_.instance_name = "F";
      fargs_.in_ports = 1;
      fargs_.out_ports = 1;
      fixed_latency_ = static_cast<
        ccm::interconnects::FixedLatency *>(areg_.construct(this, "FixedLatency", fargs_));

      producer_->out_ = fixed_latency_->ins_[0];
      fixed_latency_->outs_[0] = consumer_->in_;
    }
  private:
    
    Producer::Arguments pargs_;
    Consumer::Arguments cargs_;
    ccm::interconnects::FixedLatency::Arguments fargs_;
    krn::AgentRegistry areg_;
    State state_;
    Producer * producer_;
    Consumer * consumer_;
    ccm::interconnects::FixedLatency * fixed_latency_;
  };
  
} // namespace

TEST(Basic, t0) {
  krn::Scheduler sch;
  {
    krn::ModulePtr top = sch.construct_top<Top>("top");
    sch.set_top(std::move(top));
  }
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
