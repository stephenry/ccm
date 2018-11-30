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
#include "interconnects/fixed_latency.hpp"

#include <deque>
#include <gtest/gtest.h>

namespace {

  struct SharedState : ccm::AgentArguments {
    std::size_t period{16};
    std::size_t n{10000};
    std::size_t sender_id;
    std::size_t consumer_id;
    std::deque<std::size_t> sent;
  };

  struct BasicTransaction : ccm::Transaction {
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

  struct Producer : ccm::BasicSourceAgent {
    CCM_REGISTER_AGENT(Producer);
    
    using arg_type = ccm::AgentStateBase<SharedState>;
    Producer(arg_type & arg)
      : arg_(arg), ccm::BasicSourceAgent(arg.state.period)
    {}
    ccm::Transaction * source_transaction() override {
      if (arg_.state.n-- == 0)
        return nullptr;
      
      BasicTransaction * bt = p_.alloc();
      EXPECT_EQ(arg_.id, arg_.state.sender_id);
      bt->portid_src = arg_.id;
      bt->portid_dst = arg_.state.consumer_id;
      EXPECT_TRUE(!bt->is_valid());
      bt->set(ccm::rand_int());
      return bt;
    }
  private:
    arg_type & arg_;
    ccm::Pool<BasicTransaction> p_;
  };

  struct Consumer : ccm::BasicSinkAgent {
    CCM_REGISTER_AGENT(Consumer);
    
    using arg_type = ccm::AgentStateBase<SharedState>;
    Consumer(arg_type & arg)
      : arg_(arg)
    {}
    void sink_transaction (ccm::Transaction * t) override {
      SharedState & state = arg_.state;
      
      BasicTransaction * bt = static_cast<BasicTransaction *>(t);
      EXPECT_TRUE(bt->is_valid());
      EXPECT_EQ(bt->portid_dst, arg_.id);
      EXPECT_EQ(bt->portid_src, arg_.state.sender_id);
      EXPECT_EQ(bt->portid_dst, arg_.state.consumer_id);
      const std::size_t expected = state.sent.front();
      const std::size_t actual = bt->value();
      EXPECT_EQ(expected, actual);
      state.sent.pop_front();
      bt->release();
    }
  private:
    arg_type & arg_;
  };

  class Top : public ccm::kernel::Module {
  public:
    Top(std::string name) : ccm::kernel::Module(name) {
      pid_ = ccm::Agent::get_unique_id();
      cid_ = ccm::Agent::get_unique_id();
      
      ccm::AgentStateBase<SharedState> pstate{shared_state_};
      pstate.id = pid_;
      producer_ = ccm::AgentRegistry::construct(this, "Producer", pstate);

      ccm::AgentStateBase<SharedState> cstate{shared_state_};
      cstate.id = cid_;
      consumer_ = ccm::AgentRegistry::construct(this, "Consumer", cstate);

      ccm::FixedLatencyArguments args;
      args.latency = 16;
      interconnect_ = ccm::InterconnectRegistry::construct(this, "FixedLatency", args);
      interconnect_->register_agent(pid_, producer_);
      interconnect_->register_agent(cid_, consumer_);
    }
  private:
    std::size_t pid_, cid_;
    SharedState shared_state_;
    ccm::Agent *producer_, *consumer_;
    ccm::Interconnect  *interconnect_;
  };
  
} // namespace

TEST(Basic, t0) {
  ccm::kernel::Scheduler sch;
  {
    ccm::kernel::ModulePtr top = sch.construct_top<Top>("top");
    sch.set_top(std::move(top));
  }
  sch.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
