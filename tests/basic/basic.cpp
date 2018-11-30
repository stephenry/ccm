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

  struct State : ccm::AgentOptions {
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
    using options_type = State;
    Producer(options_type & options)
      : options_(options)
    {}
    ccm::Transaction * source_transaction() override {
      BasicTransaction * t = p_.alloc();
      t->set(ccm::rand_int());
      return t;
    }
  private:
    options_type & options_;
    ccm::Pool<BasicTransaction> p_;
  };
  CCM_REGISTER_AGENT("Producer", Producer);

  struct Consumer : ccm::BasicSinkAgent {
    using options_type = State;
    Consumer(options_type & options)
      : options_(options)
    {}
    void sink_transaction (ccm::Transaction * t) override {
      BasicTransaction * bt = static_cast<BasicTransaction *>(t);
      EXPECT_TRUE(bt->is_valid());
      const std::size_t expected = options_.sent.front();
      const std::size_t actual = bt->value();
      EXPECT_EQ(expected, actual);
      options_.sent.pop_front();
      bt->release();
    }
  private:
    options_type & options_;
  };
  CCM_REGISTER_AGENT("Consumer", Consumer);

  class Top : ccm::kernel::Module {
  public:
    Top(std::string name) : ccm::kernel::Module(name) {
      producer_ = ccm::AgentRegistry::construct_agent(this, "Producer", state_);
      consumer_ = ccm::AgentRegistry::construct_agent(this, "Consumer", state_);
    }
  private:
    State state_;
    ccm::Agent *producer_, *consumer_;
  };
  
} // namespace

TEST(Basic, t0) {
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
