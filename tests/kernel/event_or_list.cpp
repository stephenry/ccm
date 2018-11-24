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

class EventOrListTop : public ccm::Module {
  struct SharedState {
    ccm::EventHandle e[3];
    ccm::EventHandle eOR;
    std::size_t n_p0{0}, n_p1{0};
  } state_;
  
  class P0 : public ccm::Process {
   public:
    P0(SharedState & state) : state_(state) {}
   private:
    ccm::InvokeRsp invoke_initialization(ccm::InvokeReq const & req) override {
      ccm::InvokeRsp rsp;
      rsp.wake_after(100);
      return rsp;
    }
    ccm::InvokeRsp invoke_running(ccm::InvokeReq const & req) override {
      state_.e[state_.n_p0++ % 3].notify();

      ccm::InvokeRsp rsp;
      rsp.wake_after(10);
      return rsp;
    }
   private:
    SharedState & state_;
  } p0_;
  
  class P1 : public ccm::Process {
   public:
    P1(SharedState & state) : state_(state) {}
   private:
    ccm::InvokeRsp invoke_initialization(ccm::InvokeReq const & req) override {
      ccm::InvokeRsp rsp;
      rsp.wake_on(state_.eOR);
      return rsp;
    }
    ccm::InvokeRsp invoke_running(ccm::InvokeReq const & req) override {
      state_.n_p1++;
      
      ccm::InvokeRsp rsp;
      rsp.wake_on(state_.eOR);
      return rsp;
    }
   private:
    SharedState & state_;
  } p1_;

 public:
  EventOrListTop(ccm::Scheduler & sch)
      : p0_(state_), p1_(state_) {
    sch.set_top(this);
    sch.add_process(p0_);
    sch.add_process(p1_);

    state_.e[0] = sch.create_event();
    state_.e[1] = sch.create_event();
    state_.e[2] = sch.create_event();
    //
    state_.eOR = sch.create_event(
        ccm::EventOrList{state_.e[0], state_.e[1], state_.e[2]});
  }
  void on_termination() override {
    EXPECT_EQ(state_.n_p0, state_.n_p1);
  }
};

TEST(EventOrList, t0) {
  ccm::Scheduler sch;
  EventOrListTop top(sch);

  const ccm::RunOptions opts{10000};
  sch.run(opts);
}

} // namespace

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
