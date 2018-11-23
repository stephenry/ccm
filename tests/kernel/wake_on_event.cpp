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

  class WakeOnEventTop : public ccm::Module {
    enum ProcessID : uint8_t {
      P0, P1, P2
    };
    
    struct State {
      State(ccm::Scheduler & sch)
        : e0_(sch.create_event())
        , e1_(sch.create_event())
        , e2_(sch.create_event())
      {}

      ccm::Event e0_;
      ccm::Event e1_;
      ccm::Event e2_;

      ProcessID prior_id_{ProcessID::P2};
    } state_;
    
    class P0 : public ccm::Process {
    public:
      P0 (State & st, std::size_t n = 10)
        : st_(st), n_(n) {
      }
      ccm::InvokeRsp invoke(ccm::InvokeReq const & req) {
        ccm::InvokeRsp rsp;
        switch (req.state()) {
        case ccm::SimState::Initialization: {
          rsp.wake_after(100);
        } break;
        case ccm::SimState::Running: {
          rsp.wake_on(st_.e0_);
          if (n_-- != 0)
            rsp.notify_after(st_.e1_, 100);
          else
            rsp.terminate();

          EXPECT_EQ(st_.prior_id_, ProcessID::P2);
          st_.prior_id_ = ProcessID::P0;
        } break;
        }
        return rsp;
      }
    private:
      State & st_;
      std::size_t n_;
    } p0_;
    
    class P1 : public ccm::Process {
    public:
      P1 (State & st, std::size_t n = 10)
        : st_(st), n_(n) {
      }
      ccm::InvokeRsp invoke(ccm::InvokeReq const & req) {
        ccm::InvokeRsp rsp;
        rsp.wake_on(st_.e1_);
        
        if (req.state() != ccm::SimState::Running)
          return rsp;
        
        if (n_-- != 0)
          rsp.notify_after(st_.e2_, 100);
        else
          rsp.terminate();

        EXPECT_EQ(st_.prior_id_, ProcessID::P0);
        st_.prior_id_ = ProcessID::P1;
        
        return rsp;
      }
    private:
      State & st_;
      std::size_t n_;
    } p1_;
    
    class P2 : public ccm::Process {
    public:
      P2 (State & st, std::size_t n = 10)
        : st_(st), n_(n) {
      }
      ccm::InvokeRsp invoke(ccm::InvokeReq const & req) {
        ccm::InvokeRsp rsp;
        rsp.wake_on(st_.e2_);
        
        if (req.state() != ccm::SimState::Running)
          return rsp;
        
        if (n_-- != 0)
          rsp.notify_after(st_.e0_, 100);
        else
          rsp.terminate();

        EXPECT_EQ(st_.prior_id_, ProcessID::P1);
        st_.prior_id_ = ProcessID::P2;
        
        return rsp;
      }
    private:
      State & st_;
      std::size_t n_;
    } p2_;
  public:
    WakeOnEventTop(ccm::Scheduler & sch) :
      sch_(sch), state_(sch_), p0_(state_), p1_(state_), p2_(state_) {
      sch_.add_process(std::addressof(p0_));
      sch_.add_process(std::addressof(p1_));
      sch_.add_process(std::addressof(p2_));
    }
    ~WakeOnEventTop() {
      sch_.remove_process(std::addressof(p0_));
      sch_.remove_process(std::addressof(p1_));
      sch_.remove_process(std::addressof(p2_));
    }
  private:
    ProcessID prior_id_{ProcessID::P2};
    ccm::Scheduler & sch_;
  };

  TEST(WakeOnEvent, t0) {
    ccm::Scheduler sch;
    WakeOnEventTop top(sch);
    sch.run();
  }

} // namespace

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
