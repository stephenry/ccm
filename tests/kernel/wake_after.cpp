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

class WakeAfterTestTop : public ccm::Module {
  class P0 : public ccm::Process {
   public:
    P0(ccm::Module * m) : ccm::Process() {}
   private:
    ccm::InvokeRsp invoke_initialization(ccm::InvokeReq const & req) override {
      ccm::InvokeRsp rsp;
      rsp.wake_after(100);
      return rsp;
    }
    ccm::InvokeRsp invoke_running(ccm::InvokeReq const & req) override {
      ccm::InvokeRsp rsp;
      switch (n_++) {
        case 0: {
          EXPECT_EQ(req.now(), 100);
          rsp.wake_after(100);
        } break;
        case 1: {
          EXPECT_EQ(req.now(), 200);
          rsp.wake_after(100);
        } break;
        case 2: {
          EXPECT_EQ(req.now(), 300);
          rsp.wake_after(100);
        } break;
        case 3: {
          EXPECT_EQ(req.now(), 400);
          rsp.wake_after(100);
        } break;
        case 4: {
          EXPECT_EQ(req.now(), 500);
          rsp.terminate();
        } break;
      }
      return rsp;
    }
    std::size_t n_{0};
  };
 public:
  WakeAfterTestTop(ccm::Scheduler & sch) : sch_(sch) {
    p_ = new P0(this);
    sch_.add_process(p_);
    sch_.set_top(this);
  }
  ~WakeAfterTestTop() {
    delete p_;
  }
 private:
  P0 * p_;
  ccm::Scheduler & sch_;
};

TEST(WakeAfterTest, t0) {
  ccm::Scheduler sch;
  ModulePtr top = sch.construct_module<WakeAfterTestTop>(sch);
  sch.run();
}

} // namespace

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
