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

struct RunForTime : public ccm::kernel::Module {

  class P0 : public ccm::kernel::Process {
  public:
    P0(RunForTime * prnt)
      : prnt_(prnt)
    {}
  private:
    virtual void cb__on_initialization() override {
      wait_for(10);
    }
    virtual void cb__on_invoke() override {
      loop_one();
    }
    void loop_one() {
      if (prnt_->n_-- != 0)
        wait_for(10);
    }
    RunForTime * prnt_;
  };

  RunForTime(const std::string & name)
    : Module(name) {
    create_process<P0>(this);
  }
  std::size_t n_{100};
};

} // namespace

TEST(RunForTime, t_100) {
  ccm::kernel::Scheduler sch;

  ccm::kernel::ModulePtr top = sch.construct_top<RunForTime>("RunForTime");
  sch.set_top(std::move(top));
  
  ccm::kernel::RunOptions opts{100};
  sch.run(opts);
  EXPECT_EQ(sch.now(), 100);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
