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

#ifndef __FIXED_LATENCY_HPP__
#define __FIXED_LATENCY_HPP__

#include "kernel/kernel.hpp"
#include <vector>

namespace ccm::interconnects {

  class FixedLatency : public kernel::Agent {
    using event_queue_type = kernel::EventQueue<kernel::Transaction *>;
    
    struct PushProcess;
    struct PopProcess;
  public:
    CCM_AGENT_COMMON(FixedLatency);

    struct Arguments : kernel::AgentArguments {
      std::size_t in_ports;
      std::size_t out_ports;
      std::size_t latency;
    };
    using arg_type = Arguments;
    
    FixedLatency(const arg_type & arg);
    void push (kernel::Transaction * t);
    
    std::vector<kernel::TMailBox *> ins_;
    std::vector<kernel::TMailBoxIf *> outs_;
  private:
    virtual void cb__on_initialization() override;

    std::vector<kernel::EventQueue<kernel::Transaction *> *> eqs_;
    arg_type arg_;

    std::vector<PushProcess *> p_push_;
    std::vector<PopProcess *> p_pop_;
  };

} // namespace ccm::interconnects

#endif
