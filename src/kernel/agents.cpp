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

#include "agents.hpp"

namespace ccm::kernel {

  std::size_t Agent::id_{0};
  std::size_t Agent::get_unique_id() { return Agent::id_++; }
  
  std::unordered_map<char const *, AgentFactory *> AgentRegistry::agents_;

  void AgentRegistry::register_agent (const char * name, AgentFactory * f) {
    agents_[name] = f;
  }

  Agent * AgentRegistry::construct(Module * m, char const * name, AgentArguments & args) {
    AgentPtr ptr = agents_[name]->construct(args);
    Agent *ret{ptr.get()};
    m->add_child(std::move(ptr));
    return ret;
  }

  void BasicSourceAgent::WakeProcess::cb__on_initialization() {
    wait_until(now() + period_);
  }
  
  void BasicSourceAgent::WakeProcess::cb__on_invoke() {
    Transaction * t = agnt_->source_transaction();

    if (t != nullptr)
      wait_until(now() + period_);
  }
  
  BasicSourceAgent::BasicSourceAgent(std::size_t period)
    : Agent(PortType::Out), period_(period) {
    p = create_process<WakeProcess>(this, period);
  }

  BasicSinkAgent::BasicSinkAgent()
    : Agent(PortType::In) {
  }

} // namespace ccm::kernel
