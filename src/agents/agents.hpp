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

#ifndef __AGENTS_HPP__
#define __AGENTS_HPP__

#include "agents.hpp"
#include "kernel/kernel.hpp"

#include <unordered_map>
#include <memory>

#define CCM_REGISTER_AGENT(__name, __af)                                \
  static ::ccm::AgentRegisterer __reg_ ## __af(__name, std::make_unique<__af>())

namespace ccm {

  class Agent : public kernel::Module {
  };
  using AgentPtr = std::unique_ptr<Agent>;
  
  class AgentFactory {
  };
  using AgentFactoryPtr = std::unique_ptr<AgentFactory>;

  class AgentRegistry {
  public:

    //
    static void register_agent(char const * name, AgentFactoryPtr && f);

  private:
    //
    static std::unordered_map<char const *, AgentFactoryPtr> agents_;
  };

  struct AgentRegisterer {
    AgentRegisterer(const char * name, AgentFactoryPtr && f);
  };

} // namespace ccm

#endif
