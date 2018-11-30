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

#include "kernel/kernel.hpp"

#include <unordered_map>
#include <memory>

#define CCM_REGISTER_AGENT(__cls)                                       \
  static struct __cls ## Factory : ::ccm::AgentFactory {                \
    __cls ## Factory () {                                               \
      ccm::AgentRegistry::register_agent(#__cls, this);                 \
    }                                                                   \
    ccm::AgentPtr construct(ccm::AgentOptions & opts) override {        \
      using options_type = typename __cls::options_type;                \
      return std::make_unique<__cls>(static_cast<options_type &>(opts)); \
    }                                                                   \
  } __cls ## Factory

namespace ccm {

  class Transaction;

  class AgentOptions {
  };

  class Agent : public kernel::Module {
  };
  using AgentPtr = std::unique_ptr<Agent>;
  
  struct AgentFactory {
    virtual ccm::AgentPtr construct(AgentOptions & opts) { return {}; };
  };
  using AgentFactoryPtr = std::unique_ptr<AgentFactory>;

  class AgentRegistry {
  public:
    static void register_agent(char const * name, AgentFactory * f);
    static Agent * construct_agent(kernel::Module * m,
                                   char const * name,
                                   AgentOptions & opts);

  private:
    static std::unordered_map<char const *, AgentFactory *> agents_;
  };

  struct BasicSinkAgent : public ccm::Agent {
    virtual void sink_transaction (Transaction * t) = 0;
  };

  struct BasicSourceAgent : public ccm::Agent {
    virtual Transaction * source_transaction() = 0;
  };

} // namespace ccm

#endif
