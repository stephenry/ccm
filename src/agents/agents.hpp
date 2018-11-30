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
      ::ccm::AgentRegistry::register_agent(#__cls, this);               \
    }                                                                   \
    ::ccm::AgentPtr construct(::ccm::AgentArguments & args) override {  \
      using arg_type = typename __cls::arg_type;                        \
      return std::make_unique<__cls>(static_cast<arg_type &>(args));    \
    }                                                                   \
  } __cls ## Factory

namespace ccm {

  class Transaction;

  struct AgentArguments {
    std::size_t id;
  };

  template<typename STATE>
  struct AgentStateBase : AgentArguments {
    AgentStateBase(STATE & state)
      : state(state)
    {}
    STATE & state;
  };

  class Agent : public kernel::Module {
  public:
    static std::size_t get_unique_id();

    Agent(kernel::PortType type)
      : port_("port", type)
    {}
    kernel::Port & port() { return port_; }

  private:
    static std::size_t id_;
    
    kernel::Port port_;
  };
  using AgentPtr = std::unique_ptr<Agent>;
  
  struct AgentFactory {
    virtual ccm::AgentPtr construct(AgentArguments & opts) = 0;
  };

  class AgentRegistry {
  public:
    static void register_agent(char const * name, AgentFactory * f);
    static Agent * construct(kernel::Module * m,
                             char const * name,
                             AgentArguments & args);

  private:
    static std::unordered_map<char const *, AgentFactory *> agents_;
  };

  class BasicSourceAgent : public ccm::Agent {
    
    struct WakeProcess : kernel::Process {
      WakeProcess(BasicSourceAgent * agnt, std::size_t period)
        : agnt_(agnt), period_(period)
      {}
      
      void cb__on_initialization() override;
      void cb__on_invoke() override;
    private:
      std::size_t period_;
      BasicSourceAgent * agnt_;
    };
  public:
    BasicSourceAgent(std::size_t period);
  protected:
    virtual Transaction * source_transaction() = 0;
  private:
    WakeProcess * p;
    std::size_t period_;
  };

  struct BasicSinkAgent : public ccm::Agent {
    BasicSinkAgent();
  protected:
    virtual void sink_transaction (Transaction * t) = 0;
  };

} // namespace ccm

#endif
