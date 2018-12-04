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

#ifndef __MODULE_HPP__
#define __MODULE_HPP__

#include "event.hpp"

#include <vector>
#include <memory>
#include <string>

namespace ccm::kernel {

  class Transaction;

  class TopModule;
  class Module;
  class Scheduler;

  class Context {
    friend class Scheduler;
    friend class Module;
    friend class TopModule;
    
    Context(Scheduler * sch, Module * parent, const std::string & instance_name)
      : sch_(sch), parent_(parent), instance_name_(instance_name) {}

    Context create_child(Module * self, const std::string & instance_name) {
      return Context{sch_, self, instance_name};
    }
    
  public:
    std::size_t now() const;
    std::size_t delta() const;
    std::string instance_name() const;
    EventBuilder event_builder() const;
    Scheduler * sch() const;

  private:
    Scheduler * sch_;
    std::string instance_name_;
    Module * parent_{nullptr};
  };

  class Module {
    friend class Scheduler;
    friend class AgentRegistry;
    friend class InterconnectRegistry;

  public:
    
    //
    Module (const Context & ctxt);
    virtual ~Module();

  protected:

    //
    template<typename MODULE, typename ...ARGS>
    MODULE * create_child(const std::string & instance_name, ARGS && ... args) {
      const Context ctxt = ctxt_.create_child(this, instance_name);
      MODULE * ptr = new MODULE(ctxt, args...);
      children_.push_back(ptr);
      return ptr;
    }

    template<typename PROCESS, typename ...ARGS>
    PROCESS * create_process(const std::string & name, ARGS && ... args) {
      const Context ctxt = ctxt_.create_child(this, name);
      PROCESS * ptr = new PROCESS(ctxt, args...);
      processes_.push_back(ptr);
      return ptr;
    }
  
    //
    virtual void cb__on_elaboration() {};
    virtual void cb__on_initialization() {};
    virtual void cb__on_termination() {};

    //
    Context ctxt_;
  private:

    //
    void call_on_elaboration();
    void call_on_initialization();
    void call_on_termination();

    //
    std::vector<Module *> children_;
    std::vector<Process *> processes_;
    std::string name_;
  };

  class TopModule : public Module {
  public:
    TopModule (Scheduler * sch, const std::string & instance_name)
      : Module(Context{sch, nullptr, instance_name}) {}
  };

} // namespace ccm::kernel

#endif
