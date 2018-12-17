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
#include "object.hpp"
#include "context.hpp"

#include <vector>
#include <memory>
#include <string>

namespace ccm::kernel {

class Transaction;

class TopModule;
class Scheduler;

class Module : public Object {
  friend class Scheduler;
  friend class AgentRegistry;
  friend class InterconnectRegistry;

 public:
    
  //
  Module (const Context & ctxt);
  virtual ~Module();

  //
  void set_logger(Logger * logger) override;

 protected:

  //
  template<typename MODULE, typename ... ARGS>
  MODULE * create_child(const std::string & name, ARGS && ... args) {
    log_debug("Constructing child process: ", name);
    
    const Context ctxt = context_.create_child(this, name);
    MODULE * p = new MODULE(ctxt, std::forward<ARGS>(args)...);
    p->set_name(name);
    p->set_parent(this);
    children_.emplace_back(p);
    return p;
  }

  template<typename PROCESS, typename ... ARGS>
  PROCESS * create_process(const std::string & name, ARGS && ... args) {
    log_debug("Constructing child module: ", name);
    
    const Context ctxt = context_.create_child(this, name);
    PROCESS * p = new PROCESS(ctxt, std::forward<ARGS>(args)...);
    p->set_name(name);
    p->set_parent(this);
    processes_.emplace_back(p);
    return p;
  }
  
  //
  virtual void cb__on_elaboration() {}
  virtual void cb__on_initialization() {}
  virtual void cb__on_termination() {}

  //
#define DECLARE_LOG(__name)                                             \
  template<typename ... ARGS> void __name(ARGS && ... args) {           \
    Object::log_ ## __name(prefix(), std::forward<ARGS>(args)...); \
  }

  DECLARE_LOG(fatal)
  DECLARE_LOG(error)
  DECLARE_LOG(warning)
  DECLARE_LOG(info)
  DECLARE_LOG(debug)
#undef DECLARE_LOG

 private:

  //
  void call_on_elaboration();
  void call_on_initialization();
  void call_on_termination();

  //
  std::vector<std::unique_ptr<Module> > children_;
  std::vector<std::unique_ptr<Process> > processes_;
  std::string name_;
};

class TopModule : public Module {
 public:
  TopModule (Scheduler * sch, const std::string & instance_name)
      : Module(Context{sch, nullptr, instance_name})
  {}

  template<typename MODULE, typename ... ARGS>
  static std::unique_ptr<Module> construct(ARGS && ... args) {
    std::unique_ptr<Module> ptr{new MODULE(args...)};
    return ptr;
  }
};

} // namespace ccm::kernel

#endif
