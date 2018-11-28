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

#include "process.hpp"
#include "scheduler.hpp"

#include <vector>
#include <memory>
#include <string>

namespace ccm::kernel {

class Module {
  friend class Scheduler;

public:

  //
  Module () : Module("<ANONYMOUS>") {}
  Module (std::string name) : name_(name) {}
  
  //
  virtual ~Module();
  
  //
  SimState state() const;
  std::size_t now() const;
  std::size_t delta() const;

protected:

  //
  EventHandle create_event();
  EventHandle create_event(EventOrList const & e);

  //
  template<typename MODULE, typename ...ARGS>
  MODULE * create_child(ARGS && ... args) {
    ModulePtr ptr = std::make_unique<MODULE>(args...);
    ptr->set_parent(this);

    MODULE * ret = static_cast<MODULE *>(ptr.get());
    children_.push_back(std::move(ptr));
    return ret;
  }

  template<typename PROCESS, typename ...ARGS>
  PROCESS * create_process (ARGS && ... args) {
    ProcessPtr ptr = std::make_unique<PROCESS>(args...);
    ptr->set_parent(this);

    PROCESS * ret = static_cast<PROCESS *>(ptr.get());
    processes_.push_back(std::move(ptr));
    return ret;
  }
  
  //
  void add_child (ModulePtr && ptr);
  
  //
  virtual void cb__on_elaboration() {};
  virtual void cb__on_initialization() {};
  virtual void cb__on_termination() {};

private:

  //
  void call_on_elaboration(ElaborationState const & state);
  void call_on_initialization();
  void call_on_termination();
  
  //
  void set_scheduler(Scheduler * sch) { sch_ = sch; }
  void set_parent(Module * parent) { parent_ = parent; }

  //
  Scheduler * sch_{nullptr};
  Module * parent_{nullptr};

  //
  std::vector<ModulePtr> children_;
  std::vector<ProcessPtr> processes_;
  std::string name_;
};

} // namespace ccm::kernel

#endif
