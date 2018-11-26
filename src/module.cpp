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

#include "module.hpp"
#include "scheduler.hpp"
#include <algorithm>

namespace ccm {

  Module::~Module () {
  }

  SimState Module::state() const { return sch_->state(); }
  std::size_t Module::now() const { return sch_->now(); }
  std::size_t Module::delta() const { return sch_->delta(); }
  
  //
  EventHandle Module::create_event() {
    return sch_->create_event();
  }

  //
  EventHandle Module::create_event(EventOrList const & e) {
    return sch_->create_event(e);
  }

  //
  void Module::add_child (ModulePtr && ptr) {
    children_.push_back(std::move(ptr));
  }

  void Module::call_on_elaboration(ElaborationState const & state) {
    std::for_each(children_.begin(), children_.end(),
                  [=](ModulePtr & ptr) {
                    ptr->call_on_elaboration(state);
                  });
    std::for_each(processes_.begin(), processes_.end(),
                  [=](ProcessPtr & ptr) {
                    ptr->call_on_elaboration(state);
                  });
    set_scheduler(state.sch);
    cb__on_elaboration();
  }
  
  void Module::call_on_initialization() {
    std::for_each(children_.begin(), children_.end(),
                  [=](ModulePtr & ptr) {
                    ptr->call_on_initialization();
                  });
    std::for_each(processes_.begin(), processes_.end(),
                  [=](ProcessPtr & ptr) {
                    ptr->call_on_initialization();
                  });
    cb__on_initialization();
  }
  
  void Module::call_on_termination() {
    std::for_each(children_.begin(), children_.end(),
                  [=](ModulePtr & ptr) {
                    ptr->call_on_termination();
                  });
    std::for_each(processes_.begin(), processes_.end(),
                  [=](ProcessPtr & ptr) {
                    ptr->call_on_termination();
                  });
    cb__on_termination();
  }

} // namespace ccm
