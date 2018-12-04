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

#ifndef __EVENT_HPP__
#define __EVENT_HPP__

#include "common.hpp"
#include <vector>

namespace ccm::kernel {

  class Process;
  class Scheduler;
  class EventContext;

  class Event {
    friend class Process;
    friend class EventBuilder;

    Event(EventContext * ctxt);
  public:
    Event();
    ~Event();

    Event(const Event & h);
    Event & operator=(Event h);
    
    bool is_valid() const;
    void notify(std::size_t t = 0);
    void swap(Event & h) { std::swap(ctxt_, h.ctxt_); }

  private:
    void add_to_wait_set(Process * p);
    
    EventContext *ctxt_{nullptr};
  };

  class EventBuilder {
    friend class Scheduler;
    friend class Context;

    EventBuilder(Scheduler * sch)
      : sch_(sch)
    {}

  public:
    using list_type = std::vector<Event>;
    
    Event construct_event() const;
    Event construct_and_event(const list_type & l) const;
    Event construct_or_event(const list_type & l) const;
    
  private:
    mutable Scheduler * sch_{nullptr};
  };

} // namespace ccm::kernel

#endif
