#ifndef __EVENT_HPP__
#define __EVENT_HPP__

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

#include <memory>
#include <vector>

namespace ccm {

class Process;
class Scheduler;
class EventDescriptor;

class EventHandle {
  friend class Scheduler;
  
  friend bool operator==(EventHandle const & a, EventHandle const & b);
  friend bool operator!=(EventHandle const & a, EventHandle const & b);

  EventHandle(EventDescriptor * ed) : ed_(ed) {}
 public:
  EventHandle() : ed_{nullptr} {}
  bool is_valid() const;
  void notify();
  void add_to_wait_set(Process * p);
  void remove_from_wait_set(Process *p);
 private:
  EventDescriptor *ed_{nullptr};
};

using EventOrList = std::vector<EventHandle>;

class EventDescriptor {
  friend class Scheduler;
 protected:
  EventDescriptor(Scheduler * sch) : sch_(sch) {}
 public:
  virtual void notify(EventHandle h);
  virtual void add_to_wait_set(Process * p);
  virtual void remove_from_wait_set(Process * p);
  virtual ~EventDescriptor() {}
 protected:
  std::vector<Process *> suspended_on_;
  Scheduler * sch_;
};

class EventOrDescriptor : public EventDescriptor {
  friend class Scheduler;

  EventOrDescriptor(Scheduler * sch, EventOrList const & el)
      : EventDescriptor(sch), el_(el) {}
 public:
  void notify(EventHandle h) override;
 private:
  EventOrList const & el_;
};

using EventDescriptorPtr = std::unique_ptr<EventDescriptor>;

} // namespace ccm

#endif
