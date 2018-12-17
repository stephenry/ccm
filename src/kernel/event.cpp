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

#include "event.hpp"
#include "scheduler.hpp"

#include <algorithm>
#include <vector>

namespace ccm::kernel {

struct RaiseEventAtTime : Task {
  RaiseEventAtTime(EventContext * ctxt, std::size_t t);

  //
  std::size_t time() const override;
  std::string what() const override;

  //
  void apply() override;
 private:
  EventContext * context_;
  std::size_t t_;
};

struct EventContext : ReferenceCounted {
  EventContext(Scheduler * sch, const std::string & name)
      : sch_(sch), name_(name)
  {}

  std::string name() const { return name_; }

  virtual void child_event_notify(EventContext * ec)
  {}
  
  virtual void finalize()
  {}

  void add_to_wait_set(Process * p) {
    ps_.push_back(p);
  }

  void del_from_wait_set(Process * p) {
    ps_.erase(std::remove(ps_.begin(), ps_.end(), p), ps_.end());
  }

  void add_to_wait_set(EventContext * e) {
    es_.push_back(e);
  }

  void del_from_wait_set(EventContext * e) {
    es_.erase(std::remove(es_.begin(), es_.end(), e), es_.end());
  }

  void wake() {
    for (Process * p : ps_)
      sch_->add_process_next_delta(p);
    ps_.clear();
    for (EventContext * e : es_)
      e->child_event_notify(this);
  }
  
  void raise_after(std::size_t t = 0) {
    sch_->add_frontier_task(std::make_unique<RaiseEventAtTime>(this, sch_->now() + t));
  }
  
  void raise_at(std::size_t t = 0) {
    sch_->add_frontier_task(std::make_unique<RaiseEventAtTime>(this, t));
  }
 protected:
  std::vector<Process *> ps_;
  std::vector<EventContext *> es_;
  Scheduler * sch_;
  std::string name_;
};

std::string RaiseEventAtTime::what() const {
  std::stringstream ss;
  ss << "RaiseEventAtTime " << context_->name() << " at: " << time();
  return ss.str();
};

RaiseEventAtTime::RaiseEventAtTime(EventContext * context, std::size_t t)
    : context_(context), t_(t)
{}

void RaiseEventAtTime::apply() {
  context_->wake();
}

std::size_t RaiseEventAtTime::time() const {
  return t_;
}

struct NormalEventContext : EventContext {
  NormalEventContext(Scheduler * sch, const std::string & name)
      : EventContext(sch, name) {}
};

Event EventBuilder::construct_event(const std::string & name) const {
  NormalEventContext * ctxt = new NormalEventContext(sch_, name);
  ctxt->finalize();
  return Event{ctxt};
}

struct OrEventContext : EventContext {
  OrEventContext(Scheduler * sch, const std::string & name)
      : EventContext(sch, name)
  {}
  void child_event_notify(EventContext * ec) override {
    wake();
  }
  void add_event(EventContext * e) {
    e->add_to_wait_set(this);
    es_.push_back(e);
  }
 private:
  std::vector<EventContext *> es_;
};

Event EventBuilder::construct_or_event(const std::vector<Event> & l,
                                       const std::string & name) const {
  OrEventContext * ctxt = new OrEventContext{sch_, name};
  for (Event e : l)
    ctxt->add_event(e.ctxt_);
  ctxt->finalize();
  return Event{ctxt};
}

struct AndEventContext : EventContext {
  AndEventContext(Scheduler * sch, const std::string & name)
      : EventContext(sch, name)
  {}
  void child_event_notify(EventContext * ec) override {
    es_pending_.erase(std::remove(es_pending_.begin(), es_pending_.end(), ec),
                      es_pending_.end());
    if (es_pending_.size() == 0) {
      set_pending();
      wake();
    }
  }
  void finalize() override { set_pending(); }
  void add_event(EventContext * e) {
    e->add_to_wait_set(this);
    es_.push_back(e);
  }
 private:
  void set_pending() { es_pending_ = es_; }
  std::vector<EventContext *> es_pending_;
  std::vector<EventContext *> es_;
};

Event EventBuilder::construct_and_event(const std::vector<Event> & l,
                                        const std::string & name) const {
  AndEventContext * ctxt = new AndEventContext{sch_, name};
  for (Event e : l)
    ctxt->add_event(e.ctxt_);
  ctxt->finalize();
  return Event{ctxt};
}

Event::Event(EventContext * ctxt) : ctxt_(ctxt) {}

Event::Event() : ctxt_{nullptr} {}

Event::~Event() {
  if (is_valid()) ctxt_->dec();
}

Event::Event(const Event & e) {
  if (e.is_valid()) {
    ctxt_ = e.ctxt_;
    ctxt_->inc();
  }
}
Event & Event::operator=(Event e) {
  e.swap(*this);
  if (is_valid())
    ctxt_->inc();
  return *this;
}

bool Event::is_valid() const {
  return ctxt_ != nullptr;
}

void Event::add_to_wait_set(Process * p) {
  ctxt_->add_to_wait_set(p);
}

void Event::notify(std::size_t t) {
  if (t == 0)
    ctxt_->wake();
  else
    ctxt_->raise_after(t);
}

} // namespace ccm::kernel
