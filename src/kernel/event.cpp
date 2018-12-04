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

  struct Waitable : Poolable {
    virtual void notify() = 0;
  };

  struct ProcessWaitable : Waitable {
    void reset() { sch_ = nullptr; p_ = nullptr; }
    void set_sch(Scheduler * sch) { sch_ = sch; }
    void set_p(Process * p) { p_ = p; }
    void notify() override {
      if (p_)
        sch_->add_process_next_delta(p_);
    }
  private:
    Scheduler * sch_;
    Process * p_;
  };

  struct EventWaitable : Waitable {
    void reset() {}
    void set_e(Event e) { e_ = e; }
    void notify() override { e_.notify(); }
  private:
    Event e_;
  };

  struct EventContext : ReferenceCounted {
    EventContext(Scheduler * sch)
      : sch_(sch)
    {}

    virtual void notify(std::size_t t = 0) = 0;
    void add_to_wait_set(Process * p) {
      static Pool<ProcessWaitable> pool_;

      ProcessWaitable * w = pool_.alloc();
      w->set_sch(sch_);
      w->set_p(p);
      waiting_.push_back(w);
    }
    void add_to_wait_set(Event e) {
      static Pool<EventWaitable> pool_;

      EventWaitable * w = pool_.alloc();
      w->set_e(e);
      waiting_.push_back(w);
    }
    void wake() {
      for (Waitable * w : waiting_) {
        w->notify();
        w->release();
      }
      waiting_.clear();
    }
  protected:
    std::vector<Waitable *> waiting_;
    Scheduler * sch_;
  };

  struct NotifyEventFrontierTask : Frontier::Task {
    void apply () override { ctxt_->wake(); }
    std::size_t time () const override { return time_; }
    void reset() override { ctxt_ = nullptr; }
    std::size_t time_;
    EventContext * ctxt_;
  };

  struct NormalEventContext : EventContext {
    NormalEventContext(Scheduler * sch)
      : EventContext(sch)
    {}
    void notify(std::size_t t = 0) override {
      static Pool<NotifyEventFrontierTask> pool_;
      
      NotifyEventFrontierTask * p = pool_.alloc();
      p->ctxt_ = this;
      p->time_ = t;
      sch_->add_frontier_task(p);
    }
  };

  Event EventBuilder::construct_event() const {
    NormalEventContext * ctxt = new NormalEventContext(sch_);
    return Event{ctxt};
  }

  struct OrEventContext : EventContext {
    OrEventContext(Scheduler * sch)
      : EventContext(sch)
    {}
    virtual void notify(std::size_t t = 0) override {
    }
  };

  Event EventBuilder::construct_or_event(const list_type & l) const {
    OrEventContext * ctxt = new OrEventContext{sch_};
    return Event{ctxt};
  }

  struct AndEventContext : EventContext {
    AndEventContext(Scheduler * sch)
      : EventContext(sch)
    {}
    virtual void notify(std::size_t t = 0) override {
    }
  };

  Event EventBuilder::construct_and_event(const list_type & l) const {
    AndEventContext * ctxt = new AndEventContext{sch_};
    return Event{ctxt};
  }

  Event::Event(EventContext * ctxt) : ctxt_(ctxt) {}
  Event::Event() : ctxt_{nullptr} {}
  Event::~Event() { if (is_valid()) ctxt_->dec(); }

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

  bool Event::is_valid() const { return ctxt_ != nullptr; }
  void Event::notify(std::size_t t) { ctxt_->notify(t); }
  void Event::add_to_wait_set(Process * p) { ctxt_->add_to_wait_set(p); }

} // namespace ccm::kernel
