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

#ifndef __PRIMITIVES_HPP__
#define __PRIMITIVES_HPP__

#include "module.hpp"
#include "event.hpp"

#include <vector>
#include <deque>

namespace ccm::kernel {

  template<typename T>
  class MailBoxIf {
  public:
    virtual bool can_push() const = 0;
    virtual void push(const T & t) = 0;
    virtual Event event() = 0;
    virtual ~MailBoxIf() {}
  };

  template<typename T>
  class MailBox : public Module, public MailBoxIf<T> {
  public:
    MailBox(const Context & ctxt, std::size_t n = 1)
      : Module(ctxt), n_(n) {
      const EventBuilder b = ctxt.event_builder();
      e_ = b.construct_event();
    }
    void push(const T & t) override {
      ts_.push_back(t);
      e_.notify();
    }
    bool can_push() const override {
      return (ts_.size() < n_);
    }
    Event event() override { return e_; }
    bool get(T & t) {
      if (!has_mail())
        return false;
    
      t = ts_.back();
      ts_.pop_back();
      return true;
    }
    bool has_mail() const { return !ts_.empty(); }
  private:
    std::size_t n_;
    std::vector<T> ts_;
    Event e_;
  };
  using TMailBox = MailBox<Transaction *>;
  using TMailBoxIf = MailBoxIf<Transaction *>;

  template<typename MSG>
  class EventQueue : public Module {
    struct QueueEntry {
      std::size_t t;
      MSG msg;
    };
  public:
    EventQueue(const Context & ctxt) : Module(ctxt) {
      EventBuilder b = ctxt.event_builder();
      e_ = b.construct_event();
    }
    Event event() { return e_; }
    void set (MSG const & msg, std::size_t t = 0) {
      if (v_.size() == 0)
        e_.notify(t);
      v_.push_back(QueueEntry{t, msg});
    }
    bool has_msg(std::size_t t) const {
      if (!has_events())
        return false;
    
      const QueueEntry & qe = v_.front();
      return (qe.t <= t);
    }
    bool get(MSG & msg, std::size_t t) {
      const bool valid = has_msg(t);
      if (valid) {
        const QueueEntry & qe = v_.front();
        msg = qe.msg;
        v_.pop_front();

        if (has_events()) {
          const QueueEntry & qe = v_.front();
          e_.notify(qe.t);
        }
      }
      return valid;
    }
  private:
    bool has_events() const {
      return (v_.size() != 0);
    }
    std::deque<QueueEntry> v_;
    Event e_;
  };

} // namespace ccm::kernel

#endif
