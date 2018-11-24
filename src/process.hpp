#ifndef __PROCESS_HPP__
#define __PROCESS_HPP__
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

#include "src/scheduler.hpp"

namespace ccm {

class Scheduler;

class InvokeReq {
  friend class Scheduler;

  InvokeReq(Scheduler * sch) : sch_(sch) {}
 public:

  //
  SimState state() const;
  std::size_t now() const;
 private:
  Scheduler const * sch_;
};

enum ResponseType { WakeOn, WakeAfter, NotifyAfter, Terminate };

class InvokeRsp {
 public:
  InvokeRsp() : type_(ResponseType::Terminate) {}
  void wake_on(Event & e) {
    type_ = ResponseType::WakeOn;
    e_ = e;
  }
  void wake_after(std::size_t t) {
    type_ = ResponseType::WakeAfter;
    t_ = t;
  }
  void terminate() {
    type_ = ResponseType::Terminate;
  }
  void notify_after(Event & e, std::size_t t = 0) {
    type_ = ResponseType::NotifyAfter;
    t_ = t;
  }
  ResponseType type() const { return type_; }
  std::size_t time() const { return t_; }
  Event event() const { return e_; }
 private:
  ResponseType type_;
  Event e_;
  std::size_t t_;
};

class Process {
 public:
  virtual InvokeRsp invoke (InvokeReq const & req) {
    return InvokeRsp();
  };
  virtual ~Process() {}
};

} // namespace ccm

#endif
