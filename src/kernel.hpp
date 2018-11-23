#ifndef __KERNEL_HPP__
#define __KERNEL_HPP__

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

namespace ccm {

  class Scheduler;
  class EventDescriptor;
  class SchedulerImpl;

  enum SimState : int {
    Elaboration,
    Initialization,
    Running,
    Termination
  };

  enum Wake { WEvent, WTime, WTerminate };
  
  class Event {
    friend class Scheduler;
    friend class SchedulerImpl;
    Event(EventDescriptor * ed) : ed_(ed) {}
  public:
    Event() : ed_{nullptr} {}
    bool is_valid() const { return ed_ != nullptr; }
  private:
    EventDescriptor *ed_{nullptr};
  };
  
  class InvokeReq {
    friend class SchedulerImpl;

    InvokeReq(SchedulerImpl * sch) : sch_(sch) {}
  public:
    SimState state() const;
    std::size_t now() const;
  private:
    SchedulerImpl const * sch_;
  };

  class InvokeRsp {
    friend SchedulerImpl;
  public:
    InvokeRsp() : wake_(Wake::WTerminate) {}
    void wake_on(Event & e) {
      wake_ = Wake::WEvent;
      e_ = e;
    }
    void wake_after(std::size_t t) {
      wake_ = Wake::WTime;
      t_ = t;
    }
    void terminate() { wake_ = Wake::WTerminate; }
    void notify_after(Event & e, std::size_t t = 0) {
      wake_ = Wake::WTime;
      t_ = t;
    }
  private:
    Wake wake_;
    Event e_;
    std::size_t t_;
  };

  class Process {
  public:
    virtual InvokeRsp invoke (InvokeReq const & req) {};
    virtual ~Process() {}
  };

  class Module {
  public:
  };

  class Scheduler {
  public:
    Scheduler();
    ~Scheduler();
    
    //
    void run();

    //
    Event create_event();
    
    //
    void add_process (Process * p);
    void remove_process (Process * p);

    //
    SimState state() const;
    std::size_t now() const;
  private:
    std::unique_ptr<SchedulerImpl> impl_;
  };
  
} // namespace ccm


#endif
