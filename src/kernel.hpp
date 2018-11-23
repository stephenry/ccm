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

#include <vector>

namespace ccm {

  class Event {
  public:
  };

  enum SimState : int {
    Initialization,
    Running
  };
  
  class InvokeReq {
  public:
    SimState state() const { return Initialization; }
    std::size_t now() const { return 0; }
  };

  class InvokeRsp {
  public:
    void wake_on(Event & e) {}
    void wake_after(std::size_t t) {}
    void terminate() {}
    
    void notify_after(Event & e, std::size_t t = 0) {}
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
    Event create_event() {}
    void run() {}
    void add_process (Process * p) {}
    void remove_process (Process * p) {}
  };
  
} // namespace ccm


#endif
