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

namespace ccm::kernel {

bool operator==(EventHandle const & a, EventHandle const & b) {
  return (a.ed_ == b.ed_);
}

bool operator!=(EventHandle const & a, EventHandle const & b) {
  return !(a.ed_ == b.ed_);
}

bool EventHandle::is_valid() const {
  return ed_ != nullptr;
}

void EventHandle::notify_after(std::size_t t) {
  ed_->notify_after(*this, t);
};

void EventHandle::notify_on(std::size_t t) {
  ed_->notify_on(*this, t);
};

void EventHandle::add_to_wait_set(Process * p) {
  ed_->add_to_wait_set(p);
}

void EventHandle::remove_from_wait_set(Process * p) {
  ed_->remove_from_wait_set(p);
}

void EventHandle::wake_waiting_processes() {
  ed_->wake_waiting_processes();
}

void EventDescriptor::notify_after(EventHandle e, std::size_t t) {
  sch_->add_task_notify_after(e, t);
}

void EventDescriptor::notify_on(EventHandle e, std::size_t t) {
  sch_->add_task_notify_on(e, t);
}

void EventDescriptor::wake_waiting_processes() {
  for (Process * p : suspended_on_)
    if (p != nullptr)
      sch_->add_task_next_delta(p);
  suspended_on_.clear();
}

void EventDescriptor::add_to_wait_set(Process * p) {
  suspended_on_.push_back(p);
}

void EventDescriptor::remove_from_wait_set(Process * p) {
  // Convert pointer to nullptr for speed, skip nullptrs on notify.
  //
  Process * replace{nullptr};
  std::replace(suspended_on_.begin(), suspended_on_.end(), p, replace);
}

  //void EventOrDescriptor::notify(EventHandle h, std::size_t t) {
  // for (Process * p : suspended_on_) {
  //   if (p != nullptr)
  //     sch_->add_task_wake_after(p, t);

  //   for (EventHandle e : el_) {
  //     if (e != h)
  //       e.remove_from_wait_set(p);
  //   }
  // }
  //}

} // namespace ccm::kernel
