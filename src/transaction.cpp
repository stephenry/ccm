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

#include "transaction.hpp"
#include "utility.hpp"
#include <algorithm>

namespace ccm {

const char * to_string(TransactionType t) {
  switch (t) {
    case TransactionType::Load: return "Load"; break;
    case TransactionType::Store: return "Store"; break;
    default: return "Unknown"; break;
  }
}

std::string to_string(const Transaction & t) {
  using namespace std;

  StructRenderer sr;
  sr.add("type", to_string(t.type()));
  sr.add("addr", std::to_string(t.addr()));
  sr.add("tid", std::to_string(t.tid()));
  return sr.str();
}

bool NullTransactionSource::get_transaction(TimeStamped<Transaction *> & ts) {
  return false;
}
  
void ProgrammaticTransactionSource::add_transaction(
        TransactionType type, Time time, uint64_t addr) {
  Transaction * t = pool_.alloc();
  t->set_type(type);
  t->set_addr(addr);
  pending_.push_back(TimeStamped<Transaction *>{time, t});
}

bool ProgrammaticTransactionSource::get_transaction(TimeStamped<Transaction *> & ts) {
  if (pending_.size() == 0)
    return false;

  ts = pending_.front();
  pending_.pop_front();
  
  Transaction * t = ts.t();
  t->set_tid(in_flight_.size());
  in_flight_.push_back(t);

  return true;
}

void ProgrammaticTransactionSource::event_start(TimeStamped<Transaction *> ts) {
  const Transaction * t = ts.t();
  log_info("Transaction starts: ", t->tid());
}

void ProgrammaticTransactionSource::event_finish(TimeStamped<Transaction *> ts) {
  const Transaction * t = ts.t();
  log_info("Transaction ends: ", t->tid());
  
  auto it = std::find(in_flight_.begin(), in_flight_.end(), t);
  if (it != in_flight_.end()) {
    (*it)->release();
    in_flight_.erase(it);
  }
}

} // namespace ccm
