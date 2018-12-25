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

#ifndef __TRANSACTION_HPP__
#define __TRANSACTION_HPP__

#include "common.hpp"
#include <memory>

namespace ccm::kernel {

struct Transaction : public Poolable {
  friend class TransactionBuilder;
  
  std::size_t mid() const { return mid_; }
  std::size_t sid() const { return sid_; }

 private:
  void set_mid(std::size_t mid) { mid_ = mid; }
  void set_sid(std::size_t sid) { sid_ = sid; }

  std::size_t mid_; // Master ID
  std::size_t sid_; // Slave ID
};

class TransactionBuilder {
 public:
  TransactionBuilder (Transaction * msg)
      : msg_(msg)
  {}
  void set_mid(std::size_t mid) { msg_->set_mid(mid); }
  void set_sid(std::size_t sid) { msg_->set_sid(sid); }
 private:
  Transaction * msg_{nullptr};
};

using TMailBox = MailBox<Transaction *>;
using TMailBoxIf = MailBoxIf<Transaction *>;
  
using TEventQueue = EventQueue<Transaction *>;

} // namespace ccm::kernel

#endif
