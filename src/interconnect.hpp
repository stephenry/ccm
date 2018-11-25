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

#ifndef __INTERCONNECT_HPP__
#define __INTERCONNECT_HPP__

#include "src/kernel.hpp"
#include "src/transaction.hpp"

namespace ccm {

class Interconnect : public ccm::Module {

 public:

  //
  virtual void push(std::size_t id, TransactionPtr * t) = 0;

  //
  virtual void register_port(std::size_t id, EventHandle e) = 0;

  //
  virtual bool port_valid(std::size_t id) const = 0;

  //
  virtual TransactionPtr get_port(std::size_t id) = 0;
  
};

using InterconnectPtr = std::unique_ptr<Interconnect>;

enum InterconnectType : int {
  FixedLatency
};

struct InterconnectOptions {
  InterconnectType type;
};

class InterconnectFactory {

  //
  static InterconnectPtr construct(InterconnectOptions const & opts);

};

} // namespace ccm

#endif
