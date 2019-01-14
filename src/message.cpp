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

#include "message.hpp"
#include <sstream>

namespace ccm {

const char * MessageType::to_string(base_type e) {
  switch (e) {
#define __to_str(__name, __cost)                \
    case __name: return #__name; break;
    MESSAGE_TYPES(__to_str)
#undef __to_str
    default: return "Unknown";
  }
}

int MessageType::to_cost(base_type b) {
  switch (b) {
#define __to_str(__name, __cost)                \
    case __name: return __cost; break;
    MESSAGE_TYPES(__to_str)
#undef __to_str
    default: return 0;
  }
}

MessageClass Message::cls() const {
  if (is_ack())
    return MessageClass::Response;

  MessageClass cls;
  switch (type()) {
  case MessageType::Data:
    cls = MessageClass::Data;
    break;
  default:
    cls = MessageClass::Request;
    break;
  }
  return cls;
}

void Message::set_invalid() {
#define __declare_invalid(__name, __type, __default)    \
  __name ## _ = __default;
  MESSAGE_FIELDS(__declare_invalid)
#undef __declare_invalid
}
  
std::string to_string(const Message & m) {
  using namespace std;
  
  StructRenderer sr;
  sr.add("type", MessageType::to_string(m.type()));
  sr.add("src_id", to_string(m.src_id()));
  sr.add("dst_id", to_string(m.dst_id()));
  sr.add("transaction", to_string(*m.transaction()));
  sr.add("is_ack", to_string(m.is_ack()));
  switch (m.type()) {
    case MessageType::Data:
      sr.add("ack_count", to_string(m.ack_count()));
      sr.add("is_exclusive", to_string(m.is_exclusive()));
      break;
    case MessageType::FwdGetS:
    case MessageType::FwdGetM:
      sr.add("fwd_id", to_string(m.fwd_id()));
      break;
    default:
      break;
  }
  return sr.str();
}

MessageDirector::MessageDirector(const ActorOptions & opts)
    : opts_(opts) {
  src_id_ = opts.id();
}

MessageBuilder MessageDirector::builder() {
  return MessageBuilder{pool_.alloc(), src_id_};
}

} // namespace ccm

