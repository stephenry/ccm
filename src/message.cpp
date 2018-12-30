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

const char * to_string(MessageType t) {
  switch (t) {
#define __to_str(__e)                        \
    case MessageType::__e: return #__e; break;
    MESSAGE_CLASSES(__to_str)
#undef __to_str
    default: return "Unknown";
  }
}

std::string to_string(const Message & m) {
  using namespace std;
  
  StructRenderer sr;
  sr.add("type", to_string(m.type()));
  sr.add("src_id", std::to_string(m.src_id()));
  sr.add("dst_id", std::to_string(m.dst_id()));
  sr.add("tid", std::to_string(m.tid()));
  sr.add("is_ack", to_string(m.is_ack()));

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

