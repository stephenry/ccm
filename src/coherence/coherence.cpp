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

#include "coherence.hpp"
#include "msi.hpp"

namespace ccm {

const char * to_string(MessageType t) {
  switch (t) {
#define __declare_to_string(e)                  \
    case MessageType::e: return #e;
    MESSAGE_CLASSES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Unknown Message Type>";
  }
}

const char * to_string(ResponseType r) {
  switch (r) {
#define __declare_to_string(e)                  \
    case ResponseType::e: return #e;
    RESPONSE_CLASSES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Unknown Response Type>";
  }
}

const char * to_string(EvictionPolicy p) {
  switch (p) {
#define __declare_to_string(e)                  \
    case EvictionPolicy::e: return #e;
    EVICTION_POLICIES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Unknown Policy Type>";
  }
}

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts) {
  switch (opts.protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiCoherentAgentModel>(opts);
      break;
    case Protocol::MESI:
    case Protocol::MOSI:
    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}

std::unique_ptr<DirectoryModel> directory_factory(
    const DirectoryOptions & opts) {
  switch (opts.protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiDirectoryModel>(opts);
      break;
    case Protocol::MESI:
    case Protocol::MOSI:
    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}


} // namespace ccm

