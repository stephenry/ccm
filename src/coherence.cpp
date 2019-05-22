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
#include "actor.hpp"
#include "cache.hpp"
#include "common.hpp"
#include "msi.hpp"
//#include "mesi.hpp"
//#include "mosi.hpp"
#include "platform.hpp"
#include "sim.hpp"
#include "snoopfilter.hpp"
#include "protocol.hpp"

namespace ccm {

const char* to_string(Protocol p) {
  switch (p) {
    case Protocol::MSI:
      return "MSI";
      break;
    case Protocol::MESI:
      return "MESI";
      break;
    case Protocol::MOSI:
      return "MOSI";
      break;
    default:
      return "Unknown";
      break;
  }
}

// clang-format off
const char* to_string(TransactionResult r) {
  switch (r) {
#define __declare_to_string(__state) \
  case TransactionResult::__state:   \
    return #__state;                 \
    break;
    TRANSACTION_RESULT(__declare_to_string)
#undef __declare_to_string
  default:
    return "<Invalid>";
  }
}
// clang-format on

void CacheLine::set_invalid() {
#define __declare_invalid(__name, __type, __default) __name##_ = __default;
  CACHE_LINE_FIELDS(__declare_invalid)
#undef __declare_invalid
}

std::unique_ptr<AgentProtocol> coherent_agent_factory(
    Protocol protocol, const CoherentAgentOptions& opts) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiAgentProtocol>(opts);
      break;
    case Protocol::MESI:
      //      return std::make_unique<MesiAgentProtocol>(opts);
      return nullptr;
      break;
    case Protocol::MOSI:
      //      return std::make_unique<MosiAgentProtocol>(opts);
      return nullptr;
      break;
    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}

const char* to_string(EvictionPolicy p) {
  switch (p) {
#define __declare_to_string(e) \
  case EvictionPolicy::e:      \
    return #e;
    EVICTION_POLICIES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Unknown Policy Type>";
  }
}

std::unique_ptr<SnoopFilterProtocol> snoop_filter_factory(
    Protocol::type protocol, const ActorOptions& opts) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiSnoopFilterProtocol>();
      break;

    case Protocol::MESI:
      //      return std::make_unique<MesiSnoopFilterProtocol>();
      return nullptr;
      break;

    case Protocol::MOSI:
      //      return std::make_unique<MosiSnoopFilterProtocol>();
      return nullptr;
      break;

    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}

std::unique_ptr<CoherenceProtocolValidator> validator_factory(
    Protocol::type protocol) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiCoherenceProtocolValidator>();
      break;
    case Protocol::MESI:
      //      return std::make_unique<MesiCoherenceProtocolValidator>();
      return nullptr;
      break;
    case Protocol::MOSI:
      //      return std::make_unique<MosiCoherenceProtocolValidator>();
      return nullptr;
      break;
  }
}

}  // namespace ccm
