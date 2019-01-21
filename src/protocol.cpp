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

#include "protocol.hpp"
#include "mesi.hpp"
#include "mosi.hpp"
#include "msi.hpp"

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
  const char* to_string(CoherentAgentCommand command) {
    switch (command) {
      // clang-format off
#define __declare_to_string(__e)                \
      case CoherentAgentCommand::__e:           \
        return #__e;
      AGENT_COMMANDS(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "<Invalid Line State>";
  }
}

const char* to_string(TransactionResult r) {
  switch (r) {
    // clang-format off
#define __declare_to_string(__state)            \
      case TransactionResult::__state:          \
        return #__state;                        \
        break;
      TRANSACTION_RESULT(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "<Invalid>";
  }
}

const char* to_string(SnoopFilterCommand command) {
  switch (command) {
    // clang-format off
#define __declare_to_string(__e)                \
      case SnoopFilterCommand::__e:             \
        return #__e;
      SNOOP_FILTER_COMMANDS(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "<Invalid Line State>";
  }

  AgentProtocol::AgentProtocol(const ActorOptions& opts) : opts_(opts) {}

  std::unique_ptr<AgentProtocol> agent_protocol_factory(
      Protocol protocol, const ActorOptions& opts) {
    switch (protocol) {
      case Protocol::MSI:
        return std::make_unique<MsiCoherentAgentModel>(opts);
        break;

      case Protocol::MESI:
        return std::make_unique<MesiCoherentAgentModel>(opts);
        break;

      case Protocol::MOSI:
        return std::make_unique<MosiCoherentAgentModel>(opts);
        break;

      default:
        // TODO: Not implemented
        return nullptr;
        break;
    }
  }

  SnoopFilterModel::SnoopFilterModel(const SnoopFilterOptions& opts)
      : AgentProtocol(opts), opts_(opts) {}

  std::unique_ptr<SnoopFilterModel> snoop_filter_protocol_factory(
      Protocol protocol, const ActorOptions& opts) {
    switch (protocol) {
      case Protocol::MSI:
        return std::make_unique<MsiSnoopFilterModel>(opts);
        break;

      case Protocol::MESI:
        return std::make_unique<MesiSnoopFilterModel>(opts);
        break;

      case Protocol::MOSI:
        return std::make_unique<MosiSnoopFilterModel>(opts);
        break;

      default:
        // TODO: Not implemented
        return nullptr;
        break;
    }
  }

  ProtocolValidator::ProtocolValidator() {}

  CacheVisitor ProtocolValidator::get_cache_visitor() {
    return CacheVisitor{this};
  }

  bool ProtocolValidator::validate() const {
    for (auto& l : directory_lines_) {
      auto lines = cache_lines_.find(l.first);
      if (lines == cache_lines_.end()) return false;

      if (!validate_addr(l.first, lines->second, l.second)) return false;
    }
    return true;
  }

  void ProtocolValidator::add_cache_line(id_t id, addr_t addr,
                                         const CacheLine& cache_line) {
    cache_lines_[addr].push_back(std::make_tuple(id, cache_line));
  }

  void ProtocolValidator::add_dir_line(addr_t addr,
                                       const DirectoryEntry& directory_entry) {
    directory_lines_[addr] = directory_entry;
  }

  std::unique_ptr<ProtocolValidator> validator_factory(Protocol protocol) {
    switch (protocol) {
      case Protocol::MSI:
        return std::make_unique<MsiProtocolValidator>();
        break;
      case Protocol::MESI:
        return std::make_unique<MesiProtocolValidator>();
        break;
      case Protocol::MOSI:
        return std::make_unique<MosiProtocolValidator>();
        break;
    }
  }

}  // namespace ccm
