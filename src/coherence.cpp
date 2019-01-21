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
#include "mesi.hpp"
#include "mosi.hpp"
#include "msi.hpp"
#include "platform.hpp"
#include "sim.hpp"
#include "snoopfilter.hpp"

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
#define __declare_to_string(__e)  \
  case CoherentAgentCommand::__e:             \
    return #__e;
    AGENT_COMMANDS(__declare_to_string)
#undef __declare_to_string
  default:
    return "<Invalid Line State>";
  }
}
// clang-format on

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

CoherentAgentModel::CoherentAgentModel(const CoherentAgentOptions& opts)
    : CoherentActorBase(opts) {}

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    Protocol protocol, const CoherentAgentOptions& opts) {
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

const char* to_string(SnoopFilterCommand command) {
  switch (command) {
#define __declare_to_string(__e) \
  case SnoopFilterCommand::__e:  \
    return #__e;
    SNOOP_FILTER_COMMANDS(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
}

CoherentActorBase::CoherentActorBase(const ActorOptions& opts) : opts_(opts) {}

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

SnoopFilterModel::SnoopFilterModel(const ActorOptions& opts)
    : CoherentActorBase(opts), opts_(opts) {}

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
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

CoherenceProtocolValidator::CoherenceProtocolValidator() {}

struct CoherenceProtocolValidator::ProtocolValidatorVisitor : CacheVisitor {
  ProtocolValidatorVisitor(CoherenceProtocolValidator* validator)
      : validator_(validator) {}
  void set_id(id_t id) override { id_ = id; }
  void add_line(addr_t addr, const CacheLine& cache_line) override {
    validator_->add_cache_line(id_, addr, cache_line);
  }
  void add_line(addr_t addr, const DirectoryEntry& directory_entry) override {
    validator_->add_dir_line(id_, addr, directory_entry);
  }

 private:
  id_t id_;
  CoherenceProtocolValidator* validator_;
};

std::unique_ptr<CacheVisitor> CoherenceProtocolValidator::get_cache_visitor() {
  return std::make_unique<ProtocolValidatorVisitor>(this);
}

bool CoherenceProtocolValidator::validate() const {
  for (auto& l : directory_lines_) {
    auto lines = cache_lines_.find(l.first);
    if (lines == cache_lines_.end()) return false;

    if (!validate_addr(l.first, lines->second, l.second)) return false;
  }
  return true;
}

void CoherenceProtocolValidator::add_cache_line(id_t id, addr_t addr,
                                                const CacheLine& cache_line) {
  cache_lines_[addr].push_back(std::make_tuple(id, cache_line));
}

void CoherenceProtocolValidator::add_dir_line(
    id_t id, addr_t addr, const DirectoryEntry& directory_entry) {
  directory_lines_[addr] = directory_entry;
}

std::unique_ptr<CoherenceProtocolValidator> validator_factory(
    Protocol protocol) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiCoherenceProtocolValidator>();
      break;
    case Protocol::MESI:
      return std::make_unique<MesiCoherenceProtocolValidator>();
      break;
    case Protocol::MOSI:
      return std::make_unique<MosiCoherenceProtocolValidator>();
      break;
  }
}

}  // namespace ccm
