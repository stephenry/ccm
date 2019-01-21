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

AgentProtocol::AgentProtocol(const CoherentAgentOptions& opts)
    : ProtocolBase(opts) {}

std::unique_ptr<AgentProtocol> coherent_agent_factory(
    Protocol protocol, const CoherentAgentOptions& opts) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiAgentProtocol>(opts);
      break;
    case Protocol::MESI:
      return std::make_unique<MesiAgentProtocol>(opts);
      break;
    case Protocol::MOSI:
      return std::make_unique<MosiAgentProtocol>(opts);
      break;
    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}

ProtocolBase::ProtocolBase(const ActorOptions& opts) : opts_(opts) {}

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

SnoopFilterProtocol::SnoopFilterProtocol(const ActorOptions& opts)
    : ProtocolBase(opts) {}

std::unique_ptr<SnoopFilterProtocol> snoop_filter_factory(
    Protocol protocol, const ActorOptions& opts) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiSnoopFilterProtocol>(opts);
      break;

    case Protocol::MESI:
      return std::make_unique<MesiSnoopFilterProtocol>(opts);
      break;

    case Protocol::MOSI:
      return std::make_unique<MosiSnoopFilterProtocol>(opts);
      break;

    default:
      // TODO: Not implemented
      return nullptr;
      break;
  }
}

state_t DirectoryEntry::state() const { return state_; }
id_t DirectoryEntry::owner() const { return owner_.value(); }
const std::vector<id_t>& DirectoryEntry::sharers() const { return sharers_; }
std::size_t DirectoryEntry::num_sharers() const { return sharers_.size(); }

void DirectoryEntry::set_state(state_t state) { state_ = state; }
void DirectoryEntry::set_owner(id_t owner) { owner_ = owner; }
void DirectoryEntry::clear_owner() { owner_.reset(); }
void DirectoryEntry::add_sharer(id_t id) { sharers_.push_back(id); }
void DirectoryEntry::remove_sharer(id_t id) {
  sharers_.erase(std::find(sharers_.begin(), sharers_.end(), id),
                 sharers_.end());
}
void DirectoryEntry::clear_sharers() { sharers_.clear(); }
id_t DirectoryEntry::num_sharers_not_id(id_t id) const {
  return sharers_.size() - std::count(sharers_.begin(), sharers_.end(), id);
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
