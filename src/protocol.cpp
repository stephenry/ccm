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
#include "msi.hpp"
#ifdef ENABLE_MESI
#  include "mesi.hpp"
#endif
#ifdef ENABLE_MOSI
#  include "mosi.hpp"
#endif

namespace ccm {

const char* to_string(Protocol p) {
  switch (p) {
    case Protocol::MSI:
      return "MSI";
      break;
#ifdef ENABLE_MESI
    case Protocol::MESI:
      return "MESI";
      break;
#endif
#ifdef ENABLE_MOSI
    case Protocol::MOSI:
      return "MOSI";
      break;
#endif
    default:
      return "Unknown";
      break;
  }
}

const char* CoherentAgentCommand::to_string(command_t command) {
    switch (command) {
      // clang-format off
#define __declare_to_string(__e)                \
      case CoherentAgentCommand::__e:           \
        return #__e;
      AGENT_COMMANDS(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "<Invalid Command>";
  }
}

std::size_t CoherentAgentCommand::to_cost(command_t command) {
  std::size_t cost{0};
  switch (command) {
    case CoherentAgentCommand::UpdateState:
      cost += 1;
      break;
    case CoherentAgentCommand::IncAckCount:
      cost += 0;
      break;
    case CoherentAgentCommand::SetAckExpectCount:
      cost += 0;
      break;
    case CoherentAgentCommand::EmitGetS:
      cost += MessageType::to_cost(MessageType::GetS);
      break;
    case CoherentAgentCommand::EmitGetM:
      cost += MessageType::to_cost(MessageType::GetM);
      break;
    case CoherentAgentCommand::EmitPutS:
      cost += MessageType::to_cost(MessageType::PutS);
      break;
    case CoherentAgentCommand::EmitDataToReq:
      cost += MessageType::to_cost(MessageType::Data);
      break;
    case CoherentAgentCommand::EmitDataToDir:
      cost += MessageType::to_cost(MessageType::Data);
      break;
    case CoherentAgentCommand::EmitInvAck:
      cost += MessageType::to_cost(MessageType::InvAck);
      break;
  }
  return cost;
};

std::size_t CoherenceActions::compute_cost(const CoherenceActions & actions) {
  std::size_t cost{0};
  for (command_t cmd : actions.commands())
    cost += CoherentAgentCommand::to_cost(cmd);
  return cost;
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

const char* SnoopFilterCommand::to_string(command_t command) {
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
}

void CacheLine::set_invalid() {
#define __invalidate(__name, __type, __init)    \
  __name ## _ = __init;
  CACHE_LINE_FIELDS(__invalidate)
#undef __invalidate
}

bool CacheLine::is_last_inv_ack() const {
  if (!inv_ack_expect_valid())
    return false;

  return (inv_ack_count() == (inv_ack_expect() - 1));
}

AgentProtocol::AgentProtocol() {}

std::unique_ptr<AgentProtocol> agent_protocol_factory(
    Protocol protocol, const Platform & platform) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiAgentProtocol>(platform);
      break;
#ifdef ENABLE_MESI
    case Protocol::MESI:
      return std::make_unique<MesiAgentProtocol>(platform);
      break;
#endif
#ifdef ENABLE_MOSI
    case Protocol::MOSI:
      return std::make_unique<MosiAgentProtocol>(platform);
      break;
#endif
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

std::unique_ptr<SnoopFilterProtocol> snoop_filter_protocol_factory(
    Protocol protocol) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiSnoopFilterProtocol>();
      break;
#ifdef ENABLE_MESI
    case Protocol::MESI:
      return std::make_unique<MesiSnoopFilterProtocol>();
      break;
#endif
#ifdef ENABLE_MOSI
    case Protocol::MOSI:
      return std::make_unique<MosiSnoopFilterProtocol>();
      break;
#endif
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
  void set_id(id_t id) /*override*/ { id_ = id; }
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

std::unique_ptr<CoherenceProtocolValidator>
coherence_protocol_validator_factory(Protocol protocol) {
  switch (protocol) {
    case Protocol::MSI:
      return std::make_unique<MsiCoherenceProtocolValidator>();
      break;
#ifdef ENABLE_MESI
    case Protocol::MESI:
      return std::make_unique<MesiCoherenceProtocolValidator>();
      break;
#endif
#ifdef ENABLE_MOSI
    case Protocol::MOSI:
      return std::make_unique<MosiCoherenceProtocolValidator>();
      break;
#endif
  }
}

}  // namespace ccm
