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
#include "interconnect.hpp"
#include "msi.hpp"
#ifdef ENABLE_MESI
#  include "mesi.hpp"
#endif
#ifdef ENABLE_MOSI
#  include "mosi.hpp"
#endif

namespace ccm {

const char* Protocol::to_string(Protocol::type p) {
  switch (p) {
    case Protocol::MSI:
      return "msi";
      break;
#ifdef ENABLE_MESI
    case Protocol::MESI:
      return "mesi";
      break;
#endif
#ifdef ENABLE_MOSI
    case Protocol::MOSI:
      return "mosi";
      break;
#endif
    default:
      return "Unknown";
      break;
  }
}
#ifdef ENABLE_JSON
Protocol::type Protocol::from_json(nlohmann::json j) {
  Protocol::type p{Protocol::INVALID};
  if (j["protocol"] == "msi")
    p = Protocol::MSI;
  if (j["protocol"] == "mesi")
    p = Protocol::MESI;
  if (j["protocol"] == "mosi")
    p = Protocol::MOSI;
  return p;
}
#endif

command_t CoherentAgentCommand::from_string(const std::string & s) {
  static const std::map<std::string, command_t> str_to_cmd{
#define __declare_pair(__cmd) { #__cmd, __cmd },
    AGENT_COMMANDS(__declare_pair)
#undef __declare_pair
  };
  // TODO: Improve error checking
  command_t cmd;
  auto it = str_to_cmd.find(s);
  if (it != str_to_cmd.end())
    cmd = it->second;
  return cmd;
}

command_t SnoopFilterCommand::from_string(const std::string & s) {
  static const std::map<std::string, command_t> str_to_cmd{
#define __declare_pair(__cmd) { #__cmd, __cmd },
    SNOOP_FILTER_COMMANDS(__declare_pair)
#undef __declare_pair
  };
  // TODO: Improve error checking
  command_t cmd;
  auto it = str_to_cmd.find(s);
  if (it != str_to_cmd.end())
    cmd = it->second;
  return cmd;
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

CostModel::CostModel() {}

std::size_t CostModel::cost(command_t cmd) const {
  std::size_t cost{0};
  auto it = command_to_cost_.find(cmd);
  if (it != command_to_cost_.end())
    cost = it->second;
  return cost;
}
#ifdef ENABLE_JSON

AgentCostModel AgentCostModel::from_json(nlohmann::json & j) {
  AgentCostModel m;
  m.set_defaults();
  for (auto & [cmds, cost] : j.items()) {
    const command_t cmd{CoherentAgentCommand::from_string(cmds)};
    if (m.command_to_cost_.count(cmd))
      m.command_to_cost_[cmd] = cost;
  }
  return m;
}
#endif
#ifdef ENABLE_JSON

SnoopFilterCostModel SnoopFilterCostModel::from_json(nlohmann::json & j) {
  SnoopFilterCostModel m;
  m.set_defaults();
  for (auto & [cmds, cost] : j.items()) {
    const command_t cmd{SnoopFilterCommand::from_string(cmds)};
    if (m.command_to_cost_.count(cmd))
      m.command_to_cost_[cmd] = cost;
  }
  return m;
}
#endif

AgentCostModel::AgentCostModel() { set_defaults(); }

void AgentCostModel::set_defaults() {
#define INSERT_COST(cmd, cost)                                          \
  command_to_cost_.insert(std::make_pair(CoherentAgentCommand::cmd, cost))
#define INSERT_MESSAGE_COST(cmd, msg)                                   \
  command_to_cost_.insert(std::make_pair(                               \
      CoherentAgentCommand::cmd, MessageType::to_default_cost(MessageType::msg)))

  INSERT_COST(UpdateState, 0);
  INSERT_COST(IncAckCount, 0);
  INSERT_COST(SetAckExpectCount, 0);
  INSERT_MESSAGE_COST(EmitGetS, GetS);
  INSERT_MESSAGE_COST(EmitGetM, GetM);
  INSERT_MESSAGE_COST(EmitPutS, PutS);
  INSERT_MESSAGE_COST(EmitPutM, PutM);
  INSERT_MESSAGE_COST(EmitPutE, PutE);
  INSERT_MESSAGE_COST(EmitPutO, PutO);
  INSERT_MESSAGE_COST(EmitDataToReq, Data);
  INSERT_MESSAGE_COST(EmitDataToDir, Data);
  INSERT_MESSAGE_COST(EmitInvAck, Inv);
#undef INSERT_COST
#undef INSERT_MESSAGE_COST
}

SnoopFilterCostModel::SnoopFilterCostModel() { set_defaults(); }

void SnoopFilterCostModel::set_defaults() {
#define INSERT_COST(cmd, cost)                                          \
  command_to_cost_.insert(std::make_pair(SnoopFilterCommand::cmd, cost))
#define INSERT_MESSAGE_COST(cmd, msg)                                   \
  command_to_cost_.insert(std::make_pair(                               \
      SnoopFilterCommand::cmd, MessageType::to_default_cost(MessageType::msg)))

  INSERT_COST(UpdateState, 0);
  INSERT_COST(SetOwnerToReq, 0);
  INSERT_COST(SendDataToReq, 0);
  INSERT_COST(SendInvToSharers, 0);
  INSERT_COST(ClearSharers, 0);
  INSERT_COST(AddReqToSharers, 0);
  INSERT_COST(DelReqFromSharers, 0);
  INSERT_COST(DelOwner, 0);
  INSERT_COST(AddOwnerToSharers, 0);
  INSERT_MESSAGE_COST(CpyDataToMemory, Data);
  INSERT_MESSAGE_COST(SendPutSAckToReq, PutSAck);
  INSERT_MESSAGE_COST(SendPutMAckToReq, PutMAck);
  INSERT_MESSAGE_COST(SendPutEAckToReq, PutEAck);
  INSERT_MESSAGE_COST(SendPutOAckToReq, PutOAck);
  INSERT_MESSAGE_COST(SendAckCountToReq, AckCount);
  INSERT_MESSAGE_COST(SendFwdGetMToOwner, FwdGetM);
  INSERT_MESSAGE_COST(SendFwdGetSToOwner, FwdGetS);
#undef INSERT_COST
#undef INSERT_MESSAGE_COST
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
    Protocol::type protocol, const Platform & platform) {
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

id_t DirectoryLine::owner() const { return owner_.value(); }
const std::vector<id_t>& DirectoryLine::sharers() const { return sharers_; }
std::size_t DirectoryLine::num_sharers() const { return sharers_.size(); }

void DirectoryLine::set_owner(id_t owner) { owner_ = owner; }
void DirectoryLine::clear_owner() { owner_.reset(); }
void DirectoryLine::add_sharer(id_t id) { sharers_.push_back(id); }
void DirectoryLine::remove_sharer(id_t id) {
  sharers_.erase(std::find(sharers_.begin(), sharers_.end(), id),
                 sharers_.end());
}
void DirectoryLine::clear_sharers() { sharers_.clear(); }
id_t DirectoryLine::num_sharers_not_id(id_t id) const {
  return sharers_.size() - std::count(sharers_.begin(), sharers_.end(), id);
}

std::unique_ptr<SnoopFilterProtocol> snoop_filter_protocol_factory(
    Protocol::type protocol) {
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
  void set_id(id_t id) override { id_ = id; }
  void add_line(addr_t addr, const CacheLine& cache_line) override {
    validator_->add_cache_line(id_, addr, cache_line);
  }
  void add_line(addr_t addr, const DirectoryLine& directory_entry) override {
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
    id_t id, addr_t addr, const DirectoryLine& directory_entry) {
  directory_lines_[addr] = directory_entry;
}

std::unique_ptr<CoherenceProtocolValidator>
coherence_protocol_validator_factory(Protocol::type protocol) {
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
    default:
      return nullptr;
      break;
  }
}

}  // namespace ccm
