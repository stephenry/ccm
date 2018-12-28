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
#include "common.hpp"
#include "actors.hpp"
#include "sim.hpp"
#include "msi.hpp"

namespace ccm {

const char * to_string(CoherentAgentCommand command) {
  switch (command) {
#define __declare_to_string(__e)                        \
    case CoherentAgentCommand::__e: return #__e;
    AGENT_COMMANDS(__declare_to_string)
#undef __declare_to_string
    default: return "<Invalid Line State>";
  }
}

CoherentAgentCommandInvoker::CoherentAgentCommandInvoker(
    const AgentOptions & opts) : msgd_(opts) {
  id_ = opts.id();
}

void CoherentAgentCommandInvoker::invoke(
    Frontier & f, const CoherentActorActions & actions) {

  for (const uint8_t cmd : actions.actions()) {

    switch (static_cast<CoherentAgentCommand>(cmd)) {
      
      case CoherentAgentCommand::UpdateState:
        invoke_update_state(f);
        break;

      case CoherentAgentCommand::EmitGetS:
        invoke_emit_gets(f);
        break;
      
      case CoherentAgentCommand::EmitGetM:
        invoke_emit_getm(f);
        break;
      
      case CoherentAgentCommand::EmitDataToReq:
        invoke_emit_data_to_req(f);
        break;
      
      case CoherentAgentCommand::EmitDataToDir:
        invoke_emit_data_to_dir(f);
        break;
      
      case CoherentAgentCommand::EmitInvAck:
        invoke_emit_inv_ack(f);
        break;
    }
  }
}

void CoherentAgentCommandInvoker::invoke_update_state(Frontier & f) {
}

void CoherentAgentCommandInvoker::invoke_emit_gets(Frontier & f) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::GetS);
  b.set_dst_id(4);
  b.set_tid(1);
  f.add_to_frontier(1 + time(), b.msg());
}

void CoherentAgentCommandInvoker::invoke_emit_getm(Frontier & f) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::GetM);
  b.set_dst_id(4);
  b.set_tid(1);
  f.add_to_frontier(1 + time(), b.msg());
}

void CoherentAgentCommandInvoker::invoke_emit_data_to_req(Frontier & f) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
}

void CoherentAgentCommandInvoker::invoke_emit_data_to_dir(Frontier & f) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
}

void CoherentAgentCommandInvoker::invoke_emit_inv_ack(Frontier & f) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Inv);
  b.set_is_ack(true);
}


const char * to_string(SnoopFilterCommand command) {
  switch (command) {
#define __declare_to_string(__e)                \
    case SnoopFilterCommand::__e: return #__e;
    SNOOP_FILTER_COMMANDS(__declare_to_string)
#undef __declare_to_string
    default: return "<Invalid Line State>";
  }
}

SnoopFilterCommandInvoker::SnoopFilterCommandInvoker(
    const SnoopFilterOptions & opts) : msgd_(opts) {
  id_ = opts.id();
}

void SnoopFilterCommandInvoker::invoke(
    const CoherentActorActions & actions,
    const Message * msg, Frontier & f, DirectoryEntry & d) {

  for (const uint8_t cmd : actions.actions()) {

    switch (static_cast<SnoopFilterCommand>(cmd)) {
      case SnoopFilterCommand::UpdateState:
        handle_update_state(msg, actions, f, d);
        break;

      case SnoopFilterCommand::SetOwnerToReq:
        handle_set_owner_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendDataToReq:
        handle_send_data_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendInvToSharers:
        handle_send_inv_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::ClearSharers:
        handle_clear_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::AddReqToSharers:
        handle_add_req_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::DelReqFromSharers:
        handle_del_req_from_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::DelOwner:
        handle_del_owner(msg, f, d);
        break;

      case SnoopFilterCommand::AddOwnerToSharers:
        handle_add_owner_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::CpyDataToMemory:
        handle_cpy_data_to_memory(msg, f, d);
        break;

      case SnoopFilterCommand::SendPutSAckToReq:
        handle_send_put_sack_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendPutMAckToReq:
        handle_send_put_mack_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendFwdGetSToOwner:
        handle_send_fwd_gets_to_owner(msg, f, d);
        break;
    }
  }
}

void SnoopFilterCommandInvoker::handle_update_state(
    const Message * msg, const CoherentActorActions & a,
    Frontier & f, DirectoryEntry & d) {
  CCM_ASSERT(a.has_state_update());
  d.set_state(a.next_state());
}

void SnoopFilterCommandInvoker::handle_set_owner_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.set_owner(msg->src_id());
}

void SnoopFilterCommandInvoker::handle_send_data_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_dst_id(msg->src_id());
  b.set_tid(msg->tid());
  f.add_to_frontier(1 + time(), b.msg());
}

void SnoopFilterCommandInvoker::handle_send_inv_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  std::size_t time_start = 1 + time();
  MessageBuilder b = msgd_.builder();
  for (std::size_t sharer : d.sharers()) {
    b.set_type(MessageType::Inv);
    b.set_dst_id(sharer);
    b.set_tid(msg->tid());
    f.add_to_frontier(time_start++, b.msg());
  }
}

void SnoopFilterCommandInvoker::handle_clear_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.clear_sharers();
}

void SnoopFilterCommandInvoker::handle_add_req_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.add_sharer(msg->src_id());
}

void SnoopFilterCommandInvoker::handle_del_req_from_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.remove_sharer(msg->src_id());
}

void SnoopFilterCommandInvoker::handle_del_owner(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.clear_owner();
}

void SnoopFilterCommandInvoker::handle_add_owner_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  d.add_sharer(d.owner());
}

void SnoopFilterCommandInvoker::handle_cpy_data_to_memory(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  // NOP
}

void SnoopFilterCommandInvoker::handle_send_put_sack_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutS);
  b.set_dst_id(msg->src_id());
  b.set_tid(msg->tid());
  b.set_is_ack(true);
  f.add_to_frontier(1 + time(), b.msg());
}

void SnoopFilterCommandInvoker::handle_send_put_mack_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutM);
  b.set_dst_id(msg->src_id());
  b.set_tid(msg->tid());
  b.set_is_ack(true);
  f.add_to_frontier(1 + time(), b.msg());
}

void SnoopFilterCommandInvoker::handle_send_fwd_gets_to_owner(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::FwdGetS);
  b.set_dst_id(d.owner());
  b.set_tid(msg->tid());
  b.set_is_ack(true);
  f.add_to_frontier(1 + time(), b.msg());
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
    const AgentOptions & opts) {

  switch (opts.protocol()) {
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

CoherentAgentModel::CoherentAgentModel(const AgentOptions & opts) {}
SnoopFilterModel::SnoopFilterModel(const SnoopFilterOptions & opts) {}

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    const SnoopFilterOptions & opts) {

  switch (opts.protocol()) {
    case Protocol::MSI:
      return std::make_unique<MsiSnoopFilterModel>(opts);
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
