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
#include "snoopfilter.hpp"
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
    const CoherentAgentOptions & opts) : CoherentActor(opts), msgd_(opts) {
  id_ = opts.id();
  cc_model_ = coherent_agent_factory(opts);
  cache_ = cache_factory<CacheLine>(opts.cache_options());
}

void CoherentAgentCommandInvoker::execute(
    Frontier & f, const CoherentActorActions & actions,
    CacheLine & cache_line, const Transaction * t) {

  for (const uint8_t cmd : actions.actions()) {

    switch (static_cast<CoherentAgentCommand>(cmd)) {
      
      case CoherentAgentCommand::UpdateState:
        execute_update_state(f, cache_line, actions.next_state());
        break;

      case CoherentAgentCommand::EmitGetS:
        execute_emit_gets(f, t);
        break;
      
      case CoherentAgentCommand::EmitGetM:
        execute_emit_getm(f, t);
        break;
      
      case CoherentAgentCommand::EmitDataToReq:
        execute_emit_data_to_req(f, t);
        break;
      
      case CoherentAgentCommand::EmitDataToDir:
        execute_emit_data_to_dir(f, t);
        break;
      
      case CoherentAgentCommand::EmitInvAck:
        execute_emit_inv_ack(f, t);
        break;
    }
  }
}

void CoherentAgentCommandInvoker::execute_update_state(
    Frontier & f, CacheLine & cache_line, state_type state_next) {
  log_debug("Update state; current: ", cc_model_->to_string(state_next),
            " previous: ", cc_model_->to_string(cache_line.state()));
  
  cache_line.set_state(state_next);
}

void CoherentAgentCommandInvoker::execute_emit_gets(
    Frontier & f, const Transaction * t) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::GetS);
  b.set_dst_id(4);
  b.set_transaction(t);

  const Message * msg = b.msg();
  f.add_to_frontier(1 + time(), msg);

  log_debug("Emit GetS: ", to_string(*msg));
}

void CoherentAgentCommandInvoker::execute_emit_getm(
    Frontier & f, const Transaction * t) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::GetM);
  b.set_dst_id(4);
  b.set_transaction(t);

  const Message * msg = b.msg();
  f.add_to_frontier(1 + time(), msg);

  log_debug("Emit GetM: ", to_string(*msg));
}

void CoherentAgentCommandInvoker::execute_emit_data_to_req(
    Frontier & f, const Transaction * t) {
  log_debug("Emit Data To Requester.");
  
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_transaction(t);
}

void CoherentAgentCommandInvoker::execute_emit_data_to_dir(
    Frontier & f, const Transaction * t) {
  log_debug("Emit Data To Directory.");

  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_transaction(t);
}

void CoherentAgentCommandInvoker::execute_emit_inv_ack(
    Frontier & f, const Transaction * t) {
  log_debug("Emit Invalidation Acknowledgement.");

  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Inv);
  b.set_is_ack(true);
  b.set_transaction(t);
}

CoherentAgentModel::CoherentAgentModel(const CoherentAgentOptions & opts) {}

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts) {

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
    const SnoopFilterOptions & opts)
    : opts_(opts), msgd_(opts), CoherentActor(opts) {
  id_ = opts.id();
  cc_model_ = snoop_filter_factory(opts);
}

void SnoopFilterCommandInvoker::execute(
    Frontier & f, const CoherentActorActions & actions,
    const Message * msg, DirectoryEntry & d) {

  for (const uint8_t cmd : actions.actions()) {

    switch (static_cast<SnoopFilterCommand>(cmd)) {
      case SnoopFilterCommand::UpdateState:
        execute_update_state(f, d, actions.next_state());
        break;

      case SnoopFilterCommand::SetOwnerToReq:
        execute_set_owner_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendDataToReq:
        execute_send_data_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendInvToSharers:
        execute_send_inv_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::ClearSharers:
        execute_clear_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::AddReqToSharers:
        execute_add_req_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::DelReqFromSharers:
        execute_del_req_from_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::DelOwner:
        execute_del_owner(msg, f, d);
        break;

      case SnoopFilterCommand::AddOwnerToSharers:
        execute_add_owner_to_sharers(msg, f, d);
        break;

      case SnoopFilterCommand::CpyDataToMemory:
        execute_cpy_data_to_memory(msg, f, d);
        break;

      case SnoopFilterCommand::SendPutSAckToReq:
        execute_send_put_sack_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendPutMAckToReq:
        execute_send_put_mack_to_req(msg, f, d);
        break;

      case SnoopFilterCommand::SendFwdGetSToOwner:
        execute_send_fwd_gets_to_owner(msg, f, d);
        break;
    }
  }
}

void SnoopFilterCommandInvoker::execute_update_state(
    Frontier & f, DirectoryEntry & d, state_type state_next) {
  log_debug("Update state; current: ", cc_model_->to_string(state_next),
            " previous: ", cc_model_->to_string(d.state()));
  
  d.set_state(state_next);
}

void SnoopFilterCommandInvoker::execute_set_owner_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  log_debug("Set Owner To Requester: Owner = ", msg->src_id());
  
  d.set_owner(msg->src_id());
}

void SnoopFilterCommandInvoker::execute_send_data_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_dst_id(msg->src_id());
  b.set_transaction(msg->transaction());

  const Message * m = b.msg();
  f.add_to_frontier(1 + time(), m);
  
  log_debug("Send Data to Requester: ", to_string(*m));
}

void SnoopFilterCommandInvoker::execute_send_inv_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  log_debug("Send Invalidation(s) to sharers.");
  
  std::size_t time_start = 1 + time();
  MessageBuilder b = msgd_.builder();
  for (std::size_t sharer : d.sharers()) {
    b.set_type(MessageType::Inv);
    b.set_dst_id(sharer);
    b.set_transaction(msg->transaction());

    const Message * out_msg = b.msg();
    f.add_to_frontier(time_start++, out_msg);
    log_debug("Sending invalidation: ", to_string(*out_msg));
  }
}

void SnoopFilterCommandInvoker::execute_clear_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.clear_sharers();
  l.add(d.sharers(), " after = ");
  log_debug("Clear sharers; ", l.str());
  
}

void SnoopFilterCommandInvoker::execute_add_req_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.add_sharer(msg->src_id());
  l.add(d.sharers(), " after = " );

  log_debug("Add requester to sharers: ", l.str());
}

void SnoopFilterCommandInvoker::execute_del_req_from_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.remove_sharer(msg->src_id());
  l.add(d.sharers(), " after = " );

  log_debug("Remove requester from sharers; ",l.str());
}

void SnoopFilterCommandInvoker::execute_del_owner(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  log_debug("Delete owner.");
  
  d.clear_owner();
}

void SnoopFilterCommandInvoker::execute_add_owner_to_sharers(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  log_debug("Add owner to sharers.");
  
  d.add_sharer(d.owner());
}

void SnoopFilterCommandInvoker::execute_cpy_data_to_memory(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  log_debug("Copy Data to Memory.");
  
  // NOP
}

void SnoopFilterCommandInvoker::execute_send_put_sack_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutS);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());
  const Message * out_msg = b.msg();
  
  f.add_to_frontier(1 + time(), out_msg);
  log_debug("Send PutS acknowledgement to requester: ", to_string(*out_msg));
}

void SnoopFilterCommandInvoker::execute_send_put_mack_to_req(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutM);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());
  const Message * out_msg = b.msg();
  
  f.add_to_frontier(1 + time(), out_msg);
  log_debug("Send PutM acknowledgement to requester: ", to_string(*out_msg));
}

void SnoopFilterCommandInvoker::execute_send_fwd_gets_to_owner(
    const Message * msg, Frontier & f, DirectoryEntry & d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::FwdGetS);
  b.set_dst_id(d.owner());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());
  const Message * out_msg = b.msg();
  
  f.add_to_frontier(1 + time(), out_msg);
  log_debug("Send FwdGetS to owner: ", to_string(out_msg));
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

SnoopFilterModel::SnoopFilterModel(const SnoopFilterOptions & opts)
    : opts_(opts)
{}

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
