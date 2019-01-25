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

#include "snoopfilter.hpp"

namespace ccm {

const char* SnoopFilterCommand::to_string(command_t command) {
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

SnoopFilterCommandInvoker::SnoopFilterCommandInvoker(
    const SnoopFilterOptions& opts)
    : opts_(opts), msgd_(opts), CoherentActor(opts) {
  cc_model_ = snoop_filter_factory(opts.protocol(), opts);
  cache_ = cache_factory<DirectoryEntry>(opts.cache_options());
}

DirectoryEntry SnoopFilterCommandInvoker::directory_entry(
    std::size_t addr) const {
  DirectoryEntry directory_entry;
  if (cache_->is_hit(addr)) {
    directory_entry = cache_->lookup(addr);
  } else {
    cc_model_->init(directory_entry);
  }
  return directory_entry;
}

void SnoopFilterCommandInvoker::visit_cache(CacheVisitor* cache_visitor) const {
  cache_visitor->set_id(id());
  cache_->visit(cache_visitor);
}

void SnoopFilterCommandInvoker::execute(Context& context, Cursor& cursor,
                                        const CoherenceActions& actions,
                                        const Message* msg, DirectoryEntry& d) {
  for (command_t cmd : actions.commands()) {
    log_debug("Execute: ", SnoopFilterCommand::to_string(cmd));

    switch (cmd) {
      case SnoopFilterCommand::UpdateState:
        execute_update_state(context, cursor, d, actions.next_state());
        break;

      case SnoopFilterCommand::SetOwnerToReq:
        execute_set_owner_to_req(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::SendDataToReq:
        execute_send_data_to_req(msg, context, cursor, d, actions);
        break;

      case SnoopFilterCommand::SendInvToSharers:
        execute_send_inv_to_sharers(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::ClearSharers:
        execute_clear_sharers(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::AddReqToSharers:
        execute_add_req_to_sharers(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::DelReqFromSharers:
        execute_del_req_from_sharers(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::DelOwner:
        execute_del_owner(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::AddOwnerToSharers:
        execute_add_owner_to_sharers(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::CpyDataToMemory:
        execute_cpy_data_to_memory(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::SendPutSAckToReq:
        execute_send_puts_ack_to_req(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::SendPutMAckToReq:
        execute_send_putm_ack_to_req(msg, context, cursor, d);
        break;

      case SnoopFilterCommand::SendFwdGetSToOwner:
        execute_send_fwd_gets_to_owner(msg, context, cursor, d, actions);
        break;

      case SnoopFilterCommand::SendPutEAckToReq:
        execute_send_pute_ack_to_req(msg, context, cursor);
        break;

      case SnoopFilterCommand::SendPutOAckToReq:
        execute_send_puto_ack_to_req(msg, context, cursor);
        break;

      case SnoopFilterCommand::SendAckCountToReq:
        execute_send_ack_count_to_req(msg, context, cursor, actions);
        break;

      case SnoopFilterCommand::SendFwdGetMToOwner:
        execute_send_fwd_getm_to_owner(msg, context, cursor, d, actions);
        break;

      default:
        break;
    }
  }
}

void SnoopFilterCommandInvoker::execute_update_state(Context& context,
                                                     Cursor& cursor,
                                                     DirectoryEntry& d,
                                                     state_t state_next) {
  log_debug("Update state; current: ", cc_model_->to_string(state_next),
            " previous: ", cc_model_->to_string(d.state()));

  d.set_state(state_next);
}

void SnoopFilterCommandInvoker::execute_set_owner_to_req(const Message* msg,
                                                         Context& context,
                                                         Cursor& cursor,
                                                         DirectoryEntry& d) {
  log_debug("Set Owner To Requester: Owner = ", msg->src_id());

  d.set_owner(msg->src_id());
}

void SnoopFilterCommandInvoker::execute_send_data_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d,
    const CoherenceActions& a) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_dst_id(msg->src_id());
  b.set_transaction(msg->transaction());

  b.set_ack_count(a.ack_count());
  //  b.set_ack_count(0);
  b.set_is_exclusive(a.is_exclusive());

  log_debug("Sending data to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_inv_to_sharers(const Message* msg,
                                                            Context& context,
                                                            Cursor& cursor,
                                                            DirectoryEntry& d) {
  log_debug("Send Invalidation(s) to sharers.");

  for (const std::size_t sharer : d.sharers()) {
    // Do not invalidation request to requester, only the other
    // sharing agents in the system.
    //
    if (sharer == msg->src_id()) continue;

    MessageBuilder b = msgd_.builder();
    b.set_type(MessageType::Inv);
    b.set_dst_id(sharer);
    b.set_transaction(msg->transaction());

    log_debug("Sending invalidation to agent ", sharer);
    emit_message(context, cursor, b);
  }
}

void SnoopFilterCommandInvoker::execute_clear_sharers(const Message* msg,
                                                      Context& context,
                                                      Cursor& cursor,
                                                      DirectoryEntry& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.clear_sharers();
  l.add(d.sharers(), " after = ");
  log_debug("Clear sharers; ", l.str());
}

void SnoopFilterCommandInvoker::execute_add_req_to_sharers(const Message* msg,
                                                           Context& context,
                                                           Cursor& cursor,
                                                           DirectoryEntry& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.add_sharer(msg->src_id());
  l.add(d.sharers(), " after = ");

  log_debug("Add requester to sharers: ", l.str());
}

void SnoopFilterCommandInvoker::execute_del_req_from_sharers(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.remove_sharer(msg->src_id());
  l.add(d.sharers(), " after = ");

  log_debug("Remove requester from sharers; ", l.str());
}

void SnoopFilterCommandInvoker::execute_del_owner(const Message* msg,
                                                  Context& context,
                                                  Cursor& cursor,
                                                  DirectoryEntry& d) {
  log_debug("Delete owner.");

  d.clear_owner();
}

void SnoopFilterCommandInvoker::execute_add_owner_to_sharers(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d) {
  log_debug("Add owner to sharers.");

  d.add_sharer(d.owner());
}

void SnoopFilterCommandInvoker::execute_cpy_data_to_memory(const Message* msg,
                                                           Context& context,
                                                           Cursor& cursor,
                                                           DirectoryEntry& d) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::Data);
  b.set_dst_id(platform.memory_id());
  b.set_transaction(msg->transaction());

  log_debug("Copy Data to Memory.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_puts_ack_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutS);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  log_debug("Sending PutS acknowledgement to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_putm_ack_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::PutM);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  log_debug("Sending PutM acknowledgement to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_fwd_gets_to_owner(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::FwdGetS);
  b.set_dst_id(d.owner());
  b.set_fwd_id(actions.fwd_id());
  b.set_transaction(msg->transaction());

  log_debug("Sending FwdGetS to owner.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_pute_ack_to_req(const Message* msg,
                                                             Context& context,
                                                             Cursor& cursor) {
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::PutE);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);

  log_debug("Sending PutEAck to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_puto_ack_to_req(const Message* msg,
                                                             Context& context,
                                                             Cursor& cursor) {
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::PutO);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);

  log_debug("Sending PutOAck to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_ack_count_to_req(
    const Message* msg, Context& context, Cursor& cursor,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::AckCount);
  b.set_dst_id(msg->src_id());
  b.set_ack_count(actions.ack_count());

  log_debug("Sending AckCount to requester.");
  emit_message(context, cursor, b);
}

void SnoopFilterCommandInvoker::execute_send_fwd_getm_to_owner(
    const Message* msg, Context& context, Cursor& cursor, DirectoryEntry& d,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();
  b.set_type(MessageType::FwdGetM);
  b.set_dst_id(d.owner());
  b.set_fwd_id(msg->src_id());
  b.set_transaction(msg->transaction());
  log_debug("Sending FwdGetM to owner.");
  emit_message(context, cursor, b);
}

SnoopFilter::SnoopFilter(const SnoopFilterOptions& opts)
    : SnoopFilterCommandInvoker(opts), opts_(opts) {
  set_logger_scope(opts.logger_scope());
}

void SnoopFilter::apply(TimeStamped<Message*> ts) { qmgr_.push(ts); }

void SnoopFilter::eval(Context& context) {
  const Epoch epoch = context.epoch();
  Cursor cursor = epoch.cursor();

  do {
    const QueueEntry next = qmgr_.next();
    if (next.type() == QueueEntryType::Invalid) break;

    if (!epoch.in_interval(next.time())) break;

    cursor.set_time(std::max(time(), next.time()));
    set_time(cursor.time());

    CoherenceActions actions;
    switch (next.type()) {
      case QueueEntryType::Message:
        handle_msg(context, cursor, next.as_msg());
        break;
      default:;  // TODO: unexpected
    }
    next.consume();
  } while (epoch.in_interval(cursor.time()));

  set_time(cursor.time());
}

void SnoopFilter::handle_msg(Context& context, Cursor& cursor,
                             TimeStamped<Message*> ts) {
  const Message* msg = ts.t();
  const Transaction* transaction = msg->transaction();

  if (!cache_->is_hit(transaction->addr())) {
    DirectoryEntry directory_entry;
    cc_model_->init(directory_entry);
    cache_->install(transaction->addr(), directory_entry);
  }

  DirectoryEntry& directory_entry = cache_->lookup(transaction->addr());
  const CoherenceActions actions = cc_model_->get_actions(msg, directory_entry);

  execute(context, cursor, actions, msg, directory_entry);
}

}  // namespace ccm
