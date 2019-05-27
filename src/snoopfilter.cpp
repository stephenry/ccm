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
#ifdef ENABLE_JSON

SnoopFilterOptions SnoopFilterOptions::from_json(
    const Platform & platform, LoggerScope *l, nlohmann::json & j) {
  const id_t id = j["id"];
  SnoopFilterOptions opts{id, platform, CacheOptions::from_json(j["cache_options"])};
  opts.set_logger_scope(l->child_scope(j["name"]));
  return opts;
}
#endif

SnoopFilterCommandInvoker::SnoopFilterCommandInvoker(
    const SnoopFilterOptions& opts)
    : opts_(opts), msgd_(opts), CoherentActor(opts) {
  cc_model_ = snoop_filter_protocol_factory(opts.protocol());
  cache_ = cache_factory<DirectoryLine>(opts.cache_options());
}

DirectoryLine SnoopFilterCommandInvoker::directory_entry(
    std::size_t addr) const {
  DirectoryLine directory_entry;
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
                                        const Message* msg, DirectoryLine& d) {
  for (command_t cmd : actions.commands()) {
    log_debug(cursor.time(), "Execute: ", SnoopFilterCommand::to_string(cmd));

    bool updated_cursor{false};
    switch (cmd) {
      case SnoopFilterCommand::UpdateState:
        updated_cursor = execute_update_state(context, cursor, d, actions.next_state());
        break;
      case SnoopFilterCommand::SetOwnerToReq:
        updated_cursor = execute_set_owner_to_req(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::SendDataToReq:
        updated_cursor = execute_send_data_to_req(msg, context, cursor, d, actions);
        break;
      case SnoopFilterCommand::SendInvToSharers:
        updated_cursor = execute_send_inv_to_sharers(msg, context, cursor, d, actions);
        break;
      case SnoopFilterCommand::ClearSharers:
        updated_cursor = execute_clear_sharers(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::AddReqToSharers:
        updated_cursor = execute_add_req_to_sharers(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::DelReqFromSharers:
        updated_cursor = execute_del_req_from_sharers(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::DelOwner:
        updated_cursor = execute_del_owner(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::AddOwnerToSharers:
        updated_cursor = execute_add_owner_to_sharers(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::CpyDataToMemory:
        updated_cursor = execute_cpy_data_to_memory(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::SendPutSAckToReq:
        updated_cursor = execute_send_puts_ack_to_req(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::SendPutMAckToReq:
        updated_cursor = execute_send_putm_ack_to_req(msg, context, cursor, d);
        break;
      case SnoopFilterCommand::SendFwdGetSToOwner:
        updated_cursor = execute_send_fwd_gets_to_owner(msg, context, cursor, d, actions);
        break;
      case SnoopFilterCommand::SendPutEAckToReq:
        updated_cursor = execute_send_pute_ack_to_req(msg, context, cursor);
        break;
      case SnoopFilterCommand::SendPutOAckToReq:
        updated_cursor = execute_send_puto_ack_to_req(msg, context, cursor);
        break;
      case SnoopFilterCommand::SendAckCountToReq:
        updated_cursor = execute_send_ack_count_to_req(msg, context, cursor, actions);
        break;
      case SnoopFilterCommand::SendFwdGetMToOwner:
        updated_cursor = execute_send_fwd_getm_to_owner(msg, context, cursor, d, actions);
        break;
      default:
        break;
    }
    if (!updated_cursor)
      cursor.advance(CoherentAgentCommand::to_cost(cmd));
  }
}

bool SnoopFilterCommandInvoker::execute_update_state(Context& context,
                                                     Cursor& cursor,
                                                     DirectoryLine& d,
                                                     state_t state_next) {
  log_debug(cursor.time(), "Update state; current: ", cc_model_->to_string(state_next),
            " previous: ", cc_model_->to_string(d.state()));
  d.set_state(state_next);
  return false;
}

bool SnoopFilterCommandInvoker::execute_set_owner_to_req(const Message* msg,
                                                         Context& context,
                                                         Cursor& cursor,
                                                         DirectoryLine& d) {
  log_debug(cursor.time(), "Set Owner To Requester: Owner = ", msg->src_id());
  d.set_owner(msg->src_id());
  return false;
}

bool SnoopFilterCommandInvoker::execute_send_data_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d,
    const CoherenceActions& a) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::Data);
  b.set_dst_id(msg->src_id());
  b.set_transaction(msg->transaction());
  b.set_ack_count(a.ack_count());
  b.set_is_exclusive(a.is_exclusive());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending data to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_inv_to_sharers(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d,
    const CoherenceActions& actions) {
  log_debug(cursor.time(), "Send Invalidation(s) to sharers.");

  for (const std::size_t sharer : d.sharers()) {
    // Do not invalidation request to requester, only the other
    // sharing agents in the system.
    //
    if (sharer == msg->src_id()) continue;

    MessageBuilder b = msgd_.builder();

    b.set_src_id(id());
    b.set_type(MessageType::Inv);
    b.set_dst_id(sharer);
    b.set_fwd_id(actions.fwd_id());
    b.set_transaction(msg->transaction());

    Message *out = b.msg();
    log_debug(cursor.time(), "Sending invalidation to agent: ", to_string(*out));
    context.emit_message(TimeStamped{cursor.time(), out});

    cursor.advance(MessageType::to_cost(MessageType::Inv));
  }

  return true;
}

bool SnoopFilterCommandInvoker::execute_clear_sharers(const Message* msg,
                                                      Context& context,
                                                      Cursor& cursor,
                                                      DirectoryLine& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.clear_sharers();
  l.add(d.sharers(), " after = ");
  log_debug(cursor.time(), "Clear sharers; ", l.str());

  return false;
}

bool SnoopFilterCommandInvoker::execute_add_req_to_sharers(const Message* msg,
                                                           Context& context,
                                                           Cursor& cursor,
                                                           DirectoryLine& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.add_sharer(msg->src_id());
  l.add(d.sharers(), " after = ");

  log_debug(cursor.time(), "Add requester to sharers: ", l.str());

  return false;
}

bool SnoopFilterCommandInvoker::execute_del_req_from_sharers(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d) {
  StateUpdateLogger l;

  l.add(d.sharers(), " before = ");
  d.remove_sharer(msg->src_id());
  l.add(d.sharers(), " after = ");

  log_debug(cursor.time(), "Remove requester from sharers; ", l.str());

  return false;
}

bool SnoopFilterCommandInvoker::execute_del_owner(const Message* msg,
                                                  Context& context,
                                                  Cursor& cursor,
                                                  DirectoryLine& d) {
  log_debug(cursor.time(), "Delete owner.");
  d.clear_owner();

  return false;
}

bool SnoopFilterCommandInvoker::execute_add_owner_to_sharers(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d) {
  log_debug(cursor.time(), "Add owner to sharers.");

  d.add_sharer(d.owner());

  return false;
}

bool SnoopFilterCommandInvoker::execute_cpy_data_to_memory(const Message* msg,
                                                           Context& context,
                                                           Cursor& cursor,
                                                           DirectoryLine& d) {
  const Platform platform = opts_.platform();

  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::Data);
  b.set_dst_id(platform.memory_id());
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Copy Data to Memory: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_puts_ack_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutS);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending PutS acknowledgement to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_putm_ack_to_req(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutM);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending PutM acknowledgement to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_fwd_gets_to_owner(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::FwdGetS);
  b.set_dst_id(d.owner());
  b.set_fwd_id(actions.fwd_id());
  b.set_transaction(msg->transaction());
  b.set_ack_count(0);

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending FwdGetS to owner: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_pute_ack_to_req(const Message* msg,
                                                             Context& context,
                                                             Cursor& cursor) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutE);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending PutEAck to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_puto_ack_to_req(const Message* msg,
                                                             Context& context,
                                                             Cursor& cursor) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutO);
  b.set_dst_id(msg->src_id());
  b.set_is_ack(true);
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending PutOAck to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_ack_count_to_req(
    const Message* msg, Context& context, Cursor& cursor,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::AckCount);
  b.set_dst_id(msg->src_id());
  b.set_ack_count(actions.ack_count());
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending AckCount to requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

bool SnoopFilterCommandInvoker::execute_send_fwd_getm_to_owner(
    const Message* msg, Context& context, Cursor& cursor, DirectoryLine& d,
    const CoherenceActions& actions) {
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::FwdGetM);
  b.set_dst_id(d.owner());
  b.set_fwd_id(msg->src_id());
  b.set_ack_count(actions.ack_count());
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending FwdGetM to owner: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});

  return false;
}

SnoopFilter::SnoopFilter(const SnoopFilterOptions& opts)
    : SnoopFilterCommandInvoker(opts), opts_(opts) {
  set_logger_scope(opts.logger_scope());
}

void SnoopFilter::apply(TimeStamped<Message*> ts) { mq_.push_back(ts); }

void SnoopFilter::eval(Context& context) {
  const Epoch epoch = context.epoch();
  Cursor cursor = epoch.cursor();

  if (!epoch.in_interval(time())) return;

  cursor.set_time(std::max(time(), cursor.time()));

  MessageQueueManager::Proxy mqp{mq_};
  do {
    if (!mq_.is_active()) break;

    mqp.recompute_front();
    const TimeStamped<Message *> ts{mqp.front()};
    const Message * msg = ts.t();
    
    switch (handle_msg(context, cursor, msg)) {
      case MessageResult::Commit:
        msg->release();
        mqp.pop_front();
        break;
      case MessageResult::Stall:
        mqp.add_disregard_class(MessageType::to_class(msg->type()));
        break;
    }
  } while (epoch.in_interval(cursor.time()));

  set_time(std::max(cursor.time(), epoch.end()));
}

result_t SnoopFilter::handle_msg(Context& context, Cursor& cursor, const Message * msg) {
  const Transaction* transaction = msg->transaction();

  if (!cache_->is_hit(transaction->addr())) {
    DirectoryLine directory_entry;
    cc_model_->init(directory_entry);
    cache_->install(transaction->addr(), directory_entry);
  }

  DirectoryLine& directory_entry = cache_->lookup(transaction->addr());
  const CoherenceActions actions = cc_model_->get_actions(msg, directory_entry);
  execute(context, cursor, actions, msg, directory_entry);
  return actions.result();
}
#ifdef ENABLE_JSON

std::unique_ptr<SnoopFilter> SnoopFilterBuilder::construct(
    const Platform & platform, LoggerScope * l, nlohmann::json j) {
  return std::make_unique<SnoopFilter>(
      SnoopFilterOptions::from_json(platform, l, j));
}
#endif

}  // namespace ccm
