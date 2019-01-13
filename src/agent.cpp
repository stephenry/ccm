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

#include "agent.hpp"
#include "msi.hpp"
#include "common.hpp"
#include <queue>

namespace ccm {

class AgentMessageAdmissionControl : public MessageAdmissionControl {
    
public:
  AgentMessageAdmissionControl(Agent * agent)
    : agent_(agent)
  {}

  bool can_be_issued(const Message * msg) const override {
    const Transaction * trn = msg->transaction();
    CacheLine & cache_line = agent_->cache_->lookup(trn->addr());
    const CoherenceActions actions =
      agent_->cc_model_->get_actions(msg, cache_line);

    switch (actions.result()) {
    case MessageResult::Stall:
      return false;
      break;

    case MessageResult::Commit:
      return true;
      break;

    default:
      // TODO: Error
      break;
    }
    
    return true;
  }
  
private:
  Agent * agent_{nullptr};
};

class AgentTransactionAdmissionControl : public TransactionAdmissionControl {
public:
  AgentTransactionAdmissionControl(Agent * agent)
    : agent_(agent)
  {}

  bool can_be_issued(const Transaction * trn) const override {

    // TODO: variety of agent related issue conditions (max in flight count,
    // credits, etc..)

    // The coherence protocol cannot block if (by definition) the
    // transaction address is not present in the agents cache.
    //
    if (!agent_->cache_->is_hit(trn->addr()))
      return true;

    // If the address is present in the agents cache, explicitly
    // derive the set of actions to be applied and verify that the
    // protocol may advance.
    //
    // This is an expensive operation.
    //
    CacheLine & cache_line = agent_->cache_->lookup(trn->addr());
    const CoherenceActions actions =
      agent_->cc_model_->get_actions(trn, cache_line);
    switch (actions.result()) {
    case TransactionResult::Blocked:
      return false;
      break;
      
    case TransactionResult::Hit:
    case TransactionResult::Miss:
      return true;
      break;

    default:
      // Error
      return false;
    }
  }
  
private:
  Agent * agent_{nullptr};
};
  

Agent::Agent(const AgentOptions & opts)
    : CoherentAgentCommandInvoker(opts), opts_(opts) {
  set_time(0);
  set_logger_scope(opts.logger_scope());

  qmgr_.set_ac(std::make_unique<AgentMessageAdmissionControl>(this));
  qmgr_.set_ac(std::make_unique<AgentTransactionAdmissionControl>(this));
}

void Agent::eval(Context & context) {
  const Epoch epoch = context.epoch();
  Cursor cursor = epoch.cursor();

  // The agent is present at 'time()'; execute Agent actions iff the
  // time is present in the current Epoch, otherwise, sleep until the
  // next interval.
  //
  if (!epoch.in_interval(time()))
    return;

  // As the current time may not fall on a Epoch boundary, advance to
  // the 'time' location within the current Epoch and proceed from
  // there.
  //
  cursor.set_time(std::max(time(), cursor.time()));

  if (!qmgr_.pending_transactions())
    fetch_transactions(10);

  do {
    const QueueEntry next = qmgr_.next();
    if (next.type() == QueueEntryType::Invalid)
      break;
    
    if (!epoch.in_interval(next.time()))
      break;

    set_time(next.time());

    CoherenceActions actions;
    switch (next.type()) {
    case QueueEntryType::Message:
      handle_msg(context, cursor, next.as_msg());
      break;
      
    case QueueEntryType::Transaction:
      handle_trn(context, cursor, next.as_trn());
      break;

    default:
      break;
    }
    next.consume();
  } while (epoch.in_interval(cursor.time()));

  set_time(cursor.time());
}

void Agent::fetch_transactions(std::size_t n) {
  TimeStamped<Transaction *> ts;
  for (int i = 0; i < n; i++) {
    if (!trns_->get_transaction(ts))
      break;
    
    qmgr_.push(ts);
  }
}

void Agent::handle_msg(Context & context, Cursor & cursor,
                       TimeStamped<Message *> ts) {
  const Message * msg = ts.t();
  const Transaction * trn = msg->transaction();

  CacheLine & cache_line = cache_->lookup(trn->addr());
  const CoherenceActions actions = cc_model_->get_actions(msg, cache_line);

  ExecutionContext ectxt{this, context, cursor};
  execute(ectxt, actions, cache_line, msg->transaction());
  // TODO: assertion that confirms commital

  if (actions.transaction_done())
    trns_->event_finish(TimeStamped{time(), msg->transaction()});

  msg->release();
}

void Agent::handle_trn(Context & context, Cursor & cursor,
                       TimeStamped<Transaction *> ts) {
  Transaction * trn = ts.t();
  
  trns_->event_start(TimeStamped{time(), trn});

  if (trn->type() != TransactionType::Replacement)
    CCM_ASSERT(!cache_->requires_eviction(trn->addr()));
                       
  if (!cache_->is_hit(trn->addr())) {
    CacheLine cache_line;
    cc_model_->init(cache_line);
    cache_->install(trn->addr(), cache_line);
  }

  CacheLine & cache_line = cache_->lookup(trn->addr());
  const CoherenceActions actions = cc_model_->get_actions(trn, cache_line);
  
  ExecutionContext ectxt{this, context, cursor};
  execute(ectxt, actions, cache_line, trn);
}

void Agent::apply(TimeStamped<Message *> ts) {
  qmgr_.push(ts);
}

bool Agent::is_active() const { return !qmgr_.empty(); }

} // namespace ccm
