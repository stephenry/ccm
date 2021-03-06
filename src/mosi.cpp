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

#include "mosi.hpp"
#include "agent.hpp"
#include "snoopfilter.hpp"

namespace ccm {

const char* MosiAgentLineState::to_string(state_t state) {
  switch (state) {
#define __declare_state(__name)    \
  case MosiAgentLineState::__name: \
    return #__name;                \
    break;
    MOSI_LINE_STATES(__declare_state)
#undef __declare_state
    default:
      return "Invalid State";
  }
}

bool MosiAgentLineState::is_stable(state_t state) {
  bool ret{false};
  switch (state) {
    case MosiAgentLineState::I:
    case MosiAgentLineState::S:
    case MosiAgentLineState::O:
    case MosiAgentLineState::M:
      ret = true;
      break;

    default:
      ret = false;
      break;
  }
  return ret;
}

const char* MosiDirectoryLineState::to_string(state_t state) {
  switch (state) {
#define __declare_state(__name)        \
  case MosiDirectoryLineState::__name: \
    return #__name;                    \
    break;
    MOSI_DIRECTORY_STATES(__declare_state)
#undef __declare_state
    default:
      return "Invalid State";
  }
}

bool MosiDirectoryLineState::is_stable(state_t state) {
  bool ret{false};
  switch (state) {
    case MosiDirectoryLineState::I:
    case MosiDirectoryLineState::S:
    case MosiDirectoryLineState::O:
    case MosiDirectoryLineState::M:
      ret = true;
      break;
    default:
      ret = false;
      break;
  }
  return ret;
}

struct MosiAgentProtocol::MosiAgentProtocolImpl {
  MosiAgentProtocolImpl(const Platform & platform) : platform_(platform) {}

  void init(CacheLine& cache_line) const {
    cache_line.set_state(MosiAgentLineState::I);
  }

  bool is_stable(state_t state) const {
    return MosiAgentLineState::is_stable(state);
  }

  std::string to_string(state_t state) const {
    return MosiAgentLineState::to_string(state);
  }

  CoherenceActions get_actions(const Transaction* t,
                               const CacheLine& cache_line) const {
    CoherenceActions actions;
    switch (t->type()) {
      case TransactionType::load:
        handle_load(t, cache_line, actions);
        break;
      case TransactionType::store:
        handle_store(t, cache_line, actions);
        break;
      case TransactionType::replacement:
        handle_replacement(t, cache_line, actions);
        break;
      default:
        actions.set_error(true);
        break;
    }
    return actions;
  }

  CoherenceActions get_actions(const Message* m,
                               const CacheLine& cache_line) const {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::FwdGetS:
        handle_fwd_gets(m, cache_line, actions);
        break;
      case MessageType::FwdGetM:
        handle_fwd_getm(m, cache_line, actions);
        break;
      case MessageType::Inv:
        handle_inv(m, cache_line, actions);
        break;
      case MessageType::PutS:
      case MessageType::PutM:
        handle_put_ack(m, cache_line, actions);
        break;
      case MessageType::Data:
        handle_data(m, cache_line, actions);
        break;
      case MessageType::AckCount:
        handle_ack_count(m, cache_line, actions);
      default:
        actions.set_error(true);
        break;
    }
    return actions;
  }

 private:
  void handle_load(const Transaction* t, const CacheLine& cache_line,
                    CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetS);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IS_D);
        a.set_result(TransactionResult::Miss);
        break;
      case MosiAgentLineState::IS_D:
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::S:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
      case MosiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::O:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_store(const Transaction* t, const CacheLine& cache_line,
                     CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IM_AD);
        a.set_result(TransactionResult::Miss);
        break;
      case MosiAgentLineState::IS_D:
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::SM_AD);
        a.set_result(TransactionResult::Miss);
        break;
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OM_AC);
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_replacement(const Transaction* t, const CacheLine& cache_line,
                          CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::IS_D:
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
      case MosiAgentLineState::MI_A:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;
      case MosiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitPutS);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::SI_A);
        a.set_result(TransactionResult::Consumed);
        break;
      case MosiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitPutM);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::MI_A);
        a.set_result(TransactionResult::Consumed);
        break;
      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitPutO);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OI_A);
        a.set_result(TransactionResult::Consumed);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_fwd_gets(const Message* m, const CacheLine& cache_line,
                       CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;
      case MosiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::O);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OI_A);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::O:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
      case MosiAgentLineState::OI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.set_result(MessageResult::Commit);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_fwd_getm(const Message* m, const CacheLine& cache_line,
                       CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;
      case MosiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::II_A);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::OM_AC:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IM_AD);
        a.set_result(MessageResult::Commit);
        break;
      case MosiAgentLineState::OM_A:
        a.set_result(MessageResult::Stall);
        break;
      case MosiAgentLineState::OI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::II_A);
        a.set_result(MessageResult::Commit);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_inv(const Message* m, const CacheLine& cache_line,
                  CoherenceActions& a) const {
    if (m->is_ack()) {
      a.append_command(CoherentAgentCommand::IncAckCount);

      switch (cache_line.state()) {
        case MosiAgentLineState::IM_A:
        case MosiAgentLineState::SM_A:
        case MosiAgentLineState::OM_A: {
          if (cache_line.is_last_inv_ack()) {
            a.append_command(CoherentAgentCommand::UpdateState);
            a.set_next_state(MosiAgentLineState::M);
          }
        } break;
        default:
          break;
      }

    } else {
      // Invalidation request
      switch (cache_line.state()) {
        case MosiAgentLineState::IS_D:
          a.set_result(MessageResult::Stall);
          break;
        case MosiAgentLineState::S:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::I);
          a.set_result(MessageResult::Commit);
          break;
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::IM_AD);
          a.set_result(MessageResult::Commit);
          break;
        case MosiAgentLineState::SI_A:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::II_A);
          a.set_result(MessageResult::Commit);
          break;
        default:
          a.set_error(true);
          break;
      }
    }
  }

  void handle_put_ack(const Message* m, const CacheLine& cache_line,
                      CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::MI_A:
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_data(const Message* m, const CacheLine& cache_line,
                   CoherenceActions& a) const {
    const bool is_from_dir = platform_.is_valid_snoop_filter_id(m->src_id());
    const bool is_data_from_dir_ack_zero = is_from_dir && (m->ack_count() == 0);
    const bool is_data_from_dir_ack_non_zero =
        is_from_dir && (m->ack_count() != 0);

    const bool is_data_from_owner = !is_from_dir;

    if (is_data_from_dir_ack_zero || is_data_from_owner) {
      switch (cache_line.state()) {
        case MosiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::S);
          a.set_transaction_done(true);
          break;
        case MosiAgentLineState::IM_AD:
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::M);
          a.set_transaction_done(true);
          break;
        default:
          a.set_error(true);
          break;
      }

    } else if (is_data_from_dir_ack_non_zero) {
      switch (cache_line.state()) {
        case MosiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::IM_A);
          break;
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::SM_A);
          break;
        default:
          a.set_error(true);
          break;
      }
    }
  }

  void handle_ack_count(const Message* m, const CacheLine& cache_line,
                        CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MosiAgentLineState::OM_AC:
        a.append_command(CoherentAgentCommand::SetAckExpectCount);
        a.set_ack_count(m->ack_count());
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  const Platform platform_;
};

MosiAgentProtocol::MosiAgentProtocol(const Platform & platform) {
  impl_ = std::make_unique<MosiAgentProtocolImpl>(platform);
}

MosiAgentProtocol::~MosiAgentProtocol() {}

void MosiAgentProtocol::init(CacheLine& l) const { impl_->init(l); }

bool MosiAgentProtocol::is_stable(const CacheLine& l) const {
  return impl_->is_stable(l.state());
}

std::string MosiAgentProtocol::to_string(state_t s) const {
  return impl_->to_string(s);
}

//
CoherenceActions MosiAgentProtocol::get_actions(
    const Transaction* t, const CacheLine& cache_line) const {
  return impl_->get_actions(t, cache_line);
}

CoherenceActions MosiAgentProtocol::get_actions(
    const Message* m, const CacheLine& cache_line) const {
  return impl_->get_actions(m, cache_line);
}

struct MosiSnoopFilterProtocol::MosiSnoopFilterProtocolImpl {
  MosiSnoopFilterProtocolImpl() {}

  void init(DirectoryLine& l) const { l.set_state(MosiDirectoryLineState::I); }

  bool is_stable(const DirectoryLine& dir_entry) const {
    return MosiDirectoryLineState::to_string(dir_entry.state());
  }

  std::string to_string(const DirectoryLine& dir_entry) const {
    std::stringstream ss;
    ss << MosiDirectoryLineState::to_string(dir_entry.state());
    return ss.str();
  }

  std::string to_string(state_t state) const {
    return MosiDirectoryLineState::to_string(state);
  }

  CoherenceActions get_actions(const Message* m,
                               const DirectoryLine& dir_entry) {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::GetS:
        handle_gets(m, dir_entry, actions);
        break;
      case MessageType::GetM:
        handle_getm(m, dir_entry, actions);
        break;
      case MessageType::PutS:
        handle_puts(m, dir_entry, actions);
        break;
      case MessageType::PutM:
        handle_putm(m, dir_entry, actions);
        break;
      case MessageType::PutO:
        handle_puto(m, dir_entry, actions);
        break;
      default:
        actions.set_error(true);
    }
    return actions;
  }

 private:
  void handle_gets(const Message* m, const DirectoryLine& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MosiDirectoryLineState::S);
        break;
      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;
      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.set_fwd_id(m->src_id());
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;
      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.set_fwd_id(m->src_id());
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MosiDirectoryLineState::O);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_getm(const Message* m, const DirectoryLine& dir_entry,
                    CoherenceActions& a) const {
    const bool is_from_owner = false;  // TODO

    if (is_from_owner) {
      switch (dir_entry.state()) {
        case MosiDirectoryLineState::O:
          a.append_command(SnoopFilterCommand::SendAckCountToReq);
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::O);
          break;
        default:
          a.set_error(true);
          break;
      }

    } else {
      switch (dir_entry.state()) {
        case MosiDirectoryLineState::I:
          a.append_command(SnoopFilterCommand::SendDataToReq);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;
        case MosiDirectoryLineState::S:
          a.append_command(SnoopFilterCommand::SendDataToReq);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;
        case MosiDirectoryLineState::O:
          a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
          a.set_ack_count(2);
          a.set_fwd_id(m->src_id());
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;
        case MosiDirectoryLineState::M:
          a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
          a.set_fwd_id(m->src_id());
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          break;
        default:
          a.set_error(true);
          break;
      }
    }
  }

  void handle_puts(const Message* m, const DirectoryLine& dir_entry,
                    CoherenceActions& a) const {
    const bool is_last = false;  // TODO

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;
      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        if (is_last) {
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;
      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;
      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_putm(const Message* m, const DirectoryLine& dir_entry,
                    CoherenceActions& a) const {
    const bool data_is_from_owner = false;  // TODO

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        break;
      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        break;
      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;
      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;
      default:
        a.set_error(true);
        break;
    }
  }

  void handle_puto(const Message* m, const DirectoryLine& dir_entry,
                    CoherenceActions& a) const {
    const bool data_is_from_owner = false;  // TODO

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        break;
      case MosiDirectoryLineState::S:
        if (!data_is_from_owner) {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        }
        break;
      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::S);
        } else {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
        }
        break;
      case MosiDirectoryLineState::M:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        break;
      default:
        a.set_error(true);
        break;
    }
  }
};

MosiSnoopFilterProtocol::MosiSnoopFilterProtocol() {
  impl_ = std::make_unique<MosiSnoopFilterProtocolImpl>();
}

MosiSnoopFilterProtocol::~MosiSnoopFilterProtocol() {}

void MosiSnoopFilterProtocol::init(DirectoryLine& l) const { impl_->init(l); }

bool MosiSnoopFilterProtocol::is_stable(const DirectoryLine& l) const {
  return impl_->is_stable(l);
}

std::string MosiSnoopFilterProtocol::to_string(const DirectoryLine& l) const {
  return impl_->to_string(l);
}

std::string MosiSnoopFilterProtocol::to_string(state_t state) const {
  return impl_->to_string(state);
}

CoherenceActions MosiSnoopFilterProtocol::get_actions(
    const Message* m, const DirectoryLine& dir_entry) const {
  return impl_->get_actions(m, dir_entry);
}

MosiCoherenceProtocolValidator::MosiCoherenceProtocolValidator() {}

bool MosiCoherenceProtocolValidator::validate_addr(
    addr_t addr, const std::vector<Entry<CacheLine> >& lines,
    const DirectoryLine& entry) const {
  // TODO
  return true;
}

}  // namespace ccm
