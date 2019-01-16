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

#include "mesi.hpp"

namespace ccm {

const char* MesiAgentLineState::to_string(state_t state) {
  switch (state) {
#define __to_str(__state)           \
  case MesiAgentLineState::__state: \
    return #__state;                \
    break;

    MESI_LINE_STATES(__to_str)
#undef __to_str
    default:
      return "Invalid State";
      break;
  }
}

const char* MesiDirectoryLineState::to_string(state_t state) {
  switch (state) {
#define __to_str(__state)               \
  case MesiDirectoryLineState::__state: \
    return #__state;                    \
    break;

    MESI_DIRECTORY_STATES(__to_str)
#undef __to_str
    default:
      return "Invalid State";
      break;
  }
}

struct MesiCoherentAgentModel::MesiCoherentAgentModelImpl {
  MesiCoherentAgentModelImpl(const CoherentAgentOptions& opts) : opts_(opts) {}

  void init(CacheLine& l) const { l.set_state(MesiAgentLineState::I); }

  bool is_stable(const CacheLine& cache_line) const {
    bool ret;
    switch (cache_line.state()) {
      case MesiAgentLineState::I:
      case MesiAgentLineState::S:
      case MesiAgentLineState::M:
      case MesiAgentLineState::E:
        ret = true;
        break;

      default:
        ret = false;
        break;
    }
    return ret;
  }

  std::string to_string(state_t s) const {
    return MesiAgentLineState::to_string(s);
  }

  //
  CoherenceActions get_actions(const Transaction* t,
                               const CacheLine& cache_line) const {
    CoherenceActions actions;
    switch (t->type()) {
      case TransactionType::Load:
        handle__Load(t, cache_line, actions);
        break;

      case TransactionType::Store:
        handle__Store(t, cache_line, actions);
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
        handle__FwdGetS(m, cache_line, actions);
        break;

      case MessageType::FwdGetM:
        handle__FwdGetM(m, cache_line, actions);
        break;

      case MessageType::Inv:
        handle__Inv(m, cache_line, actions);
        break;

      case MessageType::PutS:
      case MessageType::PutM:
        handle__PutAck(m, cache_line, actions);
        break;

      case MessageType::Data:
        handle__Data(m, cache_line, actions);
        break;

      default:
        actions.set_error(true);
    }
    return actions;
  }

 private:
  void handle__Load(const Transaction* t, const CacheLine& cache_line,
                    CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MesiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetS);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::IS_D);
        a.set_result(TransactionResult::Miss);
        break;

      case MesiAgentLineState::IS_D:
      case MesiAgentLineState::IM_AD:
      case MesiAgentLineState::IM_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MesiAgentLineState::S:
      case MesiAgentLineState::SM_AD:
      case MesiAgentLineState::SM_A:
      case MesiAgentLineState::M:
      case MesiAgentLineState::E:
        a.set_result(TransactionResult::Hit);
        break;

      case MesiAgentLineState::MI_A:
      case MesiAgentLineState::EI_A:
      case MesiAgentLineState::SI_A:
      case MesiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Store(const Transaction* t, const CacheLine& cache_line,
                     CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MesiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::IM_AD);
        a.set_result(TransactionResult::Miss);
        break;

      case MesiAgentLineState::IS_D:
      case MesiAgentLineState::IM_AD:
      case MesiAgentLineState::IM_A:
        a.set_result(TransactionResult::Miss);
        break;

      case MesiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::SM_AD);
        break;

      case MesiAgentLineState::SM_AD:
      case MesiAgentLineState::SM_A:
        a.set_result(TransactionResult::Miss);
        break;

      case MesiAgentLineState::E:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::M);
        [[fallthrough]];

      case MesiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        break;

      case MesiAgentLineState::MI_A:
      case MesiAgentLineState::EI_A:
      case MesiAgentLineState::SI_A:
      case MesiAgentLineState::II_A:
        a.set_result(TransactionResult::Miss);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__FwdGetS(const Message* m, const CacheLine& cache_line,
                       CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MesiAgentLineState::IM_AD:
      case MesiAgentLineState::IM_A:
      case MesiAgentLineState::SM_AD:
      case MesiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MesiAgentLineState::M:
      case MesiAgentLineState::E:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::S);
        a.set_result(MessageResult::Commit);
        break;

      case MesiAgentLineState::MI_A:
      case MesiAgentLineState::EI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::SI_A);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__FwdGetM(const Message* m, const CacheLine& cache_line,
                       CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MesiAgentLineState::IM_AD:
      case MesiAgentLineState::IM_A:
      case MesiAgentLineState::SM_AD:
      case MesiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MesiAgentLineState::M:
      case MesiAgentLineState::E:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;

      case MesiAgentLineState::MI_A:
      case MesiAgentLineState::EI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::II_A);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Inv(const Message* m, const CacheLine& cache_line,
                   CoherenceActions& a) const {
    if (m->is_ack()) {
      const bool is_last_ack = (cache_line.ack_count() == 1);

      a.append_command(CoherentAgentCommand::SetAckCount);
      a.set_ack_count(cache_line.ack_count() - 1);

      if (is_last_ack) {
        switch (cache_line.state()) {
          case MesiAgentLineState::IM_A:
          case MesiAgentLineState::SM_A:
            a.append_command(CoherentAgentCommand::UpdateState);
            a.set_next_state(MesiAgentLineState::M);
            a.set_result(MessageResult::Commit);
            break;

          default:
            a.set_error(true);
        }
      }
    } else {
      // Invalidation request
      //
      switch (cache_line.state()) {
        case MesiAgentLineState::IS_D:
          a.set_result(MessageResult::Stall);
          break;

        case MesiAgentLineState::S:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::I);
          a.set_result(MessageResult::Commit);
          break;

        case MesiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::IM_AD);
          a.set_result(MessageResult::Commit);
          break;

        case MesiAgentLineState::SI_A:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::II_A);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
          break;
      }
    }
  }

  void handle__PutAck(const Message* m, const CacheLine& cache_line,
                      CoherenceActions& a) const {
    switch (cache_line.state()) {
      case MesiAgentLineState::MI_A:
      case MesiAgentLineState::EI_A:
      case MesiAgentLineState::SI_A:
      case MesiAgentLineState::II_A:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MesiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Data(const Message* m, const CacheLine& cache_line,
                    CoherenceActions& a) const {
    const Platform platform = opts_.platform();
    const bool is_from_dir = platform.is_valid_snoop_filter_id(m->src_id());

    const bool is_exclusive_data_from_dir = is_from_dir && m->is_exclusive();

    const bool is_data_from_dir_ack_zero = is_from_dir && (m->ack_count() == 0);

    const bool is_data_from_dir_ack_non_zero =
        is_from_dir && (m->ack_count() != 0);

    const bool is_data_from_owner = !is_from_dir;

    if (is_exclusive_data_from_dir) {
      switch (cache_line.state()) {
        case MesiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::E);
          a.set_result(MessageResult::Commit);
          a.set_transaction_done(true);
          break;

        default:
          a.set_error(true);
          break;
      }

    } else if (is_data_from_dir_ack_zero || is_data_from_owner) {
      switch (cache_line.state()) {
        case MesiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::S);
          a.set_result(MessageResult::Commit);
          a.set_transaction_done(true);
          break;

        case MesiAgentLineState::IM_AD:
        case MesiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::M);
          a.set_result(MessageResult::Commit);
          a.set_transaction_done(true);
          break;

        default:
          a.set_error(true);
          break;
      }

    } else if (is_data_from_dir_ack_non_zero) {
      switch (cache_line.state()) {
        case MesiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::IM_A);
          a.set_result(MessageResult::Commit);
          break;

        case MesiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MesiAgentLineState::SM_A);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
          break;
      }
    }
  }

  const CoherentAgentOptions opts_;
};

MesiCoherentAgentModel::MesiCoherentAgentModel(const CoherentAgentOptions& opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MesiCoherentAgentModelImpl>(opts);
}

MesiCoherentAgentModel::~MesiCoherentAgentModel() {}

void MesiCoherentAgentModel::init(CacheLine& l) const { impl_->init(l); }

bool MesiCoherentAgentModel::is_stable(const CacheLine& l) const {
  return impl_->is_stable(l);
}

std::string MesiCoherentAgentModel::to_string(state_t state) const {
  return impl_->to_string(state);
}

CoherenceActions MesiCoherentAgentModel::get_actions(
    const Transaction* t, const CacheLine& cache_line) const {
  return impl_->get_actions(t, cache_line);
}

CoherenceActions MesiCoherentAgentModel::get_actions(
    const Message* m, const CacheLine& cache_line) const {
  return impl_->get_actions(m, cache_line);
}

struct MesiSnoopFilterModel::MesiSnoopFilterModelImpl {
  MesiSnoopFilterModelImpl(const SnoopFilterOptions& opts) : opts_(opts) {}

  void init(DirectoryEntry& l) const {
    l.set_state(
        static_cast<DirectoryEntry::state_type>(MesiDirectoryLineState::I));
  }

  bool is_stable(const DirectoryEntry& dir_entry) {
    bool ret{false};
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I:
      case MesiDirectoryLineState::S:
      case MesiDirectoryLineState::E:
      case MesiDirectoryLineState::M:
        ret = true;
        break;

      default:
        ret = false;
        break;
    }
    return ret;
  }

  std::string to_string(const DirectoryEntry& dir_entry) const {
    std::stringstream ss;
    ss << MesiDirectoryLineState::to_string(dir_entry.state());
    return ss.str();
  }

  std::string to_string(state_t state) const {
    return MesiDirectoryLineState::to_string(state);
  }

  CoherenceActions get_actions(const Message* m,
                               const DirectoryEntry& dir_entry) {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::GetS:
        handle__GetS(m, dir_entry, actions);
        break;

      case MessageType::GetM:
        handle__GetM(m, dir_entry, actions);
        break;

      case MessageType::PutS:
        handle__PutS(m, dir_entry, actions);
        break;

      case MessageType::PutM:
        handle__PutM(m, dir_entry, actions);
        break;

      case MessageType::PutE:
        handle__PutE(m, dir_entry, actions);
        break;

      case MessageType::Data:
        handle__Data(m, dir_entry, actions);
        break;

      default:
        actions.set_error(true);
    }
    return actions;
  }

 private:
  void handle__GetS(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_is_exclusive(true);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::E);
        break;

      case MesiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;

      case MesiDirectoryLineState::E:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.set_fwd_id(m->src_id());
        a.append_command(SnoopFilterCommand::AddOwnerToSharers);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::DelOwner);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::S_D);
        break;

      case MesiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.set_fwd_id(m->src_id());
        a.append_command(SnoopFilterCommand::AddOwnerToSharers);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::DelOwner);
        break;

      case MesiDirectoryLineState::S_D:
        // TODO: stall
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__GetM(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::M);
        break;

      case MesiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_ack_count(dir_entry.num_sharers_not_id(m->src_id()));
        a.append_command(SnoopFilterCommand::SendInvToSharers);
        a.append_command(SnoopFilterCommand::ClearSharers);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::M);
        break;

      case MesiDirectoryLineState::E:
        a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::M);
        break;

      case MesiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        break;

      case MesiDirectoryLineState::S_D:
        // TODO: Stall
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__PutS(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    const bool is_last = false;  // TODO

    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MesiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        if (is_last) {
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MesiDirectoryLineState::I);
        }
        break;

      case MesiDirectoryLineState::E:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MesiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MesiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__PutM(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I: {
        const bool is_from_owner = (m->src_id() == dir_entry.owner());

        if (!is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
      } break;

      case MesiDirectoryLineState::S: {
        const bool is_from_owner = (m->src_id() == dir_entry.owner());

        if (!is_from_owner) {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        }
      } break;

      case MesiDirectoryLineState::E: {
        const bool is_from_owner = (m->src_id() == dir_entry.owner());

        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MesiDirectoryLineState::I);
        }
      } break;

      case MesiDirectoryLineState::M: {
        const bool is_from_owner = (m->src_id() == dir_entry.owner());

        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MesiDirectoryLineState::I);
        }
      } break;

      case MesiDirectoryLineState::S_D: {
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
      } break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__PutE(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendPutEAckToReq);
        break;

      case MesiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutEAckToReq);
        break;

      case MesiDirectoryLineState::E: {
        const bool is_from_owner = (m->src_id() == dir_entry.owner());

        a.append_command(SnoopFilterCommand::SendPutEAckToReq);
        if (is_from_owner) {
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MesiDirectoryLineState::I);
        }
      } break;

      case MesiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutEAckToReq);
        break;

      case MesiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutEAckToReq);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Data(const Message* m, const DirectoryEntry& dir_entry,
                    CoherenceActions& a) const {
    switch (dir_entry.state()) {
      case MesiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::CpyDataToMemory);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MesiDirectoryLineState::S);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  const SnoopFilterOptions opts_;
};

MesiSnoopFilterModel::MesiSnoopFilterModel(const SnoopFilterOptions& opts)
    : SnoopFilterModel(opts) {
  impl_ = std::make_unique<MesiSnoopFilterModelImpl>(opts);
}

MesiSnoopFilterModel::~MesiSnoopFilterModel() {}

void MesiSnoopFilterModel::init(DirectoryEntry& l) const { impl_->init(l); }

bool MesiSnoopFilterModel::is_stable(const DirectoryEntry& l) const {
  return impl_->is_stable(l);
}

std::string MesiSnoopFilterModel::to_string(const DirectoryEntry& l) const {
  return impl_->to_string(l);
}

std::string MesiSnoopFilterModel::to_string(state_t state) const {
  return impl_->to_string(state);
}

CoherenceActions MesiSnoopFilterModel::get_actions(
    const Message* m, const DirectoryEntry& dir_entry) const {
  return impl_->get_actions(m, dir_entry);
}

MesiCoherenceProtocolValidator::MesiCoherenceProtocolValidator() {}

bool MesiCoherenceProtocolValidator::validate_addr(
    addr_t addr, const std::vector<Entry<CacheLine> >& lines,
    const DirectoryEntry& entry) const {
  return false;
}

}  // namespace ccm
