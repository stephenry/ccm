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

namespace ccm {

namespace {

MosiAgentLineState _s(CacheLine::state_type s) {
  return static_cast<MosiAgentLineState>(s);
}

} // namespace

struct MosiCoherentAgentModel::MosiCoherentAgentModelImpl {
  MosiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts)
  {}

  CoherenceActions get_actions(
      const Transaction * t, const CacheLine & cache_line) const {
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

  CoherenceActions get_actions(
      const Message * m, const CacheLine & cache_line) const {
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
        break;
    }
    return actions;
  }

  void handle__Load(
      const Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
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
        break;
        
      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::O:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
        a.set_result(TransactionResult::Hit);
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
  
  void handle__Store(
      const Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
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
        a.set_next_state(MosiAgentLineState::SI_A);
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        break;

      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OM_AC);
        a.set_result(TransactionResult::Hit);
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

  void handle__FwdGetS(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
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
  

  void handle__FwdGetM(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
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

  void handle__Inv(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    if (m->is_ack()) {
      const bool is_last_ack = (cache_line.ack_count() == 1);

      a.append_command(CoherentAgentCommand::SetAckCount);
      a.set_ack_count(cache_line.ack_count() - 1);

      if (is_last_ack) {
      
        switch (_s(cache_line.state())) {
          case MosiAgentLineState::IM_A:
          case MosiAgentLineState::SM_A:
          case MosiAgentLineState::OM_A:
            a.append_command(CoherentAgentCommand::UpdateState);
            a.set_next_state(MosiAgentLineState::M);
            break;

          default:
            a.set_error(true);
            break;
        }

      }      

    } else {
      // Invalidation request
      switch (_s(cache_line.state())) {
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
  
  void handle__PutAck(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
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

  void handle__Data(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    const bool is_data_from_dir_ack_zero = false;
    const bool is_data_from_dir_ack_non_zero = false;
    const bool is_data_from_owner = false;

    if (is_data_from_dir_ack_zero || is_data_from_owner) {

      switch (_s(cache_line.state())) {
        case MosiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::S);
          break;
          
        case MosiAgentLineState::IM_AD:
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::M);
          break;
          
        default:
          a.set_error(true);
          break;
      }
      
    } else if (is_data_from_dir_ack_non_zero) {

      switch (_s(cache_line.state())) {
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
  
  const CoherentAgentOptions opts_;
};

MosiCoherentAgentModel::MosiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
}

MosiCoherentAgentModel::~MosiCoherentAgentModel() {
}

void MosiCoherentAgentModel::init(CacheLine & l) const {
}

bool MosiCoherentAgentModel::is_stable(const CacheLine & l) const {
  return false;
}

std::string MosiCoherentAgentModel::to_string(CacheLine::state_type s) const {
  return "";
}
  
//
CoherenceActions MosiCoherentAgentModel::get_actions(
    const Transaction * t, const CacheLine & cache_line) const {
  return {};
}

CoherenceActions MosiCoherentAgentModel::get_actions(
    const Message * m, const CacheLine & cache_line) const {
  return {};
}

MosiSnoopFilterModel::MosiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
}

MosiSnoopFilterModel::~MosiSnoopFilterModel() {
}

void MosiSnoopFilterModel::init(DirectoryEntry & l) const {
}

bool MosiSnoopFilterModel::is_stable(const DirectoryEntry & l) const {
  return false;
}

std::string MosiSnoopFilterModel::to_string(const DirectoryEntry & l) const {
  return "";
}

std::string MosiSnoopFilterModel::to_string(CacheLine::state_type l) const {
  return "";
}

CoherenceActions MosiSnoopFilterModel::get_actions(
    const Message * m, const DirectoryEntry & dir_entry) const {
  return {};
}

} // namespace ccm
