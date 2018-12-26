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

#include "msi.hpp"
#include "message.hpp"
#include "transaction.hpp"
#include "common.hpp"
#include <algorithm>

namespace ccm {

#define LINE_STATES(__func)                     \
  __func(I)                                     \
  __func(IS_D)                                  \
  __func(IM_AD)                                 \
  __func(IM_A)                                  \
  __func(S)                                     \
  __func(SM_AD)                                 \
  __func(SM_A)                                  \
  __func(M)                                     \
  __func(MI_A)                                  \
  __func(SI_A)                                  \
  __func(II_A)

enum class MsiAgentLineState : uint8_t {
#define __declare_state(__state)                \
  __state,
  LINE_STATES(__declare_state)
#undef __declare_state
};

const char * to_string(MsiAgentLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiAgentLineState::__e: return #__e;
    LINE_STATES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
}

#define MSI_HANDLER_EMIT_CLASSES(__func)        \
  __func(Nop)                                   \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitInvAck)                            \
  __func(EmitDataReq)                           \
  __func(EmitDataDir)                           \
  __func(EmitFwdGetSToOwner)

enum class MsiHandlerEmitType {
#define __declare_enum(e) e,
  MSI_HANDLER_EMIT_CLASSES(__declare_enum)
#undef __declare_enum
};

const char * to_string(MsiHandlerEmitType t) {
  switch (t) {
#define __declare_to_string(__e)                \
    case MsiHandlerEmitType::__e: return #__e;
    MSI_HANDLER_EMIT_CLASSES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
}

template<typename ACTION, typename STATE>
struct MsiHandlerAction {

  bool has_state_update() const { return next_state_.has_value(); }
  STATE next_state() const { return next_state_.value(); }
  void set_next_state(STATE state) { next_state_ = state; }

  //
  bool error() const { return false; }
  void set_error() {}

  //
  bool stall() const { return stall_; }
  void set_stall() { stall_ = true; }

  std::size_t message_count() const { return msgs_n_; }

  const std::vector<ACTION> & actions() const { return actions_; }
  void add_action(const ACTION & a) {
    actions_.push_back(a);
  }

 private:
  bool stall_{false};
  std::optional<STATE> next_state_;
  std::vector<ACTION> actions_;
  std::size_t msgs_n_{0};
};

#define AGENT_ACTION_STATES(__func)             \
  __func(UpdateState)                           \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitDataToReq)                         \
  __func(EmitDataToDir)                         \
  __func(EmitInvAck)

enum class MsiCoherentAgentAction {
#define __declare_state(__state)                \
  __state,
  AGENT_ACTION_STATES(__declare_state)
#undef __declare_state
};

const char * to_string(MsiCoherentAgentAction t) {
  switch (t) {
#define __declare_to_string(__e)                \
    case MsiCoherentAgentAction::__e: return #__e;
    AGENT_ACTION_STATES(__declare_to_string)
#undef __declare_to_string
    default:
        return "<Invalid Line State>";
  }
}

struct MsiCoherentAgentModel::MsiCoherentAgentModelImpl {
  using ActionType = MsiHandlerAction<MsiCoherentAgentAction, MsiAgentLineState>;

  struct LineEntry {
    LineEntry() : state_(MsiAgentLineState::I) {}
    
    MsiAgentLineState state() const { return state_; }
    void set_state(MsiAgentLineState state) { state_ = state; }
   private:
    MsiAgentLineState state_;
  };
  
  MsiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}

  CoherentActorResult apply(const Transaction * t) {
    ActionType action;
    LineEntry line_entry;
    switch (t->type()) {
      case TransactionType::Load:
        handle__Load(t, line_entry, action);
        break;
        
      case TransactionType::Store:
        handle__Store(t, line_entry, action);
        break;
    }
    return {};
  }
  
  CoherentActorResult apply(const Message * m) {
    CoherentActorResult ret;
    ActionType actions;
    LineEntry line_entry;
    switch (m->type()) {
      case MessageType::FwdGetS:
        handle__FwdGetS(m, line_entry, actions);
        break;

      case MessageType::FwdGetM:
        handle__FwdGetM(m, line_entry, actions);
        break;

      case MessageType::Inv:
        handle__Inv(m, line_entry, actions);
        break;

      case MessageType::PutS:
      case MessageType::PutM:
        handle__PutAck(m, line_entry, actions);
        break;

      case MessageType::Data:
        handle__Data(m, line_entry, actions);
        break;

      default:
        actions.set_error();
    }
    return ret;
  }

  bool message_requires_eviction(const Message * m) {
    return false;
  }

  void handle__Load(const Transaction * t, const LineEntry & line_entry, ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(MsiCoherentAgentAction::EmitGetS);
        a.set_next_state(MsiAgentLineState::IS_D);
        [[fallthrough]];
        
      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_stall();
        break;

      case MsiAgentLineState::S:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::M:
        //        a.response = ResponseType::Hit;
        break;
        
      default:
        a.set_error();
    }
  }
  
  void handle__Store(const Transaction * t, const LineEntry & line_entry, ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(MsiCoherentAgentAction::EmitGetM);
        a.set_next_state(MsiAgentLineState::IM_AD);
        [[fallthrough]];

      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::S:
        a.add_action(MsiCoherentAgentAction::EmitGetM);
        a.set_next_state(MsiAgentLineState::SM_AD);
        a.set_stall();
        break;
                     
      case MsiAgentLineState::M:
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__FwdGetS(const Message * m, const LineEntry & line_entry, ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::M:
        a.add_action(MsiCoherentAgentAction::EmitDataToReq);
        a.add_action(MsiCoherentAgentAction::EmitDataToDir);
        a.set_next_state(MsiAgentLineState::S);
        break;
        
      case MsiAgentLineState::MI_A:
        a.add_action(MsiCoherentAgentAction::EmitDataToReq);
        a.add_action(MsiCoherentAgentAction::EmitDataToDir);
        a.set_next_state(MsiAgentLineState::SI_A);
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__FwdGetM(const Message * m, const LineEntry & line_entry, ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::M:
        a.add_action(MsiCoherentAgentAction::EmitDataToReq);
        a.set_next_state(MsiAgentLineState::S);
        break;
        
      case MsiAgentLineState::MI_A:
        a.add_action(MsiCoherentAgentAction::EmitDataToReq);
        a.set_next_state(MsiAgentLineState::SI_A);
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__Inv(const Message *m, const LineEntry & line_entry, ActionType & a) {
    if (m->is_ack()) {
      // Line awaits invalidation acknowledgements from other agents
      // before upgrade to modified state.
      //
      // TODO: retain this on a line-by-line basis.
      const bool is_last_ack = false;

      if (is_last_ack) {
        switch (line_entry.state()) {
          case MsiAgentLineState::IM_A:
          case MsiAgentLineState::SM_A:
            a.set_next_state(MsiAgentLineState::M);
            break;
            
          default:
            a.set_error();
        }
      }
      
    } else {
      // Inbound invalidation request from the home directory. Update
      // line state accordingly.
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IS_D:
          a.set_stall();
          break;
          
        case MsiAgentLineState::S:
          a.add_action(MsiCoherentAgentAction::EmitInvAck);
          a.set_next_state(MsiAgentLineState::I);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.add_action(MsiCoherentAgentAction::EmitInvAck);
          a.set_next_state(MsiAgentLineState::IM_AD);
          break;
          
        case MsiAgentLineState::SI_A:
          a.add_action(MsiCoherentAgentAction::EmitInvAck);
          a.set_next_state(MsiAgentLineState::II_A);
          break;
          
        default:
          a.set_error();
      }
    }
  }

  void handle__PutAck(const Message * m, const LineEntry & line_entry, ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_next_state(MsiAgentLineState::I);
        break;
      default:
        a.set_error();
    }
  }

  void handle__Data(const Message * m, const LineEntry & line_entry, ActionType & a) {

    // TODO: retain this on a line-by-line basis.

    bool is_from_dir = false;
    bool is_last_ack = false;

    if (is_last_ack) {

      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).n
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IS_D:
          a.set_next_state(MsiAgentLineState::S);
          break;
          
        case MsiAgentLineState::IM_AD:
          a.set_next_state(MsiAgentLineState::M);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.set_next_state(MsiAgentLineState::M);
          break;
          
        default:
          a.set_error();
      }
    } else {

      // Data has been received, but we are currently awaiting
      // pending responses from other agents in the system.
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IM_AD:
          a.set_next_state(MsiAgentLineState::IM_A);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.set_next_state(MsiAgentLineState::SM_A);
          break;
          
        default:
          a.set_error();
      }
    }
  }

  CacheModel<MsiAgentLineState> cache_;
  CoherentAgentOptions opts_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

CoherentActorResult MsiCoherentAgentModel::apply(const Transaction * t) {
  return impl_->apply(t);
}

CoherentActorResult MsiCoherentAgentModel::apply(const Message * m) {
  return impl_->apply(m);
}

#define DIRECTORY_STATES(__func)                \
  __func(I)                                     \
  __func(S)                                     \
  __func(M)                                     \
  __func(S_D)

enum class MsiDirectoryLineState {
#define __declare_state(__state)                \
  __state,
  DIRECTORY_STATES(__declare_state)
#undef __declare_state
};

const char * to_string(MsiDirectoryLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiDirectoryLineState::__e: return #__e;
    DIRECTORY_STATES(__declare_to_string)
#undef __declare_to_string
  }
  return "<Invalid Directory State>";
}

#define SNOOP_FILTER_ACTION_STATES(__func)      \
  __func(UpdateState)                           \
  __func(SetOwnerToReq)                         \
  __func(SendDataToReq)                         \
  __func(SendInvToSharers)                      \
  __func(ClearSharers)                          \
  __func(AddReqToSharers)                       \
  __func(DelReqFromSharers)                     \
  __func(DelOwner)                              \
  __func(AddOwnerToSharers)                     \
  __func(CpyDataToMemory)                       \
  __func(SendPutSAckToReq)                      \
  __func(SendPutMAckToReq)                      \
  __func(SendFwdGetSToOwner)


enum class MsiSnoopFilterHandlerAction {
#define __declare_state(__state)                \
  __state,
  SNOOP_FILTER_ACTION_STATES(__declare_state)
#undef __declare_state
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelImpl {
  using ActionType = MsiHandlerAction<MsiSnoopFilterHandlerAction, MsiDirectoryLineState>;

  struct DirEntry {

    DirEntry() : state_(MsiDirectoryLineState::I) {}

    //
    MsiDirectoryLineState state() const { return state_; }
    void set_state(MsiDirectoryLineState state) { state_ = state; }
    
    //
    std::size_t owner() const { return owner_.value(); }
    void set_owner(std::size_t owner) { owner_ = owner; }
    void clear_owner() { owner_.reset(); }

    //
    const std::vector<std::size_t> & sharers() const { return sharers_; }
    void add_sharer(std::size_t id) { sharers_.push_back(id); }
    void remove_sharer(std::size_t id) {
      sharers_.erase(std::find(sharers_.begin(), sharers_.end(), id), sharers_.end());
    }
    void remove_all_sharers() { sharers_.clear(); }

   private:
    // Current state of the line
    MsiDirectoryLineState state_;

    // Current set of sharing agents
    std::vector<std::size_t> sharers_;

    // Current owner agent
    std::optional<std::size_t> owner_;
  };
  
  MsiSnoopFilterModelImpl(const SnoopFilterOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}
  
  CoherentActorResult apply(const Message * m) {
    CoherentActorResult ret;
    if (message_requires_recall(m)) {

    } else {

      CCM_ASSERT(cache_.is_hit(m->addr()));

      DirEntry & dir_entry = cache_.entry(m->addr());

      ActionType actions;
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
          
        case MessageType::Data:
          handle__Data(m, dir_entry, actions);
          break;
          
        default:
          actions.set_error();
      }
      CCM_ASSERT(!actions.error());

      if (!actions.stall()) {
        ret.set_status(CoherentActorResultStatus::BlockedOnProtocol);
        return ret;
      }

      ret.set_status(CoherentActorResultStatus::Advances);
    }
    return ret;
  }
 private:
  
  void handle__GetS(const Message * m, const DirEntry & dir_entry, ActionType & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        a.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        a.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        break;
      case MsiDirectoryLineState::M:
        a.add_action(MsiSnoopFilterHandlerAction::SendFwdGetSToOwner);
        a.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        a.add_action(MsiSnoopFilterHandlerAction::AddOwnerToSharers);
        a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S_D:
        a.set_stall();
        break;
      default:
        a.set_error();
    }
  }

  void handle__GetM(const Message * m, const DirEntry & dir_entry, ActionType & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        a.add_action(MsiSnoopFilterHandlerAction::SetOwnerToReq);
        a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        a.add_action(MsiSnoopFilterHandlerAction::SendInvToSharers);
        a.add_action(MsiSnoopFilterHandlerAction::ClearSharers);
        a.add_action(MsiSnoopFilterHandlerAction::SetOwnerToReq);
        a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::M:
        break;
      case MsiDirectoryLineState::S_D:
        a.set_stall();
        break;
      default:
        a.set_error();
    }
  }

  void handle__PutS(const Message * m, const DirEntry & dir_entry, ActionType & a) const {
    const bool is_last = false; // TODO
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        a.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        if (is_last) {
          a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::M:
        a.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        a.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S_D:
        a.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        a.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      default:
        a.set_error();
    }
  }

  void handle__PutM(const Message * m, const DirEntry & dir_entry, ActionType & a) const {
    const bool is_data_from_owner = false;
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        if (!is_data_from_owner) {
          a.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::S:
        if (!is_data_from_owner) {
          a.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
          a.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::M:
        a.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        if (is_data_from_owner) {
          a.add_action(MsiSnoopFilterHandlerAction::DelOwner);
          a.add_action(MsiSnoopFilterHandlerAction::CpyDataToMemory);
          a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::S_D:
        if (!is_data_from_owner) {
          a.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
          a.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      default:
        a.set_error();
    }
  }

  void handle__Data(const Message * m, const DirEntry & dir_entry, ActionType & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::S_D:
        a.add_action(MsiSnoopFilterHandlerAction::CpyDataToMemory);
        a.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      default:
        a.set_error();
    }
  }

  bool message_requires_recall(const Message * m) {
    return false;
  }

  bool transaction_must_hit(MessageType t) {
    return true;
  }
  
  SnoopFilterOptions opts_;
  CacheModel<DirEntry> cache_;
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelNullFilterImpl
    : MsiSnoopFilterModel::MsiSnoopFilterModelImpl {

  MsiSnoopFilterModelNullFilterImpl(const SnoopFilterOptions & opts)
      : MsiSnoopFilterModelImpl(opts)
  {}
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelDirectoryImpl
    : MsiSnoopFilterModel::MsiSnoopFilterModelImpl {

  MsiSnoopFilterModelDirectoryImpl(const SnoopFilterOptions & opts)
      : MsiSnoopFilterModelImpl(opts)
  {}
};

MsiSnoopFilterModel::MsiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
  impl_ = std::make_unique<MsiSnoopFilterModelDirectoryImpl>(opts);
}

MsiSnoopFilterModel::~MsiSnoopFilterModel() {}

CoherentActorResult MsiSnoopFilterModel::apply(const Message * m) {
  return impl_->apply(m);
}

} // namespace ccm
