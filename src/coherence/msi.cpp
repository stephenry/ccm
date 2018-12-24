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
struct MsiHandlerAction : SnoopFilterAction {

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

  auto actions() const { return actions_; }
  void add_action(const ACTION & a) {
    actions_.push_back(a);
  }

 private:
  bool stall_{false};
  std::optional<STATE> next_state_;
  std::vector<ACTION> actions_;
  std::size_t msgs_n_{0};
};

struct MsiAgentHandlerAction : public CoherentAgentAction {
  std::vector<MsiHandlerEmitType> handler_emit;
  std::optional<MsiAgentLineState> state_update;
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
  
  CoherentAgentAction apply(CoherentAgentContext & ctxt, CoherencyMessage * m) {
    CoherentAgentAction ret;

    if (message_requires_eviction(m)) {
      // TODO: require eviction mechanism
      ret.response = ResponseType::Stall;
    } else {
      // Dispatch to handler


      ActionType actions;
      LineEntry line_entry;
      switch (m->type()) {

        case MessageType::Load:
          handle__Load(static_cast<LoadCoherencyMessage *>(m), line_entry, actions);
          break;

        case MessageType::Store:
          handle__Store(static_cast<StoreCoherencyMessage *>(m), line_entry, actions);
          break;

        case MessageType::FwdGetS:
          handle__FwdGetS(static_cast<FwdGetSCoherencyMessage * >(m), line_entry, actions);
          break;

        case MessageType::FwdGetM:
          handle__FwdGetM(static_cast<FwdGetMCoherencyMessage * >(m), line_entry, actions);
          break;

        case MessageType::Inv:
          handle__Inv(static_cast<InvCoherencyMessage * >(m), line_entry, actions);
          break;

        case MessageType::PutS:
        case MessageType::PutM:
          handle__PutAck(line_entry, actions);
          break;

        case MessageType::Data:
          handle__Data(static_cast<DataCoherencyMessage *>(m), line_entry, actions);
          break;

        default:
          actions.set_error();
      }

      commit_actions(m, actions, line_entry, ret);
    }
    
    return ret;
  }

  bool message_requires_eviction(CoherencyMessage * m) {
    bool ret = false;
    switch (m->type()) {
      case MessageType::Load:
      case MessageType::Store:
        ret = cache_.requires_eviction(m->addr());
        break;
      default:
        ret = false;
    }
    return ret;
  }

  void handle__Load(const LoadCoherencyMessage * m, const LineEntry & line_entry,
                    ActionType & a) {

    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(MsiCoherentAgentAction::EmitGetS);
        a.set_next_state(MsiAgentLineState::IS_D);
        
        // Fallthrough
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
  
  void handle__Store(const StoreCoherencyMessage * m, const LineEntry & line_entry,
                     ActionType & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(MsiCoherentAgentAction::EmitGetM);
        a.set_next_state(MsiAgentLineState::IM_AD);

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

  void handle__FwdGetS(const FwdGetSCoherencyMessage * m, const LineEntry & line_entry,
                       ActionType & a) {
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

  void handle__FwdGetM(const FwdGetMCoherencyMessage * m, const LineEntry & line_entry,
                       ActionType & a) {
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

  void handle__Inv(const InvCoherencyMessage *m, const LineEntry & line_entry,
                   ActionType & a) {
    
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

  void handle__PutAck(const LineEntry & line_entry, ActionType & a) {
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

  void handle__Data(const DataCoherencyMessage * m, const LineEntry & line_entry, ActionType & a) {

    // TODO: retain this on a line-by-line basis.

    bool is_from_dir = false;
    bool is_last_ack = false;

    if (is_last_ack) {

      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).
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

  void commit_actions(const CoherencyMessage * m, const ActionType & a, LineEntry & line_entry,
                      CoherentAgentAction & ret) {

    for (const MsiCoherentAgentAction action : a.actions()) {
      switch (action) {
        case MsiCoherentAgentAction::UpdateState:
          CCM_ASSERT(a.has_state_update());
          line_entry.set_state(a.next_state());
          break;
            
        case MsiCoherentAgentAction::EmitGetS:
          ret.add_msg(construct_gets());
          break;
            
        case MsiCoherentAgentAction::EmitGetM:
          ret.add_msg(construct_getm());
          break;
            
        case MsiCoherentAgentAction::EmitDataToReq:
          ret.add_msg(construct_datareq());
          break;
            
        case MsiCoherentAgentAction::EmitDataToDir:
          ret.add_msg(construct_datadir());
          break;
            
        case MsiCoherentAgentAction::EmitInvAck:
          ret.add_msg(construct_invack());
          break;
      }
    }
  }

  GetSCoherencyMessage * construct_gets() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
      
    GetSCoherencyMessageBuilder b = gets_.builder();
    //    b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  GetMCoherencyMessage * construct_getm() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    GetMCoherencyMessageBuilder b = getm_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  InvCoherencyMessage * construct_invack() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    InvCoherencyMessageBuilder b = inv_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    b.set_is_ack();
    return b.msg();
  }

  DataCoherencyMessage * construct_datareq() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    DataCoherencyMessageBuilder b = data_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  DataCoherencyMessage * construct_datadir() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    DataCoherencyMessageBuilder b = data_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  CacheModel<MsiAgentLineState> cache_;
  CoherentAgentOptions opts_;
  GetSCoherencyMessageDirector gets_;
  GetMCoherencyMessageDirector getm_;
  InvCoherencyMessageDirector inv_;
  DataCoherencyMessageDirector data_;
  IdPool idpool_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

CoherentAgentAction MsiCoherentAgentModel::apply(CoherentAgentContext & ctxt,
                                                 CoherencyMessage * m) {
  return impl_->apply(ctxt, m);
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
    auto sharers() const { return sharers_; }
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
  
  SnoopFilterAction apply(CoherentAgentContext & ctxt, CoherencyMessage * m) {
    SnoopFilterAction ret;
    if (message_requires_recall(m)) {

    } else {

      CCM_ASSERT(cache_.is_hit(m->addr()));

      DirEntry & dir_entry = cache_.entry(m->addr());

      ActionType actions;
      switch (m->type()) {
        case MessageType::GetS:
          handle__GetS(static_cast<GetSCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::GetM:
          handle__GetM(static_cast<GetMCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::PutS:
          handle__PutS(static_cast<PutSCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::PutM:
          handle__PutM(static_cast<PutMCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::Data:
          handle__Data(static_cast<DataCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        default:
          actions.set_error();
      }
      CCM_ASSERT(!actions.error());

      if (!actions.stall()) {
        ret.set_result(SnoopFilterActionResult::BlockedOnProtocol);
        return ret;
      }

      if (!idpool_.available(actions.message_count())) {
        ret.set_result(SnoopFilterActionResult::TagsExhausted);
        return ret;
      }

      commit_actions(m, actions, dir_entry, ret);
      ret.set_result(SnoopFilterActionResult::Advances);
    }
    return ret;
  }
 private:
  
  void handle__GetS(const GetSCoherencyMessage * m, const DirEntry & dir_entry,
                    ActionType & a) const {
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

  void handle__GetM(const GetMCoherencyMessage * m, const DirEntry & dir_entry,
                    ActionType & a) const {
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

  void handle__PutS(const PutSCoherencyMessage * m, const DirEntry & dir_entry,
                    ActionType & a) const {
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

  void handle__PutM(const PutMCoherencyMessage * m, const DirEntry & dir_entry, 
                    ActionType & a) const {
    const bool is_data_from_owner = (m->mid() == dir_entry.owner());
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

  void handle__Data(const DataCoherencyMessage * m, const DirEntry & dir_entry,
                    ActionType & a) const {
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

  bool message_requires_recall(CoherencyMessage * m) {
    if (opts_.is_null_filter)
      return false;

    // TODO: Directory eviction code.
    
    return false;
  }

  void commit_actions(const CoherencyMessage * m, const ActionType & a,
                      DirEntry & dir_entry, SnoopFilterAction & ret) {
    for (MsiSnoopFilterHandlerAction action : a.actions()) {
      
      switch (action) {
        case MsiSnoopFilterHandlerAction::UpdateState:
          CCM_ASSERT(a.has_state_update());
          
          dir_entry.set_state(a.next_state());
          break;
          
        case MsiSnoopFilterHandlerAction::SetOwnerToReq:
          dir_entry.set_owner(m->mid());
          break;
          
        case MsiSnoopFilterHandlerAction::SendDataToReq:
          ret.add_msg(construct_Data(m->mid()));
          break;
          
        case MsiSnoopFilterHandlerAction::SendInvToSharers:
          for (std::size_t sharer_id : dir_entry.sharers())
            ret.add_msg(construct_Inv(sharer_id));
          break;
          
        case MsiSnoopFilterHandlerAction::ClearSharers:
          dir_entry.remove_all_sharers();
          break;
          
        case MsiSnoopFilterHandlerAction::AddReqToSharers:
          dir_entry.add_sharer(m->mid());
          break;
          
        case MsiSnoopFilterHandlerAction::DelReqFromSharers:
          dir_entry.remove_sharer(m->mid());
          break;
          
        case MsiSnoopFilterHandlerAction::DelOwner:
          dir_entry.clear_owner();
          break;
          
        case MsiSnoopFilterHandlerAction::AddOwnerToSharers:
          dir_entry.add_sharer(dir_entry.owner());
          break;
          
        case MsiSnoopFilterHandlerAction::CpyDataToMemory:
          // TODO
          break;
          
        case MsiSnoopFilterHandlerAction::SendPutSAckToReq:
          ret.add_msg(construct_PutSAck(m->mid()));
          break;
          
        case MsiSnoopFilterHandlerAction::SendPutMAckToReq:
          ret.add_msg(construct_PutMAck(m->mid()));
          break;
          
        case MsiSnoopFilterHandlerAction::SendFwdGetSToOwner:
          ret.add_msg(construct_FwdGetS(dir_entry.owner()));
          break;
      }
    }
  }

  FwdGetSCoherencyMessage * construct_FwdGetS(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    FwdGetSCoherencyMessageBuilder b = fwdgets_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  FwdGetMCoherencyMessage * construct_FwdGetM(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    FwdGetMCoherencyMessageBuilder b = fwdgetm_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  PutSCoherencyMessage * construct_PutSAck(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    PutSCoherencyMessageBuilder b = puts_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  PutMCoherencyMessage * construct_PutMAck(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    PutMCoherencyMessageBuilder b = putm_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  DataCoherencyMessage * construct_Data(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    DataCoherencyMessageBuilder b = data_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  InvCoherencyMessage * construct_Inv(std::size_t dest_id) {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    InvCoherencyMessageBuilder b = inv_.builder();
    b.set_tid(tid);
    return b.msg();
  }

  bool transaction_must_hit(MessageType t) {
    return true;
  }
  
  SnoopFilterOptions opts_;
  CacheModel<DirEntry> cache_;
  FwdGetSCoherencyMessageDirector fwdgets_;
  FwdGetMCoherencyMessageDirector fwdgetm_;
  PutSCoherencyMessageDirector puts_;
  PutMCoherencyMessageDirector putm_;
  InvCoherencyMessageDirector inv_;
  DataCoherencyMessageDirector data_;
  IdPool idpool_;
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
  if (opts.is_null_filter) {
    // TODO: not presently supported by the protocol
    impl_ = std::make_unique<MsiSnoopFilterModelNullFilterImpl>(opts);
  } else {
    impl_ = std::make_unique<MsiSnoopFilterModelDirectoryImpl>(opts);
  }
}

MsiSnoopFilterModel::~MsiSnoopFilterModel() {}

SnoopFilterAction MsiSnoopFilterModel::apply(CoherentAgentContext & ctxt,
                                             CoherencyMessage * m) {
  return impl_->apply(ctxt, m);
}

} // namespace ccm
