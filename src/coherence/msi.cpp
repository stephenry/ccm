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
  }
  return "<Invalid Line State>";
}

struct MsiCoherentAgentModel::MsiCoherentAgentModelImpl {
  MsiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}
  
  CoherentAgentAction apply(CoherencyMessage * m) {
    CoherentAgentAction a;
    // Dispatch to handler
    switch (m->type()) {
#define __declare_dispatcher(e)                                         \
      case MessageType::e:                                              \
        handle__ ## e(static_cast<e ## CoherencyMessage *>(m), a);

      MESSAGE_CLASSES(__declare_dispatcher)
#undef __declare_dispatcher
      default:
          // TOOD: unknown message class.
          ;
     }
    // Return transaction to owner pool.
    m->release();
    return a;
  }

  void handle__Load(LoadCoherencyMessage * m, CoherentAgentAction & a) {
    MsiAgentLineState s;
    if (cache_.is_hit(m->addr(), s)) {
      switch (s) {
        case MsiAgentLineState::IS_D:
        case MsiAgentLineState::IM_AD:
        case MsiAgentLineState::IM_A:
        case MsiAgentLineState::MI_A:
        case MsiAgentLineState::SI_A:
        case MsiAgentLineState::II_A:
          a.resp = ResponseType::Stall;
          break;

        case MsiAgentLineState::S:
        case MsiAgentLineState::SM_AD:
        case MsiAgentLineState::SM_A:
        case MsiAgentLineState::M:
          a.resp = ResponseType::Hit;
          break;

        case MsiAgentLineState::I:
          ; // TODO: Invalid state
      }
    } else {
      // Miss, emit GetS if maximum in-flight count has not yet been reached.

      if (!cache_.requires_eviction(m->addr())) {
        std::size_t tid;
        if (ids_.get_id(tid)) {
          GetSCoherencyMessageBuilder b = gets_.builder();
          b.set_addr(m->addr());
          b.set_tid(tid);

          a.resp = ResponseType::Stall;
          a.msg = b.msg();
        } else {
          // TODO: When ID pool has been exhasuted, fill cannot be issued to
          // interconnect.
          a.resp = ResponseType::Blocked;
        }
        cache_.update(m->addr(), MsiAgentLineState::IS_D);
      } else {
        // TODO: Cache eviction if necessary
      }
    }
  }
  
  void handle__Store(StoreCoherencyMessage * m, CoherentAgentAction & a) {
    MsiAgentLineState s;
    if (cache_.is_hit(m->addr(), s)) {
      switch (s) {
        case MsiAgentLineState::IS_D:
        case MsiAgentLineState::IM_AD:
        case MsiAgentLineState::IM_A:
        case MsiAgentLineState::SM_AD:
        case MsiAgentLineState::SM_A:
        case MsiAgentLineState::MI_A:
        case MsiAgentLineState::SI_A:
        case MsiAgentLineState::II_A:
          a.resp = ResponseType::Stall;
          break;
          
        case MsiAgentLineState::S: {
          // Transaction requires line promotion to M before operation may complete.
          //
          GetMCoherencyMessageBuilder b = getm_.builder();
          b.set_addr(m->addr());
          b.set_tid(tid);
          
          a.resp = ResponseType::Stall;
          a.msg = b.msg();
          cache_.update(m->addr(), MsiAgentLineState::SM_AD);
        } break;

        case MsiAgentLineState::M:
          a.resp = ResponseType::Hit;
          break;

        case MsiAgentLineState::I:; // TODO: Invalid state
      }
    } else {
      // Miss, emit GetM if maximum in-flight count has not yet been reached.
      
      std::size_t tid;
      if (ids_.get_id(tid)) {
        GetMCoherencyMessageBuilder b = getm_.builder();
        b.set_addr(m->addr());
        b.set_tid(tid);
        
        a.resp = ResponseType::Stall;
        a.msg = b.msg();
        cache_.update(m->addr(), MsiAgentLineState::IM_AD);
      } else {
        a.resp = ResponseType::Blocked;
      }
    }
  }
  
  void handle__Replacement(ReplacementCoherencyMessage * m, CoherentAgentAction & a) {
                           
  }
  
  void handle__FwdGetS(FwdGetSCoherencyMessage * m, CoherentAgentAction & a) {
                       
  }

  // Never Handled
  void handle__GetS(GetSCoherencyMessage * m, CoherentAgentAction & a) {
  }
  void handle__GetM(GetMCoherencyMessage * m, CoherentAgentAction & a) {
  }
  
  void handle__FwdGetM(FwdGetMCoherencyMessage * m, CoherentAgentAction & a) {
                       
  }
  
  void handle__Inv(InvCoherencyMessage * m, CoherentAgentAction & a) {
                   
  }
  
  void handle__PutAck(PutAckCoherencyMessage * m, CoherentAgentAction & a) {
                      
  }
  
  void handle__DataDir(DataDirCoherencyMessage * m, CoherentAgentAction & a) {
                       
  }
  
  void handle__DataOwner(DataOwnerCoherencyMessage * m, CoherentAgentAction & a) {
                         
  }
  
  void handle__InvAck(InvAckCoherencyMessage * m, CoherentAgentAction & a) {
                      
  }

  GenericCacheModel<MsiAgentLineState> cache_;
  CoherentAgentOptions opts_;
  GetSCoherencyMessageDirector gets_;
  GetMCoherencyMessageDirector getm_;
  IdPool ids_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

CoherentAgentAction MsiCoherentAgentModel::apply(CoherencyMessage * m) {
  return impl_->apply(m);
}

enum class MsiDirectoryLineState {
  I,
  S,
  M,
  S_D
};

struct MsiDirectoryModel::MsiDirectoryModelImpl {
  MsiDirectoryModelImpl(const DirectoryOptions & opts)
      : opts_(opts)
  {}
  
  DirectoryAction apply(CoherencyMessage * m) {
    return {};
  }
  
  DirectoryOptions opts_;
};

MsiDirectoryModel::MsiDirectoryModel(const DirectoryOptions & opts)
    : DirectoryModel(opts) {
}

MsiDirectoryModel::~MsiDirectoryModel() {}

DirectoryAction MsiDirectoryModel::apply(CoherencyMessage * m) {
  return impl_->apply(m);
}

} // namespace ccm
