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
  
  void apply(CoherencyMessage * m) {
    // Dispatch to handler
    switch (m->type()) {
#define __declare_dispatcher(e)                                         \
      case MessageType::e:                                              \
      return handle__ ## e(static_cast<e ## CoherencyMessage *>(m));

      MESSAGE_CLASSES(__declare_dispatcher)
#undef __declare_dispatcher
      default:
          // TOOD: unknown message class.
          ;
     }
    // Return transaction to owner pool.
    m->release();
  }

  void handle__Load(LoadCoherencyMessage * m) {
  }
  
  void handle__Store(StoreCoherencyMessage * m) {
  }
  
  void handle__Replacement(ReplacementCoherencyMessage * m) {
  }
  
  void handle__FwdGetS(FwdGetSCoherencyMessage * m) {
  }
  
  void handle__FwdGetM(FwdGetMCoherencyMessage * m) {
  }
  
  void handle__Inv(InvCoherencyMessage * m) {
  }
  
  void handle__PutAck(PutAckCoherencyMessage * m) {
  }
  
  void handle__DataDir(DataDirCoherencyMessage * m) {
  }
  
  void handle__DataOwner(DataOwnerCoherencyMessage * m) {
  }
  
  void handle__InvAck(InvAckCoherencyMessage * m) {
  }

  GenericCacheModel<MsiAgentLineState> cache_;
  CoherentAgentOptions opts_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

void MsiCoherentAgentModel::apply(CoherencyMessage * m) {
  impl_->apply(m);
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
  
  void apply(CoherencyMessage * m) {
  }
  
  DirectoryOptions opts_;
};

MsiDirectoryModel::MsiDirectoryModel(const DirectoryOptions & opts)
    : DirectoryModel(opts) {
}

MsiDirectoryModel::~MsiDirectoryModel() {}

void MsiDirectoryModel::apply(CoherencyMessage * m) {
  impl_->apply(m);
}

} // namespace ccm
