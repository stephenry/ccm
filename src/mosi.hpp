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

#ifndef __SRC_MOSI_HPP__
#define __SRC_MOSI_HPP__

#include "coherence.hpp"

namespace ccm {

// clang-format off
#define MOSI_LINE_STATES(__func)                \
  __func(I)                                     \
  __func(IS_D)                                  \
  __func(IM_AD)                                 \
  __func(IM_A)                                  \
  __func(S)                                     \
  __func(SM_AD)                                 \
  __func(SM_A)                                  \
  __func(M)                                     \
  __func(MI_A)                                  \
  __func(O)                                     \
  __func(OM_AC)                                 \
  __func(OM_A)                                  \
  __func(OI_A)                                  \
  __func(SI_A)                                  \
  __func(II_A)                                  \
  __func(STATE_COUNT)
// clang-format on

struct MosiAgentLineState {
  // clang-format off
  enum : state_t {
#define __declare_state(__state)                \
  __state,
  MOSI_LINE_STATES(__declare_state)
#undef __declare_state
  };
  // clang-format on

  static const char* to_string(state_t state);
  static bool is_stable(state_t state);
};

// clang-format off
#define MOSI_DIRECTORY_STATES(__func)           \
  __func(I)                                     \
  __func(S)                                     \
  __func(O)                                     \
  __func(M)
// clang-format on

struct MosiDirectoryLineState {
  // clang-format off
  enum : state_t {
#define __declare_state(__state)                \
  __state,
  MOSI_DIRECTORY_STATES(__declare_state)
#undef __declare_state
  };
  // clang-format on

  static const char* to_string(state_t state);
  static bool is_stable(state_t state);
};

class MosiAgentProtocol : public AgentProtocol {
 public:
  MosiAgentProtocol(const CoherentAgentOptions& opts);
  virtual ~MosiAgentProtocol();

  //
  Protocol protocol() const override { return Protocol::MSI; }

  //
  void init(CacheLine& l) const override;
  bool is_stable(const CacheLine& l) const override;
  std::string to_string(CacheLine::state_type s) const override;

  //
  CoherenceActions get_actions(const Transaction* t,
                               const CacheLine& cache_line) const override;
  CoherenceActions get_actions(const Message* m,
                               const CacheLine& cache_line) const override;

 private:
  struct MosiAgentProtocolImpl;

  std::unique_ptr<MosiAgentProtocolImpl> impl_;
};

class MosiSnoopFilterProtocol : public SnoopFilterProtocol {
 public:
  MosiSnoopFilterProtocol(const ActorOptions& opts);
  virtual ~MosiSnoopFilterProtocol();

  //
  Protocol protocol() const override { return Protocol::MSI; }

  //
  void init(DirectoryEntry& l) const override;
  bool is_stable(const DirectoryEntry& l) const override;
  std::string to_string(const DirectoryEntry& l) const override;
  std::string to_string(CacheLine::state_type l) const override;

  //
  CoherenceActions get_actions(const Message* m,
                               const DirectoryEntry& dir_entry) const override;

 private:
  struct MosiSnoopFilterProtocolImpl;

  std::unique_ptr<MosiSnoopFilterProtocolImpl> impl_;
};

class MosiCoherenceProtocolValidator : public CoherenceProtocolValidator {
 public:
  MosiCoherenceProtocolValidator();

  bool validate_addr(addr_t addr, const std::vector<Entry<CacheLine> >& lines,
                     const DirectoryEntry& entry) const;
};

}  // namespace ccm

#endif
