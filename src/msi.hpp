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

#ifndef __SRC_MSI_HPP__
#define __SRC_MSI_HPP__

#include "coherence.hpp"
#include <memory>

namespace ccm {

class MsiCoherentAgentModel : public CoherentAgentModel {
 public:
  MsiCoherentAgentModel(const AgentOptions & opts);
  virtual ~MsiCoherentAgentModel();

  //
  Protocol protocol() const override { return Protocol::MSI; }

  //
  CoherentActorActions get_actions(const Transaction * t) override;
  CoherentActorActions get_actions(const Message * m) override;

 private:
  struct MsiCoherentAgentModelImpl;

  std::unique_ptr<MsiCoherentAgentModelImpl> impl_;
};

class MsiSnoopFilterModel : public SnoopFilterModel {
 public:
  MsiSnoopFilterModel(const SnoopFilterOptions & opts);
  virtual ~MsiSnoopFilterModel();

  //
  Protocol protocol() const override { return Protocol::MSI; }

  //
  CoherentActorActions get_actions(const Message * m) override;

 private:
  struct MsiSnoopFilterModelImpl;
  struct MsiSnoopFilterModelNullFilterImpl;
  struct MsiSnoopFilterModelDirectoryImpl;

  std::unique_ptr<MsiSnoopFilterModelImpl> impl_;
};


} // namespace ccm

#endif
