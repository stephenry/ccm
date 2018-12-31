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
