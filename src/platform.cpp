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

#include "platform.hpp"

namespace ccm {

void Platform::add_agent(id_t id) {
  agent_ids_.insert(id);
}

void Platform::add_snoop_filter(id_t id, std::shared_ptr<AddressRegion> && ar) {
  snoop_filters_.insert(std::make_pair(id, std::move(ar)));
}

bool Platform::is_valid_agent_id(id_t id) const {
  return (agent_ids_.find(id) != agent_ids_.end());
}

bool Platform::is_valid_snoop_filter_id(id_t id) const {
  return (snoop_filters_.count(id) != 0);
}

id_t Platform::get_snoop_filter_id(addr_t addr) const {
  for (auto & [id, address_region] : snoop_filters_) {
    if (address_region->is_valid(addr))
      return id;
  }
  // Otherwise, throw exception.
  throw std::invalid_argument("SnoopFilter for address not found.");
}

} // namespace ccm