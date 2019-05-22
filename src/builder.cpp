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

#include "builder.hpp"
#include "platform.hpp"
#include "protocol.hpp"
#include "agent.hpp"
#include "snoopfilter.hpp"

namespace ccm {

void Builder::drc(nlohmann::json & h) {
  // TODO: validate correctness of json format.
}

std::unique_ptr<Sim> Builder::construct(nlohmann::json & j) {
  Builder::drc(j);
  
  std::unique_ptr<Sim> sim = std::make_unique<Sim>();

  sim->platform_ = Platform::from_json(j);
  for (nlohmann::json & j_agent : j["agents"])
    sim->add_actor(AgentBuilder::construct(sim->platform(), j_agent));
  for (nlohmann::json & j_snoop : j["snoopfilters"])
    sim->add_actor(SnoopFilterBuilder::construct(sim->platform(), j_snoop));

  return std::move(sim);
}


} // namespace ccm
