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
#include <sstream>
#include "platform.hpp"
#include "protocol.hpp"
#include "agent.hpp"
#include "snoopfilter.hpp"
#include "interconnect.hpp"

namespace {

bool has_key(nlohmann::json & j, const std::string & s) {
  return (j.find(s) != j.end());
}

} // namespace

namespace ccm {

void Builder::drc(nlohmann::json & j) {
  if (!has_key(j, "loglevel"))
    j["loglevel"] = "error";
  if (!has_key(j, "name"))
    j["name"] = "top";
  
  drc_agents(j);
  drc_snoop_filters(j);
}

void Builder::drc_agents(nlohmann::json & j) {
  std::size_t idx{0};
  for (nlohmann::json & j_agent : j["agents"]) {
    if (!has_key(j, "name")) {
      std::stringstream ss;
      ss << "UnknownAgent" << idx++;
      j_agent["name"] = ss.str();
    }
  }
}

void Builder::drc_snoop_filters(nlohmann::json & j) {
  std::size_t idx{0};
  for (nlohmann::json & j_snoop : j["snoopfilters"]) {
    if (!has_key(j, "name")) {
      std::stringstream ss;
      ss << "UnknownSnoopFilter" << idx++;
      j_snoop["name"] = ss.str();
    }
  }
}

void Builder::setup_sim(Sim * sim, nlohmann::json & j) {
  sim->logger_.set_llevel(LogLevel::from_string(j["loglevel"]));

  sim->platform_ = Platform::from_json(j);
  sim->epoch_period_ = j["sim"]["epoch_period"];
  sim->epoch_step_ = j["sim"]["epoch_step"];
}

std::unique_ptr<Sim> Builder::construct(nlohmann::json & j) {
  Builder::drc(j);
  
  std::unique_ptr<Sim> sim = std::make_unique<Sim>();

  // Setup logger
  setup_sim(sim.get(), j);

  LoggerScope * l = sim->logger_.top(j["name"]);

  // Agents
  AgentCostModel agent_cost;
  if (has_key(j, "costs") && has_key(j["costs"], "agent"))
    agent_cost = AgentCostModel::from_json(j["costs"]["agent"]);
  for (nlohmann::json & j_agent : j["agents"])
    sim->add_actor(AgentBuilder::construct(
        sim->platform(), l, j_agent, agent_cost));
  // Snoopfilters
  SnoopFilterCostModel snoopfilter_cost;
  if (has_key(j, "costs") && has_key(j["costs"], "snoopfilter"))
    snoopfilter_cost = SnoopFilterCostModel::from_json(j["costs"]["snoopfilter"]);
  for (nlohmann::json & j_snoop : j["snoopfilters"])
    sim->add_actor(SnoopFilterBuilder::construct(
        sim->platform(), l, j_snoop, snoopfilter_cost));

  // Interconnect
  sim->add_interconnect(InterconnectModel::from_json(j["interconnect"]));

  return std::move(sim);
}


} // namespace ccm
