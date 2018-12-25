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

#include "coherence/coherence.hpp"
#include <gtest/gtest.h>
#include <deque>
#include <map>
#include <list>
#include <memory>

struct PlatformHost {

  PlatformHost(std::size_t agents = 4) {
    // Construct agents
    //
    for (std::size_t i = 0; i < agents; i++) {
      ccm::CoherentAgentOptions opts{i};
      //      std::unique_ptr<CoherentAgentModel> agent
      //          = ccm::coherent_agent_factory(ccm::Protocol::MSI, opts);
      
      //      agents_.push_back(agent.get());
      //      m_.insert(std::make_pair(opts.id(), agent.get));
      //      actors_.push_back(std::move(agent));
    }

    // Construct directory
    //
    ccm::SnoopFilterOptions opts{agents};
    //    std::unique_ptr<SnoopFilterModel> filter =
    //        ccm::snoop_filter_factory(ccm::Protocol::MSI, opts);
    //    m_.insert(std::make_pair(opts.id(), filter.get()));
    //    actors_.push_back(std::move(filter));
  }

  void apply_transaction(ccm::Transaction t, std::size_t agent = 0) {

    
    // transaction_ = t;
    // while (pending_msg_.size() != 0) {
    //   ccm::CoherencyMessage * msg = pending_msg_.front(); pending_msg_.pop_front();
    //   const ccm::CoherentActorResult r = m_[msg->sid()]->apply(msg);
    //   switch (r.status()) {
    //     case ccm::CoherentActorResultStatus::Advances: {
    //       for (ccm::CoherencyMessage * m : r.msgs())
    //         pending_msg_.push_back(m);
    //       msg->release();
    //     } break;

    //     default:
    //       ; // TODO
    //   }
    // }
  }

 private:
  std::map<std::size_t, ccm::CoherentActorBase *> m_;
  std::vector<ccm::CoherentAgentModel *> agents_;
  std::vector<std::unique_ptr<ccm::CoherentActorBase> > actors_;
  
  // std::deque<ccm::CoherenceMessage *> pending_msg_;
};


TEST(MSI, Load) {
  PlatformHost h;
  h.apply_transaction(ccm::Transaction::make_load(0));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
