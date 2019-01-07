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

#ifndef __TESTS_COMMON_HPP__
#define __TESTS_COMMON_HPP__

#include "ccm.hpp"

namespace ccm::test {

struct BasicPlatform {
  BasicPlatform(Sim & sim, Protocol protocol, std::size_t agents_n);
  ~BasicPlatform();
  std::size_t agents() const { return agents_.size(); }
  
  Protocol protocol() const { return protocol_; }
  Agent * agent(std::size_t id) { return agents_[id]; }
  SnoopFilter * snoop_filter() { return snoop_filter_; }
  
 private:
  void construct_snoop_filter(std::size_t id);
  void construct_agent(std::size_t id);
  void add_actor(CoherentActor * actor);

  Logger logger_;
  LoggerScope * top_;
  SnoopFilter * snoop_filter_;
  Sim & sim_;
  std::vector<Agent *> agents_;
  std::vector<CoherentActor *> actors_;
  Protocol protocol_;
};

} // namespace ccm::test

#endif
