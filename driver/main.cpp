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

#include "ccm.hpp"
#include <iostream>
#include <map>
#include <sstream>
#include <nlohmann/json.hpp>

const char * VERSION{"0.0.1"};

ccm::Protocol string_to_protocol(const std::string & s) {
  static const std::map<std::string, ccm::Protocol> pmap{
    { "MSI", ccm::Protocol::MSI },
#ifdef ENABLE_MESI
    { "MESI", ccm::Protocol::MESI },
#endif
#ifdef ENABLE_MOSI
    { "MOSI", ccm::Protocol::MOSI },
#endif
    { "INVALID", ccm::Protocol::INVALID }
  };

  auto it = pmap.find(s);
  if (it == pmap.end())
    throw std::invalid_argument("Unknown protocol: "  + s);

  return it->second;
}

int main(int argc, char **argv) {
  nlohmann::json j;

  j << std::cin;
  j >> std::cout;

  
  return 0;
}
