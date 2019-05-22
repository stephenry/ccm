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

struct Options {
  ccm::Protocol protocol{ccm::Protocol::MSI};
  std::size_t agents{4};
  std::size_t run{1000};
};

std::string to_string(const Options & options) {
  std::stringstream ss;
  ss << "Protocol: " << ccm::to_string(options.protocol) << ", "
     << "Agents: " << options.agents
    ;
  return ss.str();
};

template<typename OutIt>
void split(const std::string & str, OutIt it, const char sep = ',') {
  if (str.empty())
    return ;

  std::string::size_type i{0}, j{0};
  do {
    j = str.find(sep, i);
    *it = (j == std::string::npos) ? str.substr(i) : str.substr(i, (j - i));
    i = (j + 1);
  } while (j != std::string::npos);
}

struct Platform {
  void sim() {}
};

struct Builder {

  Builder(const Options & opts) {}

  Platform build() const { return Platform{}; }

private:
  const Options * opts_;
};

struct Driver {
  static void parse_command_line(Options & opts,
                                 const std::vector<const char *> & args) {
    std::size_t i = 1;
    auto shift = [&]() -> std::string {
      if (++i < args.size())
        return args[i];
      else
        throw std::out_of_range("Malformed argument list.");
    };

    try {
      while (i < args.size()) {
        const std::string sstr{args[i]};
        if (sstr == "-h" || sstr == "--help") {
          help();
        } else if (sstr == "-p" || sstr == "--protocol") {
          opts.protocol = string_to_protocol(shift());
        } else if (sstr == "-a" || sstr == "--agents") {
          opts.agents = std::stoull(shift());
        } else {
          help();
        }
        ++i;
      }
    } catch (std::exception & ex) {
      std::cerr << "Invalid command line: " << ex.what() << "\n";
      help();
    }
  }

private:
  static void help() {
    std::cerr << "CCM simulator driver version: " << VERSION << "\n";
    std::exit(1);
  }
};

int main(int argc, char **argv) {
  const std::vector<const char *> args{argv, argv + argc};
  
  Options opts;
  Driver::parse_command_line(opts, args);
  std::cout << "Options: " << to_string(opts) << "\n";
  Builder builder{opts};
  Platform p = builder.build();
  p.sim();

  return 0;
}
