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

#ifndef __LOG_HPP__
#define __LOG_HPP__

#include <sstream>
#include <iostream>

namespace ccm::kernel {

enum class LogLevel {
  Fatal,
  Error,
  Warning,
  Info,
  Debug
};

const char * to_string(LogLevel ll);

struct Logger {

  Logger()
      : os_(std::cout)
  {}

  ~Logger() {
    os_.flush();
  }

  template<typename ... ARGS>
  void log(LogLevel ll, ARGS && ... args) {
    log_helper(os_, std::forward<ARGS>(args)...);
    std::cout << "\n";
  }

 private:

  //
  template<typename ARG, typename ... ARGS>
  void log_helper(std::ostream & os, ARG arg, ARGS && ... args) {
    log_helper(os << arg, std::forward<ARGS>(args)...);
  }

  //
  template<typename ARG>
  void log_helper(std::ostream & os, ARG arg) {
    os << arg;
  }

  std::ostream & os_;
};
  
} // namespace ccm::kernel

#endif
