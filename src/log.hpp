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

#ifndef __SRC_LOG_HPP__
#define __SRC_LOG_HPP__

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include "types.hpp"

namespace ccm {

// clang-format off
#define LOGLEVELS(__func)                       \
  __func(debug)                                 \
  __func(info)                                  \
  __func(warning)                               \
  __func(error)                                 \
  __func(fatal)
// clang-format on

// clang-format off
struct LogLevel {
  using type = uint8_t;
  
  enum : type {
#define __declare_level(__level) __level,
  LOGLEVELS(__declare_level)
#undef __declare_level
  };
  static type from_string(const std::string & s);
  static const char* to_string(type ll);
};
// clang-format on

class LoggerScope;

class Logger {
  friend class LoggerScope;

 public:
  explicit Logger(LogLevel::type llevel = LogLevel::debug);
  ~Logger();

  LoggerScope* top(const std::string & top = "t");
  void set_llevel(LogLevel::type llevel) { llevel_ = llevel; }
  void set_force_flush(bool force_flush = true) { force_flush_ = force_flush; }

 private:
  void log(const LogLevel::type ll, const std::string& s);
  void log(const LogLevel::type ll, const char* s);

  std::ostream& os_{std::cout};
  std::unique_ptr<LoggerScope> top_{nullptr};
  bool force_flush_{false};
  LogLevel::type llevel_{LogLevel::debug};
};

class LoggerScope {
  friend class Loggable;
  friend class Logger;

  static const char SEP;

  LoggerScope(const std::string& s, Logger* logger);

 public:
  std::string path() const { return path_; }
  Logger* logger() const { return logger_; }

  LoggerScope* child_scope(const std::string& leaf);

 private:
  void log(const LogLevel::type ll, const std::string& s) { logger_->log(ll, s); }

  std::string path_;
  Logger* logger_{nullptr};
  std::unordered_map<std::string, std::unique_ptr<LoggerScope>> scopes_;
  LogLevel::type llevel_{LogLevel::debug};
};

class Loggable {
 public:
  LoggerScope* logger_scope() const { return scope_; }
  void set_logger_scope(LoggerScope* scope) { scope_ = scope; }

// clang-format off
#define __declare_handler(__level)                              \
  template <typename... ARGS>                                   \
  void log_##__level(const Time & t, ARGS&&... args) {          \
    log(LogLevel::__level, t, std::forward<ARGS>(args)...);     \
  }
  LOGLEVELS(__declare_handler)
#undef __declare_handler
  // clang-format on

 private:
  template <typename ARG>
  void log_helper(std::ostream& os, ARG arg) {
    os << arg;
  }

  template <typename ARG, typename... ARGS>
  void log_helper(std::ostream& os, ARG arg, ARGS&&... args) {
    log_helper(os << arg, std::forward<ARGS>(args)...);
  }

  template <typename... ARGS>
  void log(LogLevel::type ll, const Time & time, ARGS&&... args) {
    if (scope_) {
      std::stringstream ss;
      prefix(ss, time);
      log_helper(ss, std::forward<ARGS>(args)...);
      scope_->log(ll, ss.str());
    }
  }

  void prefix(std::ostream& os, const Time & time);

  LoggerScope* scope_{nullptr};
};

struct StateUpdateLogger {
  StateUpdateLogger() {}

  std::string str() const { return ss.str(); }

  template <typename T>
  void add(const T& t, const std::string& prefix = "") {
    ss << prefix << "'{" << join(t.begin(), t.end()) << "}";
  }

 private:
  std::stringstream ss;
};

}  // namespace ccm

#endif
