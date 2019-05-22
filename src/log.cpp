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

#include "log.hpp"
#include "actor.hpp"

namespace ccm {

const char* to_string(LogLevel ll) {
  switch (ll) {
// clang-format off
#define __declare_to_string(__level) \
  case LogLevel::__level:            \
    return #__level;                 \
    break;
    LOGLEVELS(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "Unknown";
  }
}

const char LoggerScope::SEP = '.';

Logger::Logger() {}

Logger::~Logger() { os_.flush(); }

LoggerScope* Logger::top() {
  if (!top_) top_ = std::unique_ptr<LoggerScope>(new LoggerScope("t", this));

  return top_.get();
}

LoggerScope::LoggerScope(const std::string& path, Logger* logger)
    : path_(path), logger_(logger) {}

LoggerScope* LoggerScope::child_scope(const std::string& leaf) {
  if (scopes_.find(leaf) == scopes_.end()) {
    std::stringstream ss;
    ss << path() << LoggerScope::SEP << leaf;
    scopes_[leaf] =
        std::unique_ptr<LoggerScope>(new LoggerScope(ss.str(), logger()));
  }
  return scopes_[leaf].get();
}

void Logger::log(const LogLevel ll, const std::string& s) {
  os_ << s << "\n";
  if (force_flush_) os_.flush();
}

void Logger::log(const LogLevel ll, const char* s) {
  os_ << s << "\n";
  if (force_flush_) os_.flush();
}

void Loggable::prefix(std::ostream& os) {
  os << "[" << scope_->path() << "@" << time() << "]: ";
}

}  // namespace ccm
