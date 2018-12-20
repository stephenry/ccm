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

#include <sstream>
#include <cmath>

namespace ccm::kernel {

namespace detail {

template<typename ARG>
std::ostream & join_helper(std::ostream & os, ARG && arg) {
  return (os << arg);
}

template<typename ARG, typename ... REST>
std::ostream & join_helper(std::ostream & os, ARG && arg, REST && ... rest) {
  return join_helper(os << arg, std::forward<REST>(rest)...);
}

} // namespace detail

template<typename ...ARGS>
std::string join(ARGS && ... args) {
  std::stringstream ss;
  detail::join_helper(ss, std::forward<ARGS>(args)...);
  return ss.str();
}

template<typename T>
T log2ceil(T t) {
  return static_cast<T>(std::ceil(std::log2(t)));
}

template<typename T, typename = std::is_integral<T> >
T mask(T t) { return (1 << t) - 1; }

} // namespace ccm::kernel
