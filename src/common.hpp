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

#ifndef __SRC_COMMON_HPP__
#define __SRC_COMMON_HPP__

#include <cstdint>
#include <iostream>

namespace ccm {

#define CCM_MACRO_BEGIN do {
#define CCM_MACRO_END \
  }                   \
  while (false)

#define CCM_ASSERT(__cond)                                 \
  CCM_MACRO_BEGIN                                          \
  if (!(__cond)) {                                         \
    std::cout << __FILE__ << ":" << __LINE__               \
              << " assertion failed: " << #__cond << "\n"; \
    std::exit(1);                                          \
  }                                                        \
  CCM_MACRO_END

#define CCM_AGENT_ASSERT(__cond)                                        \
  CCM_MACRO_BEGIN                                                       \
  if (!(__cond))                                                        \
    log_fatal(__FILE__, ":", __LINE__, " assertion failed: ", #__cond); \
  CCM_MACRO_END

using addr_t = std::size_t;

}  // namespace ccm

#endif
