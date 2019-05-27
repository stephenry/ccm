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

#include "cache.hpp"
#include "utility.hpp"

namespace ccm {

#ifdef ENABLE_JSON

CacheOptions CacheOptions::from_json(nlohmann::json j) {
  // TODO: remains TBD
  return CacheOptions();
}
#endif

CacheOptions::CacheOptions() { reset(); }

void CacheOptions::reset() {
#define __reset_fields(__name, __type, __default)     \
  __type __name ## _{__default};
  CACHE_OPTION_FIELDS(__reset_fields)
#undef __reset_fields
}

CacheAddressFieldHelper::CacheAddressFieldHelper(const CacheOptions & opts)
    : opts_(opts) {
  precompute();
}

void CacheAddressFieldHelper::precompute() {
  bits_for_line_ = log2ceil(opts_.line_bytes());
  lines_n_ = (opts_.size_bytes() / opts_.line_bytes());
  sets_n_ = (lines_n_ / opts_.ways_n());
  bits_for_set_ = log2ceil(sets_n_);
}

std::size_t CacheAddressFieldHelper::offset(addr_t a) const {
  return get_range(a, bits_for_line());
}

std::size_t CacheAddressFieldHelper::set(addr_t a) const {
  return get_range(a, bits_for_line() + bits_for_set(), bits_for_set());
}

std::size_t CacheAddressFieldHelper::tag(addr_t a) const {
  return a >> (bits_for_line() + bits_for_set());
}

std::size_t CacheAddressFieldHelper::base(addr_t a) const {
  return a & ~mask(bits_for_line() - 1);
}

}  // namespace ccm
