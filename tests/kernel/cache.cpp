//========================================================================== //
// Copyright (c) 2019, Stephen Henry
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

#include <gtest/gtest.h>
#include <vector>
#include <algorithm>
#include "cache.hpp"
#include "protocol.hpp"

template<typename T>
void basic_cache_tests(ccm::GenericCache<T> & cache, std::size_t n = 1000) {
  cache.reset();
  ccm::Random::UniformRandomInterval<ccm::addr_t> rnd{};
  for (std::size_t i = 0; i < n; i++) {
    const ccm::addr_t addr = rnd();

    EXPECT_FALSE(cache.is_hit(addr));
    cache.install(addr, ccm::CacheLine{});
    EXPECT_TRUE(cache.is_hit(addr));

    const ccm::CacheLine & line = cache.lookup(addr);
    EXPECT_TRUE(line.is_valid());

    cache.evict(addr);
    EXPECT_FALSE(cache.is_hit(addr));

    cache.install(addr, ccm::CacheLine{});
    EXPECT_TRUE(cache.is_hit(addr));
    cache.reset();
    EXPECT_FALSE(cache.is_hit(addr));
  }
}

template<typename T>
void associativity_tests(ccm::GenericCache<T> & cache,
                         const ccm::CacheOptions & opts) {
  const ccm::addr_t a_victim{0x0};
  const ccm::addr_t a_aggressor{0x10000000};
  
  EXPECT_FALSE(cache.evict(a_victim));
  cache.install(a_victim, ccm::CacheLine{});
  EXPECT_FALSE(cache.install(a_aggressor, ccm::CacheLine{}));
  EXPECT_TRUE(cache.requires_eviction(a_aggressor));
  EXPECT_EQ(cache.nominate_evictee(a_aggressor).base(), a_victim);
  EXPECT_TRUE(cache.evict(0));
  EXPECT_TRUE(cache.install(0x10000000, ccm::CacheLine{}));
}

TEST(Cache, DirectMapped) {
  using namespace ccm;

  CacheOptions opts;
  opts.set_type(CacheType::DirectMapped);
  opts.set_line_bytes(64);
  opts.set_ways_n(1);
  opts.set_size_bytes(opts.line_bytes() * (1 << 10));
  
  SetAssociativeCache<CacheLine> cache{opts};
  basic_cache_tests(cache);

  cache.reset();
  associativity_tests(cache, opts);
}

TEST(Cache, SetAssociative2) {
  using namespace ccm;

  CacheOptions opts;
  opts.set_type(CacheType::SetAssociative);
  opts.set_line_bytes(64);
  opts.set_ways_n(2);
  opts.set_size_bytes(opts.line_bytes() * (1 << 10));

  SetAssociativeCache<CacheLine> cache{opts};
  basic_cache_tests(cache);

  cache.reset();
  //  associativity_tests(cache, opts);
}

TEST(Cache, SetAssociative4) {
  using namespace ccm;

  CacheOptions opts;
  opts.set_type(CacheType::SetAssociative);
  opts.set_line_bytes(64);
  opts.set_ways_n(4);
  opts.set_size_bytes(opts.line_bytes() * (1 << 10));

  SetAssociativeCache<CacheLine> cache{opts};
  basic_cache_tests(cache);

  cache.reset();
  //  associativity_tests(cache, opts);
}

TEST(Cache, InfiniteCapacity) {
  using namespace ccm;

  CacheOptions opts;
  opts.set_type(CacheType::InfiniteCapacity);
  
  InfiniteCapacityCache<CacheLine> cache{opts};
  basic_cache_tests(cache);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
