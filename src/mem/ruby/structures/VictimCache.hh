/*
 * Copyright (c) 2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEM_RUBY_STRUCTURES_VICTIMCACHE_HH__
#define __MEM_RUBY_STRUCTURES_VICTIMCACHE_HH__

#include <string>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/CacheRequestType.hh"
#include "mem/ruby/protocol/CacheResourceType.hh"
#include "mem/ruby/protocol/RubyRequest.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/structures/BankedArray.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/RubyCache.hh"
#include "params/VictimCache.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class VictimCache : public SimObject
{
  public:
    typedef VictimCacheParams Params;
    VictimCache(const Params &p);
    ~VictimCache();

    void init();


    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an unused line in the same cache "way"
    Addr selectVictim(NetDest dom) const;
    void invalDomain(NetDest dom);
    bool allLessThanThreshold(NetDest dom) const;
    bool cacheAvail(Addr address, NetDest dom) const;
    

    // find an unused entry and sets the tag appropriate for the address
    AbstractCacheEntry* allocate(Addr address, AbstractCacheEntry* new_entry, NetDest dom);

    // Explicitly free up this address
    void deallocate(Addr address);

    // Returns with the physical address of the conflicting cache line
    Addr cacheProbe() const;

    // looks an address up in the cache
    AbstractCacheEntry* lookup(Addr address);
    const AbstractCacheEntry* lookup(Addr address) const;


    // Print cache contents
    void print(std::ostream& out) const;
    void printData(std::ostream& out) const;

  public:
    int getCacheAssoc() const { return m_cache_assoc; }
    int getNumBlocks() const { return m_cache_assoc; }
    
  private:
    int addressToCacheIndex(Addr address) const;

  private:
    std::unordered_map<Addr, int> m_address_index;
    std::vector<AbstractCacheEntry*> m_cache;
    std::vector<std::vector<int>> m_security_domian;
    std::vector<int> active_threshold;
    
    int pre_threshold;
    int m_block_size;
    int m_cache_assoc;
    int m_num_security_domain;


    private:
      struct VictimCacheStats : public statistics::Group
      {
          VictimCacheStats(statistics::Group *parent, int num);

          statistics::Scalar m_demand_hits;
          statistics::Scalar m_demand_misses;
          statistics::Formula m_demand_accesses;
	  statistics::Scalar m_buffered_blocks;
	  statistics::Scalar m_LLC_evictionss;
	  statistics::Scalar m_LLC_evictionm;
	  statistics::Scalar m_LLC_evictionmt;

	  std::vector<statistics::Scalar *> m_owned_blocks;
      } victimCacheStats;

    public:
      // These function increment the number of demand hits/misses by one
      // each time they are called
      void profileDemandHit();
      void profileDemandMiss();
      void profileBufferedBlocks();
      void profileLLCEvictionss();
      void profileLLCEvictionm();
      void profileLLCEvictionmt();
      void profileOwnedBlocks(int core_id);
};

std::ostream& operator<<(std::ostream& out, const VictimCache& obj);

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_VICTIMCACHE_HH__
