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

#include "mem/ruby/structures/VictimCache.hh"

#include "base/random.hh"
#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HtmMem.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubyResourceStalls.hh"
#include "debug/RubyStats.hh"
#include "mem/cache/replacement_policies/weighted_lru_rp.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

std::ostream&
operator<<(std::ostream& out, const VictimCache& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

VictimCache::VictimCache(const Params &p)
    : SimObject(p),
    victimCacheStats(this, p.core_num)
{
    pre_threshold = p.threshold;
    m_num_security_domain = p.core_num;
    m_cache_assoc = p.assoc;
    m_block_size = p.block_size;  // may be 0 at this point. Updated in init()_
}

void
VictimCache::init()
{
    if (m_block_size == 0) {
        m_block_size = RubySystem::getBlockSizeBytes();
    }

    m_cache.resize(m_cache_assoc, NULL);
    m_security_domian.resize(m_num_security_domain, std::vector<int>(m_cache_assoc, 0));
    active_threshold.resize(m_num_security_domain, 0);
}

VictimCache::~VictimCache()
{
    for (int i = 0; i < m_cache_assoc; i++) {
        delete m_cache[i];
    }
}

int
VictimCache::addressToCacheIndex(Addr address) const
{
    assert(address == makeLineAddress(address));
    auto it = m_address_index.find(address);
    if(it != m_address_index.end())
        return it->second;
    return -1;
}

Addr
VictimCache::selectVictim(NetDest dom) const
{   
    std::vector<unsigned int> domain = dom.getAllDest();
    
    for(int i = 0; i < domain.size(); i++){
        int core_id = domain[i] - m_num_security_domain;
        if(active_threshold[core_id] >= pre_threshold){
            int total = std::accumulate(m_security_domian[core_id].begin(), m_security_domian[core_id].end(), 0);
            int random_index = random_mt.random<unsigned>(0,
                                                          total - 1);
            int index = 0;
            for(int j = 0; j < m_cache_assoc; j++){
                if(m_security_domian[core_id][j]){
                    if(index == random_index)
                        return m_cache[j]->m_Address;
                    index++;
                }
            }
            panic("cannot find victim block in victim cache");
        }
    }
    return 0;
}

bool
VictimCache::allLessThanThreshold(NetDest dom) const
{
    std::vector<unsigned int> domain = dom.getAllDest();
    
    for(int i = 0; i < domain.size(); i++){
        int core_id = domain[i] - m_num_security_domain;
        if(active_threshold[core_id] >= pre_threshold){
            return false;
        }
    }
    return true;
}

void
VictimCache::invalDomain(NetDest dom)
{
    std::vector<unsigned int> domain = dom.getAllDest();
    for(int i = 0; i < domain.size(); i++){
        int core_id = domain[i] - m_num_security_domain;
        if(active_threshold[core_id] >= pre_threshold){
            int total = std::accumulate(m_security_domian[core_id].begin(), m_security_domian[core_id].end(), 0);
            int random_index = random_mt.random<unsigned>(0, total - 1);
            int index = 0;
            for(int j = 0; j < m_cache_assoc; j++){
                if(m_security_domian[core_id][j]){
                    if(index == random_index){
                        index = j;
                        break;
                    }
                    index++;
                }
            }
            assert(m_security_domian[core_id][index] == 1);
            m_security_domian[core_id][index] = 0;
            active_threshold[core_id]--;
            return;
        }
    }
    panic("cannot invalidate any line from domain level");
}

bool
VictimCache::cacheAvail(Addr address, NetDest dom) const
{
    assert(address == makeLineAddress(address));

    std::vector<unsigned int> domain = dom.getAllDest();
    
    for(int i = 0; i < domain.size(); i++){
        int core_id = domain[i] - m_num_security_domain;
        if(active_threshold[core_id] >= pre_threshold)
            return false;
    }
    
    assert(m_address_index.size() <= m_cache_assoc);
    if(m_address_index.size() == m_cache_assoc)
        return false;
    return true;
}


AbstractCacheEntry*
VictimCache::allocate(Addr address, AbstractCacheEntry *entry, NetDest dom)
{
    assert(address == makeLineAddress(address));
    assert(cacheAvail(address, dom));
    assert(addressToCacheIndex(address) == -1);
    
    DPRINTF(RubyCache, "allocate vc: address: %#x\n", address);

    for (int i = 0; i < m_cache_assoc; i++) {
        if (m_cache[i] == NULL) {
            m_cache[i] = entry;  // Init entry
            m_cache[i]->m_Address = address;
            m_cache[i]->m_Permission = AccessPermission_Invalid;
            DPRINTF(RubyCache, "Allocate clearing lock for addr: %x\n",
                    address);
            m_cache[i]->m_locked = -1;
            m_address_index[address] = i;
            
            //set the security domain
            std::vector<unsigned int> domain = dom.getAllDest();
            for(int j = 0; j < domain.size(); j++){
                int core_id = domain[j] - m_num_security_domain;
                m_security_domian[core_id][i] = 1;
                active_threshold[core_id]++;
		profileOwnedBlocks(core_id);
            }

            return entry;
        }
    }
    panic("Allocate didn't find an available entry");
}

void
VictimCache::deallocate(Addr address)
{
    DPRINTF(RubyCache, "address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    uint32_t index = addressToCacheIndex(address);
    for(int i = 0; i < m_num_security_domain; i++){
	if(m_security_domian[i][index]){
	   active_threshold[i]--;
           m_security_domian[i][index] = 0;
	}
    }
    
    assert(entry != nullptr);
    assert(addressToCacheIndex(address) != -1);
    delete entry;
    m_cache[index] = NULL;
    m_address_index.erase(address);
}

// Returns with the physical address of the conflicting cache line
Addr
VictimCache::cacheProbe() const
{
    //gurantee all block exist and one victim exists
    for(int i = 0; i < m_cache_assoc; i++){
        int sum = 0;
        for(int j = 0; j < m_num_security_domain; j++){
            sum += m_security_domian[j][i];
        }
        // is a block which don't have security domain
        if(sum == 0)
            return m_cache[i]->m_Address;
    }
    panic("don't find a probe a cache block");
}

// looks an address up in the cache
AbstractCacheEntry*
VictimCache::lookup(Addr address)
{
    assert(address == makeLineAddress(address));
    int loc = addressToCacheIndex(address);
    if (loc == -1) return NULL;
    return m_cache[loc];
}

// looks an address up in the cache
const AbstractCacheEntry*
VictimCache::lookup(Addr address) const
{
    assert(address == makeLineAddress(address));
    int loc = addressToCacheIndex(address);
    if (loc == -1) return NULL;
    return m_cache[loc];
}


void
VictimCache::print(std::ostream& out) const
{
    out << "Cache dump: " << name() << m_num_security_domain << std::endl;
    out << "Active Threshold: " << std::endl;
    for (int i = 0; i < m_num_security_domain; i++){
	out << active_threshold[i] << " ";
    }
    out << std::endl;
    for (int i = 0; i < m_cache_assoc; i++) {
        if (m_cache[i] != NULL) {
            out << "  Index: " << i;
            out << " entry address: " <<std::hex<< m_cache[i]->m_Address;
	    for(int j = 0; j < m_num_security_domain; j++){
		out <<" "<< m_security_domian[j][i] << " ";
	    }
	    out << std::endl;
        } else {
            out << "  Index: " << i
                << " entry: NULL" << std::endl;
        }
    }
}

void
VictimCache::printData(std::ostream& out) const
{
    out << "printData() not supported" << std::endl;
}



VictimCache::
VictimCacheStats::VictimCacheStats(statistics::Group *parent, int num)
    : statistics::Group(parent), 

      ADD_STAT(m_demand_hits, "Number of cache demand hits"),
      ADD_STAT(m_demand_misses, "Number of cache demand misses"),
      ADD_STAT(m_demand_accesses, "Number of cache demand accesses",
               m_demand_hits + m_demand_misses),
      ADD_STAT(m_buffered_blocks, "Number of blocks pass the secure placement policy"),
      ADD_STAT(m_LLC_evictionss, "Number of LLC evictions SS state"),
      ADD_STAT(m_LLC_evictionm, "Number of LLC evictions M state"),
      ADD_STAT(m_LLC_evictionmt, "Number of LLC evictions MT states")
{
   for(int i = 0; i < num; i++){
      m_owned_blocks.push_back(new statistics::Scalar(this));
      m_owned_blocks[i]
	->name(csprintf("OwnedBlcoks_The%dthSecurityDomain", i))
	.desc("")
	.flags(statistics::nozero);
   }
}

void
VictimCache::profileDemandHit()
{
    victimCacheStats.m_demand_hits++;
}

void
VictimCache::profileDemandMiss()
{
    victimCacheStats.m_demand_misses++;
}

void
VictimCache::profileBufferedBlocks()
{
    victimCacheStats.m_buffered_blocks++;
}

void
VictimCache::profileOwnedBlocks(int i)
{
    (*victimCacheStats.m_owned_blocks[i])++;
}

void
VictimCache::profileLLCEvictionss()
{
    victimCacheStats.m_LLC_evictionss++;
}

void
VictimCache::profileLLCEvictionm()
{
    victimCacheStats.m_LLC_evictionm++;
}

void
VictimCache::profileLLCEvictionmt()
{
    victimCacheStats.m_LLC_evictionmt++;
}

} // namespace ruby
} // namespace gem5
