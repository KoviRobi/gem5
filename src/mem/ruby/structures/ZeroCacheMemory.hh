/*
 * Copyright (c) 2018 Robert Kovacsics
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

#ifndef __MEM_RUBY_STRUCTURES_ZEROCACHEMEMORY_HH__
#define __MEM_RUBY_STRUCTURES_ZEROCACHEMEMORY_HH__

#include <string>
#include <unordered_set>
#include <vector>

#include "mem/ruby/structures/CacheMemory.hh"
#include "params/RubyZeroCache.hh"

class ZeroCacheMemory : public CacheMemory
{
  public:
    typedef RubyCacheParams Params;
    ZeroCacheMemory(const Params *p);
    virtual ~ZeroCacheMemory();
    virtual void verifyZero(Addr address, DataBlock &data);
    virtual void migrateZero(Addr address, DataBlock &data);
    virtual AbstractCacheEntry *lookup(Addr address);
    virtual const AbstractCacheEntry *lookup(Addr address) const;
    // find an unused entry and sets the tag appropriate for the address
    virtual AbstractCacheEntry* allocate(Addr address,
                                 AbstractCacheEntry* new_entry, bool touch);
    virtual AbstractCacheEntry* allocate(Addr address,
                                 AbstractCacheEntry* new_entry)
    {
        return allocate(address, new_entry, true);
    }
    virtual void allocateVoid(Addr address, AbstractCacheEntry* new_entry)
    {
        allocate(address, new_entry, true);
    }
    // Explicitly free up this address
    virtual void deallocate(Addr address);
  protected:
    DataBlock zeroData;
    std::set<Addr> zeros;
};

std::ostream& operator<<(std::ostream& out, const ZeroCacheMemory& obj);

#endif // __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__
