/*
 * Copyright (c) 2018 Robert Kovacsics
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
 *
 * Authors: Robert Kovacsics
 */

#include "mem/cache/zero_cache.hh"

#include <algorithm>

#include "base/logging.hh"
#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"

ZeroCache::ZeroCache(const ZeroCacheParams *p)
    : Cache(p),
      zeroBlockSize(p->zero_block_size),
      zeroTagRegionStart(p->zero_tag_region_start),
      zeroTagRegionEnd(p->zero_tag_region_end)
{
    zeroTags = dynamic_cast<ZeroTags*>(p->tags);
    assert(zeroTags != nullptr);
    // Even though BaseCache does this, we need to do this again, as
    // in the BaseCache constructor, the type of this is BaseCache*,
    // and cannot be dynamic_cast to ZeroCache*.
    zeroTags->setZeroCache(this);
}

ZeroCache::~ZeroCache()
{
}

bool
ZeroCache::isZeroTagAddr(Addr addr) const
{
    return zeroTagRegionStart <= addr && addr < zeroTagRegionEnd;
}

Addr
ZeroCache::tagToDataAddr(TagAddr tag_addr)
{
    return (tag_addr.addr-zeroTagRegionEnd)*zeroBlockSize;
}

TagAddr
ZeroCache::dataToTagAddr(Addr data_addr)
{
    Addr tag_addr = zeroTagRegionStart + data_addr/zeroBlockSize;
    assert(isZeroTagAddr(tag_addr));
    return (TagAddr){tag_addr};
}

TagAddr
ZeroCache::alignTagAddrToBlock(TagAddr tag_addr)
{
    return (TagAddr){tag_addr.addr-(tag_addr.addr % zeroBlockSize)};
}

bool
ZeroCache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                  PacketList &writebacks)
{
    return Cache::access(pkt, blk, lat, writebacks);
}

CacheBlk *
ZeroCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    CacheBlk *allocatedBlock = Cache::allocateBlock(pkt, writebacks);
    return allocatedBlock;
}

void
ZeroCache::satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                          bool deferred_response, bool pending_downgrade)
{
    Cache::satisfyRequest(pkt, blk, deferred_response, pending_downgrade);
}

void
ZeroCache::invalidateBlock(CacheBlk *blk)
{
    Cache::invalidateBlock(blk);
}

void
ZeroCache::zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk)
{
}

/** Takes a packet addressed at data in the cache, and returns a
 * packet which reads the corresponding zero-tag in the cache.  The
 * use of this is to load the zero-tags into the cache. */
void
ZeroCache::regStats()
{
    Cache::regStats();
    zeroReplacements
        .name(name() + ".zeroReplacements")
        .desc("number of zero-tag replacements")
        ;
}

ZeroCache*
ZeroCacheParams::create()
{
    return new ZeroCache(this);
}
