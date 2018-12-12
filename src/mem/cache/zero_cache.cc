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
#include "debug/ZeroCache.hh"
#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"

ZeroCache::ZeroCache(const ZeroCacheParams *p)
    : Cache(p),
      zeroBlockSize(p->zero_block_size),
      zeroTagRegion(p->zero_tag_region)
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
    return zeroTagRegion.contains(addr);
}

Addr
ZeroCache::tagToDataAddr(TagAddr tag_addr)
{
    return (tag_addr.addr-zeroTagRegion.end())*zeroBlockSize;
}

TagAddr
ZeroCache::dataToTagAddr(Addr data_addr)
{
    Addr tag_addr = zeroTagRegion.start() + data_addr/zeroBlockSize;
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
    Addr addr = pkt->getAddr();
    if (isZeroTagAddr(addr)){
        // No need to fetch zero-tags, as zero-tags themselves aren't
        // tagged
        return Cache::access(pkt, blk, lat, writebacks);
    } else {
        // Ensure that the corresponding tag is in the cache, by
        // making an access to it.
        // TODO: Filter out accesses which would not insert/access a
        // block
        CacheBlk *zblk = nullptr;
        PacketPtr read_tag_pkt = packetToReadTagPacket(pkt);
        Cycles zero_lat = lat;
        bool zero_satisfied =
            Cache::access(read_tag_pkt, zblk, zero_lat, writebacks);
        bool satisfied = Cache::access(pkt, blk, lat, writebacks);
        // Simulate these happening in parallel
        lat = lat < zero_lat ? zero_lat : lat;
        if (!zero_satisfied) {
            // handleTimingReqMiss doesn't handle a zero-tag miss (as
            // we used Cache::access not Cache::recvTimingReq [because
            // the latter doesn't return whether it succeeded]) so we
            // do that here.
            MSHR *mshr = mshrQueue.findMatch(read_tag_pkt->getAddr(),
                                             pkt->isSecure());
            if (!mshr) {
                Tick forward_time = clockEdge(forwardLatency) +
                    pkt->headerDelay;
                mshr = allocateMissBuffer(read_tag_pkt, forward_time);
            }
        }
        return satisfied && zero_satisfied;
    }
}

CacheBlk *
ZeroCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    const Addr data_addr = pkt->getAddr();
    CacheBlk *allocatedBlock = Cache::allocateBlock(pkt, writebacks);
    /// The request into the tag region should ensure that the zero
    /// block is in the cache
    if (allocatedBlock && !isZeroTagAddr(data_addr)) {
        const TagAddr tag_addr = dataToTagAddr(data_addr);
        const bool is_secure = pkt->isSecure();
        ZeroBlk *zeroBlock = zeroTags->findZeroBlock(tag_addr, is_secure);
        // We always keep the zero-tag inclusive of the data blocks,
        // by fetching the zero-tag before fetching the data
        assert(zeroBlock);
        zeroBlock->setEntryWay(tag_addr, allocatedBlock->way);
    }

    return allocatedBlock;
}

void
ZeroCache::satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                          bool deferred_response, bool pending_downgrade)
{
    Addr data_addr = pkt->getAddr();
    if (pkt->req->isRegionZero()) {
        zeroRegionInstr(pkt, blk);
    } else {
        Cache::satisfyRequest(pkt, blk, deferred_response, pending_downgrade);
    }
    if (!isZeroTagAddr(data_addr)) {
        TagAddr tag_addr = dataToTagAddr(data_addr);
        ZeroBlk *zblk = zeroTags->findZeroBlock(tag_addr, pkt->isSecure());
        if (pkt->isRead() && zblk) {
            if (zblk->isEntryZero(tag_addr)) {
                std::memset(pkt->getPtr<uint8_t*>(), 0, pkt->getSize());
            }
        }
    }
}

void
ZeroCache::invalidateBlock(CacheBlk *blk)
{
    Addr data_addr = regenerateBlkAddr(blk);
    Cache::invalidateBlock(blk);
    if (!isZeroTagAddr(data_addr)) {
        TagAddr tag_addr = dataToTagAddr(data_addr);
        ZeroBlk *zblk = zeroTags->findZeroBlock(tag_addr, blk->isSecure());
        assert(zblk); // Zero-tags should be inclusive of data blocks
        zblk->invalidateEntry(tag_addr);
    }
}

void
ZeroCache::zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk)
{
    /// TODO: Ensure zero-block is loaded
    DPRINTF(ZeroCache, "Zeroing memory from 0x%08x of size 0x%08x\n",
            pkt->getAddr(),
            *(uint64_t*)pkt->getConstDataPtr());

    // TODO: RMK35-base: Zero region; invalidate blocks
    Addr start = pkt->getAddr();
    Addr size = (Addr)*(uint64_t*)pkt->getConstDataPtr();
    Addr end = start + size;
    bool is_secure = pkt->isSecure();
    unsigned zero_block_coverage =
        /* One bit of the zero block is one data block many bytes */
        getBlockSize() *
        /* And one zero block is below many bits */
        zeroBlockSize * 8;
    /* Assert that the zeroing is data cache block aligned */
    assert(end%getBlockSize() == 0);

    // First invalidate all the lines, to help the cache avoid
    // evicting blocks to free up space, which would then be zeroed
    // anyway
    for (Addr zblk_addr = start; zblk_addr < end;
         zblk_addr += zero_block_coverage) {
        Addr last_blk_addr = std::min(zblk_addr + zero_block_coverage, end);
        for (Addr blk_addr = zblk_addr; blk_addr < last_blk_addr;
             blk_addr += getBlockSize()) {
            CacheBlk *blk = tags->findBlock(blk_addr, is_secure);
            if (blk) invalidateBlock(blk);
        }
    }

    for (Addr zblk_addr = start; zblk_addr < end;
         zblk_addr += zero_block_coverage) {

        ZeroBlk *zblk = findZeroBlock(dataToTagAddr(zblk_addr),
                                      pkt->isSecure());
        // The zero block should always be in the cache, because
        // ZeroCache::access will detect that the address of this is
        // in the non zero-tag region, so generate a request for the
        // corresponding zero-tag, and only do
        // ZeroCache::satisfyRequest (which calls zeroRegionInstr)
        // when the zero-tag is in the cache
        assert(zblk);
        Addr last_blk_addr = std::min(zblk_addr + zero_block_coverage, end);

        for (Addr blk_addr = zblk_addr; blk_addr < last_blk_addr;
             blk_addr += getBlockSize()) {
            zblk->setEntryZero(dataToTagAddr(blk_addr), true);
        }
    }
}

/** Takes a packet addressed at data in the cache, and returns a
 * packet which reads the corresponding zero-tag in the cache.  The
 * use of this is to load the zero-tags into the cache. */
PacketPtr
ZeroCache::packetToReadTagPacket(PacketPtr pkt)
{
    Addr data_addr = pkt->getAddr();
    TagAddr tag_addr = dataToTagAddr(data_addr);
    tag_addr = alignTagAddrToBlock(tag_addr);
    RequestPtr req = std::make_shared<Request>(tag_addr.addr,
                                               zeroBlockSize,
                                               /* flags */ 0,
                                               pkt->req->masterId(),
                                               pkt->req->time());
    req->setExtraData(data_addr);
    Packet *zero_tag_pkt = new Packet(req, MemCmd::ReadZeroTag);
    zero_tag_pkt->allocate();
    return zero_tag_pkt;
}

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
