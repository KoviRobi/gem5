// TODO licence header

#include "mem/cache/zero_cache.hh"

#include "base/logging.hh"
#include "debug/ZeroCache.hh"
#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"
#include <algorithm>

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

Addr
ZeroCache::tagToDataAddr(TagAddr tag_addr)
{
    return (tag_addr.addr-zeroTagRegionEnd)*zeroBlockSize;
}

TagAddr
ZeroCache::dataToTagAddr(Addr data_addr)
{
    Addr tag_addr = zeroTagRegionStart + data_addr/zeroBlockSize;
    assert(zeroTagRegionStart <= tag_addr && tag_addr < zeroTagRegionEnd);
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
    if (addr < zeroTagRegionStart || zeroTagRegionEnd <= addr){
        // TODO: Filter out accesses which would not insert/access a block
        // Ensure that the corresponding tag is in the cache, by
        // making an access to it.
        CacheBlk *zblk = nullptr;
        PacketPtr read_tag_pkt = packetToReadTagPacket(pkt);
        Cycles zero_lat = lat;
        bool zero_satisfied =
            Cache::access(read_tag_pkt, zblk, zero_lat, writebacks);
        bool satisfied = Cache::access(pkt, blk, lat, writebacks);
        // Simulate these happening in parallel
        lat = lat < zero_lat ? zero_lat : lat;
        if (!zero_satisfied) {
            // TODO: RMK35
            MSHR *mshr = mshrQueue.findMatch(read_tag_pkt->getAddr(),
                                             pkt->isSecure());
            if (!mshr) {
                Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;
                mshr = allocateMissBuffer(read_tag_pkt, forward_time);
            }
        }
        return satisfied && zero_satisfied;
    } else {
        return Cache::access(pkt, blk, lat, writebacks);
    }
}

CacheBlk *
ZeroCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    const Addr data_addr = pkt->getAddr();
    CacheBlk *allocatedBlock = Cache::allocateBlock(pkt, writebacks);
    /// The request into the tag region should ensure that the zero
    /// block is in the cache
    if (data_addr < zeroTagRegionStart || data_addr >= zeroTagRegionEnd) {
        const TagAddr tag_addr = dataToTagAddr(data_addr);
        const bool is_secure = pkt->isSecure();
        ZeroBlk *zeroBlock = zeroTags->findZeroBlock(tag_addr, is_secure);
        if (allocatedBlock && zeroBlock) {
            zeroBlock->setEntryWay(tag_addr, allocatedBlock->way);
        }
    }

    return allocatedBlock;
}

CacheBlk *
ZeroCache::handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                      bool allocate)
{
    return Cache::handleFill(pkt, blk, writebacks, allocate);
    // Addr addr = pkt->getAddr();
    // bool is_secure = pkt->isSecure();
    // if (std::memcmp(blk, zeroBlk, getBlockSize())) {
    //     ZeroBlk *zblk = zeroTags->findZeroBlock(addr, is_secure);
    //     if (!zblk) {
    //         ZeroBlock *zblk = allocate ? allocateZeroBlock(pkt, blk, writebacks)
    //                                    : nullptr;
    //         if (!blk) {
    //             return nullptr;
    //         }
    //     }

    //     blk->setZero(addr, true);
    //     blk->setValid(addr, true);
    // } else {
    //     CacheBlk *filled_blk = Cache::handleFill(pkt, blk, writebacks, allocate);
    //     ZeroBlk *zblk = zeroTags->findZeroBlock(addr, is_secure);

    //     if (!zblk) {
    //         if (allocate) allocateZeroBlock(pkt, writebacks);
    //         // zblk will be set valid when the response comes back
    //         return filled_blk;
    //     }

    //     if (filled_blk && zblk) {
    //         zblk->setValid(addr, true);
    //         zblk->setWay(addr, filled_blk->way);
    //     }
    //     return zblk && filled_blk;
    // }
    // return nullptr;
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
    if (data_addr < zeroTagRegionStart || data_addr >= zeroTagRegionEnd) {
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
    if (data_addr < zeroTagRegionStart || data_addr >= zeroTagRegionEnd) {
        TagAddr tag_addr = dataToTagAddr(data_addr);
        ZeroBlk *zblk = zeroTags->findZeroBlock(tag_addr, blk->isSecure());
        /// TODO: If we have proper inclusivity, we could just assert!
        if (zblk) zblk->invalidateEntry(tag_addr);
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

        // TODO: Check that zero_block_coverage increments the tag addr by 1
        ZeroBlk *zblk = zeroTags->findZeroBlock(dataToTagAddr(zblk_addr),
                                      pkt->isSecure());
        // The zero block should always be in the cache, because TODO: RMK35
        assert(zblk);
        Addr last_blk_addr = std::min(zblk_addr + zero_block_coverage, end);

        for (Addr blk_addr = zblk_addr; blk_addr < last_blk_addr;
             blk_addr += getBlockSize()) {
            zblk->setEntryZero(dataToTagAddr(blk_addr), true);
        }
    }
}

// ZeroBlk *
// ZeroCache::allocateZeroBlock(const PacketPtr pkt, PacketList &writebacks)
// {
//     // TODO: Is this true? This doesn't load anything from memory
//     // Load in from closer to the memory; Evicting a zero
//     // block might cause us to evict (or write-back) data
//     // blocks, as we keep the zero-tag blocks inclusive of the
//     // data blocks
//     Addr addr = pkt->getAddr();
//     bool is_secure = pkt->isSecure();

//     // if (memSidePort.sendTimingReq(pkt)) {
//     //     TODO;
//     // }

//     std::vector<CacheBlk*> evict_blks;
//     ZeroBlk *victim = zeroTags->findZeroVictim(addr, is_secure, evict_blks,
//                                                tags);
//     if (!victim) return nullptr;
//     if (transientEvicts(evict_blks)) return nullptr;

//     // The victim will be replaced by a new entry, so increase the replacement
//     // counter if a valid block is being replaced
//     if (victim->isValid()) {
//         DPRINTF(Cache, "zero replacement: replacing %#llx (%s) with %#llx "
//                 "(%s): %s\n", regenerateBlkAddr(victim),
//                 victim->isSecure() ? "s" : "ns",
//                 addr, is_secure ? "s" : "ns",
//                 victim->isDirty() ? "writeback" : "clean");

//         zeroReplacements++;
//     }

//     for (const auto& blk : evict_blks) {
//         if (blk->isValid()) {
//             if (blk->wasPrefetched()) {
//                 unusedPrefetches++;
//             }

//             evictBlock(blk, writebacks);
//         }
//     }

//     // Insert new block at victimized entry
//     zeroTags->insertZeroBlock(pkt->getAddr(), pkt->req->masterId(), victim);

//     return victim;
// }

// ZeroBlk *
// ZeroCache::findOrAllocateZeroBlock(const PacketPtr pkt, PacketList &writebacks)
// {
//     Addr data_addr = pkt->getAddr();
//     Addr tag_addr = dataToTagAddr(data_addr);
//     bool is_secure = pkt->isSecure();
//     ZeroBlk *zblk = zeroTags->findZeroBlock(tag_addr, is_secure);
//     if (!zblk) {
//         allocateZeroBlock(pkt, writebacks);
//     }
//     return zblk;
// }

/// Takes a packet addressed at data in the cache, and returns a
/// packet which reads the corresponding zero-tag in the cache.
/// The use of this is to load the zero-tags into the cache.
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
