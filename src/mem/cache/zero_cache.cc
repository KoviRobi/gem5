// TODO licence header

#include "mem/cache/zero_cache.hh"

#include "base/logging.hh"
#include "debug/ZeroCache.hh"
#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"
#include <algorithm>

ZeroCache::ZeroCache(const ZeroCacheParams *p)
    : Cache(p), zeroTags(p->zero_tags),
      zeroBlockSize(p->zero_block_size),
      zeroTagRegionStart(p->zero_tag_region_start),
      zeroTagRegionEnd(p->zero_tag_region_end)
{
    zeroTags->setCache(this);
}

ZeroCache::~ZeroCache()
{
}

bool
ZeroCache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                  PacketList &writebacks)
{
    if (pkt->req->isRegionZero())
        return zeroRegionInstr(pkt, blk, lat, writebacks);
    return Cache::access(pkt, blk, lat, writebacks);
    // if (isInZeroTagRegion(pkt->getAddr()))
    //     return zeroTagAccessToMemoryAccess(pkt, blk, lat, writebacks);

    // ZeroBlk *zblk = zeroTags->accessZeroBlock(pkt->getAddr(),
    //                                            pkt->isSecure(), lat);
    // bool satisfied = Cache::access(pkt, blk, lat, writebacks);
    // // If we have inserted a new block, make sure it has a
    // // corresponding zero-tag block which is aware of it
    // if (pkt->isWriteback() || pkt->cmd == MemCmd::WriteClean) {
    //     if (!zblk) {
    //         zblk = allocateZeroBlock(pkt, writebacks);
    //         // TODO:
    //         // // Done by handleTimingReqMiss
    //         // return false;
    //     } else {
    //         zblk->setWay(pkt->getAddr(), blk->way);
    //     }
    //     // If we have written data to the cache, it might cause a
    //     // block to be moved between the zero-tag and the data caches
    //     doMigrations(writebacks);
    // }
    // return satisfied;
}

CacheBlk *
ZeroCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    return Cache::allocateBlock(pkt, writebacks);
    // // TODO
    // const Addr addr = pkt->getAddr();
    // const bool is_secure = pkt->isSecure();
    // Cycles lat;

    // ZeroBlk *zeroBlk = zeroTags->findZeroBlock(addr, is_secure, lat);
    // CacheBlk *allocatedBlock = Cache::allocateBlock(pkt, writebacks);
    // if (zeroBlk && allocateBlock) {
    //     // If the zero block is already in there, set the way
    //     zeroBlk->setWay(addr, blk->way);
    //     return allocatedBlock;
    // } else if (allocatedBlock) {
    //     // Need to allocate a zero block
    //     std::vector<CacheBlk*> evict_blks;
    //     ZeroBlk *zeroVictim = zeroTags->findZeroVictim(addr, is_secure,
    //                                                    evict_blks);
    //     // A zero block should only point have zeros, which should not
    //     // be in the data cache, or non-zeros which may, and may need
    //     // to be evicted to keep inclusivity
    //     for (const auto& non_zero_blk : zeroVictim->nonZeros(tags)) {
    //         if (non_zero_blk->isValid()) {
    //             evict_blks.push_back(non_zero_blk);
    //         }
    //     }

    //     // Check for transient state allocations. If any of the entries listed
    //     // for eviction has a transient state, the allocation fails
    //     if (transientEvicts(evict_blks))) return nullptr;

    //     // The victim will be replaced by a new entry, so increase the replacement
    //     // counter if a valid block is being replaced
    //     if (zeroVictim->isValid()) {
    //         DPRINTF(Cache, "zero replacement: replacing %#llx (%s) with %#llx "
    //                 "(%s): %s\n", regenerateBlkAddr(zeroVictim),
    //                 victim->isSecure() ? "s" : "ns",
    //                 addr, is_secure ? "s" : "ns",
    //                 victim->isDirty() ? "writeback" : "clean");

    //         replacements++;
    //     }

    //     // Evict valid blocks associated to this victim block
    //     for (const auto& blk : evict_blks) {
    //         if (blk->isValid()) {
    //             if (blk->wasPrefetched()) {
    //                 unusedPrefetches++;
    //             }

    //             evictBlock(blk, writebacks);
    //         }
    //     }

    //     zeroTags->insertZeroBlock(TODO, zeroVictim);

    //     return allocatedBlock();
    // }

    // return nullptr;
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
    ZeroBlk *zblk = zeroTags->findZeroBlock(pkt->getAddr(), pkt->isSecure());
    Cache::satisfyRequest(pkt, blk, deferred_response, pending_downgrade);
    if (pkt->isRead() && zblk) {
        if (zblk->isEntryZero(pkt->getAddr()))
            std::memset(pkt->getPtr<uint8_t*>(), 0, pkt->getSize());
    }
}

void
ZeroCache::recvTimingResp(PacketPtr pkt)
{
    return Cache::recvTimingResp(pkt);
    // If we get a response to our zero-tags read.
}

void
ZeroCache::handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                               Tick forward_time, Tick request_time)
{
    return Cache::handleTimingReqMiss(pkt, blk, forward_time, request_time);
    // TODO
}

void
ZeroCache::invalidateBlock(CacheBlk *blk)
{
    Addr addr = regenerateBlkAddr(blk);
    ZeroBlk *zblk = zeroTags->findZeroBlock(addr, blk->isSecure());
    if (zblk) zblk->setEntryValid(addr, false);
    Cache::invalidateBlock(blk);
}

bool
ZeroCache::doMigrations(PacketList &writebacks)
{
    // for (const auto &now_zero : migrateToZeroBlocks) {
    //     Addr addr = regenerateBlkAddr(now_zero);
    //     bool is_secure = now_zero->isSecure();
    //     ZeroBlk *zblk = zeroTags->findZeroBlock(addr, is_secure);
    //     assert(zblk != nullptr); // Should be ensured by zero-tag inclusitivity
    //     assert(blk->isValid());
    //     zblk->setZero(addr, true);
    //     zblk->setValid(addr, true);
    // }
    // for (const auto &now_nonzero : migrateToDataBlocks) {
    //     Addr addr = regenerateBlkAddr(now_nonzero);
    //     bool is_secure = now_nonzero->isSecure();
    //     ZeroBlk *zblk = zeroTags->findZeroBlock(addr, is_secure);
    //     assert(zblk != nullptr); // Should be ensured by zero-tag inclusitivity
    //     zblk->setZero(addr, false);
    //     zblk->setValid(addr, now_nonzero->isValid());
    // }
    return true;
}

bool
ZeroCache::zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                      PacketList writebacks)
{
    DPRINTF(ZeroCache, "Zeroing memory from 0x%08x of size 0x%08x\n",
            pkt->getAddr(),
            *(uint64_t*)pkt->getConstDataPtr());

    // TODO: RMK35-base: Zero region; invalidate blocks
    Addr start = pkt->getAddr();
    Addr end = start + *(uint64_t*)pkt->getConstDataPtr();
    bool is_secure = pkt->isSecure();
    unsigned zero_block_coverage = zeroBlockSize * getBlockSize();
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

        ZeroBlk *zblk = findOrAllocateZeroBlock(pkt, writebacks);
        Addr last_blk_addr = std::min(zblk_addr + zero_block_coverage, end);
        if (!zblk) continue;

        for (Addr blk_addr = zblk_addr; blk_addr < last_blk_addr;
             blk_addr += getBlockSize()) {
            zblk->setEntryZero(blk_addr, true);
        }
    }
    return true;
}

bool
ZeroCache::zeroTagAccessToMemoryAccess(PacketPtr pkt, CacheBlk *blk,
                                       Cycles lat, PacketList &writebacks)
{
    assert(pkt->getSize() == zeroBlockSize);
    assert(pkt->getAddr());
    if (pkt->isRead()) {
    } else if (pkt->isWrite()) {
    } else {
        fatal("Unsupported action on zero tag memory region.");
    }
    return true;
}

ZeroBlk *
ZeroCache::allocateZeroBlock(const PacketPtr pkt, PacketList &writebacks)
{
    // Load in from closer to the memory; Evicting a zero
    // block might cause us to evict (or write-back) data
    // blocks, as we keep the zero-tag blocks inclusive of the
    // data blocks
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();

    std::vector<CacheBlk*> evict_blks;
    ZeroBlk *victim = zeroTags->findZeroVictim(addr, is_secure, evict_blks);
    if (!victim) return nullptr;
    if (transientEvicts(evict_blks)) return nullptr;

    // The victim will be replaced by a new entry, so increase the replacement
    // counter if a valid block is being replaced
    if (victim->isValid()) {
        DPRINTF(Cache, "replacement: replacing %#llx (%s) with %#llx "
                "(%s): %s\n", regenerateBlkAddr(victim),
                victim->isSecure() ? "s" : "ns",
                addr, is_secure ? "s" : "ns",
                victim->isDirty() ? "writeback" : "clean");

        zeroReplacements++;
    }

    for (const auto& blk : evict_blks) {
        if (blk->isValid()) {
            if (blk->wasPrefetched()) {
                unusedPrefetches++;
            }

            evictBlock(blk, writebacks);
        }
    }

    // Insert new block at victimized entry
    zeroTags->insertBlock(pkt, victim);

    return victim;
}

ZeroBlk *
ZeroCache::findOrAllocateZeroBlock(const PacketPtr pkt, PacketList &writebacks)
{
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
    ZeroBlk *zblk = zeroTags->findZeroBlock(addr, is_secure);
    if (!zblk) {
        allocateZeroBlock(pkt, writebacks);
    }
    return zblk;
}

bool
ZeroCache::isInZeroTagRegion(Addr addr)
{
    return zeroTagRegionStart <= addr && addr < zeroTagRegionEnd;
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
