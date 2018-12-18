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
#include "debug/Cache.hh"
#include "debug/CacheVerbose.hh"
#include "debug/ZeroCache.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"

ZeroCache::ZeroCache(const ZeroCacheParams *p)
    : Cache(p),
      zeroBlockSize(p->zero_block_size),
      zeroTagRegion(p->zero_tag_region),
      blockOfZeros(new uint8_t[blkSize])
{
    zeroTags = dynamic_cast<ZeroTags*>(p->tags);
    assert(zeroTags);
    zeroTags->setZeroCache(this);
    tempZeroBlock = new TempZeroBlk(zeroBlockSize);
    tempZeroBlock->setOwner(zeroTags);
}

ZeroCache::~ZeroCache()
{
    delete tempZeroBlock;
    delete [] blockOfZeros;
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

void
ZeroCache::recvTimingReq(PacketPtr pkt)
{
    // anything that is merely forwarded pays for the forward latency and
    // the delay provided by the crossbar
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    // We use lookupLatency here because it is used to specify the latency
    // to access.
    Cycles lat = lookupLatency;
    Cycles zero_tag_lat = lookupLatency;
    CacheBlk *blk = nullptr;
    ZeroBlk *zblk = nullptr;
    bool satisfied = false;
    bool zero_tag_satisfied = true;
    PacketPtr read_tag_pkt = nullptr;
    {
        PacketList writebacks;
        if (!pkt->isZeroTagAccess() &&
            (pkt->isRead() || pkt->isWrite())) {
            // In timing, deleting this packet is the responsibility
            // of the receiver
            read_tag_pkt = packetToReadTagPacket(pkt);
            // Note that zero_tag_lat is passed by reference here. The
            // function access() calls accessBlock() which can modify
            // lat value.
            zero_tag_satisfied =
                accessZeroTag(read_tag_pkt, zblk, zero_tag_lat, writebacks);
        }

        // Note that lat is passed by reference here. The function
        // access() calls accessBlock() which can modify lat value.
        satisfied = access(pkt, blk, lat, writebacks);

        // Simulate both zero-tag and data access happening in
        // parallel
        lat = lat < zero_tag_lat ? zero_tag_lat : lat;

        // copy writebacks to write buffer here to ensure they logically
        // precede anything happening below
        doWritebacks(writebacks, forward_time);
    }

    // Here we charge the headerDelay that takes into account the latencies
    // of the bus, if the packet comes from it.
    // The latency charged it is just lat that is the value of lookupLatency
    // modified by access() function, or if not just lookupLatency.
    // In case of a hit we are neglecting response latency.
    // In case of a miss we are neglecting forward latency.
    Tick request_time = clockEdge(lat) + pkt->headerDelay;
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;
    // track time of availability of next prefetch, if any
    Tick next_pf_time = MaxTick;

    if (satisfied && zero_tag_satisfied) {
        // if need to notify the prefetcher we have to do it before
        // anything else as later handleTimingReqHit might turn the
        // packet in a response.
        // We chose not to notify the prefetcher about zero-tag
        // accesses.
        if (prefetcher && pkt && !pkt->isZeroTagAccess() &&
            (prefetchOnAccess || (blk && blk->wasPrefetched()))) {
            if (blk)
                blk->status &= ~BlkHWPrefetched;

            // Don't notify on SWPrefetch
            if (!pkt->cmd.isSWPrefetch()) {
                assert(!pkt->req->isCacheMaintenance());
                next_pf_time = prefetcher->notify(pkt);
            }
        }

        handleTimingReqHit(pkt, blk, request_time);
    }

    Tick data_request_time = request_time;
    if (!zero_tag_satisfied) {
        handleTimingReqMiss(read_tag_pkt, zblk, forward_time, request_time);
        data_request_time += forward_time;
    }

    if (!satisfied) {
        handleTimingReqMiss(pkt, blk, forward_time, data_request_time);

        // We should call the prefetcher reguardless if the request is
        // satisfied or not, reguardless if the request is in the MSHR
        // or not. The request could be a ReadReq hit, but still not
        // satisfied (potentially because of a prior write to the same
        // cache line. So, even when not satisfied, there is an MSHR
        // already allocated for this, we need to let the prefetcher
        // know about the request

        // Don't notify prefetcher on SWPrefetch or cache maintenance
        // operations, or on zero-tag accesses, as they would reduce
        // the utility of the prefetcher
        if (prefetcher && pkt && !pkt->isZeroTagAccess() &&
            !pkt->cmd.isSWPrefetch() &&
            !pkt->req->isCacheMaintenance()) {
            next_pf_time = prefetcher->notify(pkt);
        }
    }

    if (next_pf_time != MaxTick) {
        schedMemSideSendEvent(next_pf_time);
    }
}

bool
ZeroCache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                  PacketList &writebacks)
{
    if (pkt->isZeroTagAccess()) {
        ZeroBlk *zblk = nullptr;
        bool rtn = accessZeroTag(pkt, zblk, lat, writebacks);
        blk = zblk;
        return rtn;
    } else {
        // Perhaps we have not normal data block in the cache,
        // because we know it is all zero (from the zero-tag
        // cache), and thus don't actually store a line full of
        // zeros.
        // The latency for accessing the zero block is already paid by
        // the recvTimingReq.
        Addr data_addr = pkt->getAddr();
        TagAddr tag_addr = dataToTagAddr(data_addr);
        bool is_secure = pkt->isSecure();
        ZeroBlk *zblk = findZeroBlock(tag_addr, is_secure);

        // Can't satisfy access if we don't have zero-tag block
        if (!zblk) return false;

        bool entry_zero = zblk->isEntryZero(tag_addr);
        // entry_zero => no normal block
        assert(!entry_zero ||
               !tags->findBlock(data_addr,is_secure));

        if (false/*XXX get zero-tags properly working first*/&&entry_zero && pkt->isRead()) {
            // TODO: RMK35: Load linked -- store conditional
            assert(pkt->hasRespData());
            pkt->setDataFromBlock(blockOfZeros, pkt->getSize());
            return true;
        } else if (false/*XXX get zero-tags properly working first*/&&entry_zero) {
            // Insert a cache-block full of zero, to satisfy the
            // request
#if TRACING_ON
            CacheBlk::State old_state = blk ? blk->status : 0;
#endif

            // When handling a fill, we should have no writes to this line.
            assert(data_addr == pkt->getBlockAddr(blkSize));
            assert(!writeBuffer.findMatch(data_addr, is_secure));

            // This only uses the addr, isSecure and masterID of the
            // packet, so OK to use here, even if the packet wouldn't
            // naturally perform insertions
            blk = allocateBlock(pkt, writebacks);

            // Could use a temporary block, but I haven't figured that
            // out fully
            assert(blk);

            blk->status |= BlkValid | BlkReadable;

            DPRINTF(ZeroCache, "Block addr %#llx (%s) moving from state %x to %s\n",
                    data_addr, is_secure ? "s" : "ns", old_state, blk->print());

            blk->setData(blockOfZeros, blkSize);
            return Cache::access(pkt, blk, lat, writebacks);
        } else {
            // Entry is not zero according to the zero-tags, proceed
            // as normal.
            return Cache::access(pkt, blk, lat, writebacks);
        }
    }
}

void
ZeroCache::handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                               Tick forward_time, Tick request_time)
{
    //xxxif(true)return Cache::handleTimingReqMiss(pkt,blk,forward_time,request_time);
    /// Functionality copied from Cache @{ {{{
    if (pkt->req->isUncacheable()) {
        // ignore any existing MSHR if we are dealing with an
        // uncacheable request

        // should have flushed and have no valid block
        assert(!blk || !blk->isValid());

        mshr_uncacheable[pkt->cmdToIndex()][pkt->req->masterId()]++;

        if (pkt->isWrite()) {
            allocateWriteBuffer(pkt, forward_time);
        } else {
            assert(pkt->isRead());

            // uncacheable accesses always allocate a new MSHR

            // Here we are using forward_time, modelling the latency of
            // a miss (outbound) just as forwardLatency, neglecting the
            // lookupLatency component.
            allocateMissBuffer(pkt, forward_time);
        }

        return;
    }

    Addr blk_addr = pkt->getBlockAddr(blkSize);
    /// }}} @}

    /// Modified functionality
    MSHR *mshr = nullptr;
    if (isZeroTagAddr(blk_addr)) {
        mshr = mshrQueue.findMatch(blk_addr, pkt->isSecure());
    } else {
        TagAddr tag_addr = dataToTagAddr(blk_addr);
        TagAddr blk_tag_addr = alignTagAddrToBlock(tag_addr);
        mshr = mshrQueue.findMatch(blk_tag_addr.addr,
                                   pkt->isSecure());
        if (!mshr) {
            mshr = mshrQueue.findMatch(blk_addr, pkt->isSecure());
        }
    }

    /// Functionality copied from Cache @{ {{{
    // Software prefetch handling:
    // To keep the core from waiting on data it won't look at
    // anyway, send back a response with dummy data. Miss handling
    // will continue asynchronously. Unfortunately, the core will
    // insist upon freeing original Packet/Request, so we have to
    // create a new pair with a different lifecycle. Note that this
    // processing happens before any MSHR munging on the behalf of
    // this request because this new Request will be the one stored
    // into the MSHRs, not the original.
    if (pkt->cmd.isSWPrefetch()) {
        assert(pkt->needsResponse());
        assert(pkt->req->hasPaddr());
        assert(!pkt->req->isUncacheable());

        // There's no reason to add a prefetch as an additional target
        // to an existing MSHR. If an outstanding request is already
        // in progress, there is nothing for the prefetch to do.
        // If this is the case, we don't even create a request at all.
        PacketPtr pf = nullptr;

        if (!mshr) {
            // copy the request and create a new SoftPFReq packet
            RequestPtr req = std::make_shared<Request>(pkt->req->getPaddr(),
                                                       pkt->req->getSize(),
                                                       pkt->req->getFlags(),
                                                       pkt->req->masterId());
            pf = new Packet(req, pkt->cmd);
            pf->allocate();
            assert(pf->getAddr() == pkt->getAddr());
            assert(pf->getSize() == pkt->getSize());
        }

        pkt->makeTimingResponse();

        // request_time is used here, taking into account lat and the delay
        // charged if the packet comes from the xbar.
        cpuSidePort.schedTimingResp(pkt, request_time, true);

        // If an outstanding request is in progress (we found an
        // MSHR) this is set to null
        pkt = pf;
    }
    /// }}} @}

    BaseCache::handleTimingReqMiss(pkt, mshr, blk, forward_time, request_time);
}

bool
ZeroCache::sendMSHRQueuePacket(MSHR* mshr)
{
    /// From Cache::sendMSHRQueuePacket(MSHR* mshr) @{ {{{
    assert(mshr);

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;

    if (tgt_pkt->cmd == MemCmd::HardPFReq && forwardSnoops) {
        DPRINTF(Cache, "%s: MSHR %s\n", __func__, tgt_pkt->print());

        // we should never have hardware prefetches to allocated
        // blocks
        assert(!tags->findBlock(tgt_pkt->getAddr(), mshr->isSecure));

        // We need to check the caches above us to verify that
        // they don't have a copy of this block in the dirty state
        // at the moment. Without this check we could get a stale
        // copy from memory that might get used in place of the
        // dirty one.
        Packet snoop_pkt(tgt_pkt, true, false);
        snoop_pkt.setExpressSnoop();
        // We are sending this packet upwards, but if it hits we will
        // get a snoop response that we end up treating just like a
        // normal response, hence it needs the MSHR as its sender
        // state
        snoop_pkt.senderState = mshr;
        cpuSidePort.sendTimingSnoopReq(&snoop_pkt);

        // Check to see if the prefetch was squashed by an upper cache (to
        // prevent us from grabbing the line) or if a Check to see if a
        // writeback arrived between the time the prefetch was placed in
        // the MSHRs and when it was selected to be sent or if the
        // prefetch was squashed by an upper cache.

        // It is important to check cacheResponding before
        // prefetchSquashed. If another cache has committed to
        // responding, it will be sending a dirty response which will
        // arrive at the MSHR allocated for this request. Checking the
        // prefetchSquash first may result in the MSHR being
        // prematurely deallocated.
        if (snoop_pkt.cacheResponding()) {
            auto M5_VAR_USED r = outstandingSnoop.insert(snoop_pkt.req);
            assert(r.second);

            // if we are getting a snoop response with no sharers it
            // will be allocated as Modified
            bool pending_modified_resp = !snoop_pkt.hasSharers();
            markInService(mshr, pending_modified_resp);

            DPRINTF(Cache, "Upward snoop of prefetch for addr"
                    " %#x (%s) hit\n",
                    tgt_pkt->getAddr(), tgt_pkt->isSecure()? "s": "ns");
            return false;
        }

        if (snoop_pkt.isBlockCached()) {
            DPRINTF(Cache, "Block present, prefetch squashed by cache.  "
                    "Deallocating mshr/packet target %#x/%#x.\n",
                    mshr->blkAddr, tgt_pkt->getAddr());

            // Deallocate the mshr target
            if (mshrQueue.forceDeallocateTarget(mshr)) {
                // Clear block if this deallocation resulted freed an
                // mshr when all had previously been utilized
                clearBlocked(Blocked_NoMSHRs);
            }

            // given that no response is expected, delete Request and Packet
            delete tgt_pkt;

            return false;
        }
    }

    /// }}} @}

    /// From BaseCache::sendMSHRQueuePacket(mshr); @{ {{{
    // also above: assert(mshr);

    // use request from 1st target
    // also above: PacketPtr tgt_pkt = mshr->getTarget()->pkt;

    DPRINTF(Cache, "%s: MSHR %s\n", __func__, tgt_pkt->print());

    Addr blk_addr = tgt_pkt->getBlockAddr(blkSize);
    CacheBlk *blk = tags->findBlock(blk_addr, tgt_pkt->isSecure());

    // either a prefetch that is not present upstream, or a normal
    // MSHR request, proceed to get the packet to send downstream
    PacketPtr pkt = createMissPacket(tgt_pkt, blk, mshr->needsWritable());

    mshr->isForward = (pkt == nullptr);

    if (mshr->isForward) {
        // not a cache block request, but a response is expected
        // make copy of current packet to forward, keep current
        // copy for response handling
        pkt = new Packet(tgt_pkt, false, true);
        assert(!pkt->isWrite());
    }

    // play it safe and append (rather than set) the sender state,
    // as forwarded packets may already have existing state
    pkt->pushSenderState(mshr);

    if (pkt->isClean() && blk && blk->isDirty()) {
        // A cache clean opearation is looking for a dirty block. Mark
        // the packet so that the destination xbar can determine that
        // there will be a follow-up write packet as well.
        pkt->setSatisfied();
    }

    if (!memSidePort.sendTimingReq(pkt)) {
        // we are awaiting a retry, but we
        // delete the packet and will be creating a new packet
        // when we get the opportunity
        delete pkt;

        // note that we have now masked any requestBus and
        // schedSendEvent (we will wait for a retry before
        // doing anything), and this is so even if we do not
        // care about this packet and might override it before
        // it gets retried
        return true;
    } else {
        // As part of the call to sendTimingReq the packet is
        // forwarded to all neighbouring caches (and any caches
        // above them) as a snoop. Thus at this point we know if
        // any of the neighbouring caches are responding, and if
        // so, we know it is dirty, and we can determine if it is
        // being passed as Modified, making our MSHR the ordering
        // point
        bool pending_modified_resp = !pkt->hasSharers() &&
            pkt->cacheResponding();
        markInService(mshr, pending_modified_resp);

        if (pkt->isClean() && blk && blk->isDirty()) {
            // A cache clean opearation is looking for a dirty
            // block. If a dirty block is encountered a WriteClean
            // will update any copies to the path to the memory
            // until the point of reference.
            DPRINTF(CacheVerbose, "%s: packet %s found block: %s\n",
                    __func__, pkt->print(), blk->print());
            PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(),
                                             pkt->id);
            PacketList writebacks;
            writebacks.push_back(wb_pkt);
            doWritebacks(writebacks, 0);
        }

        return false;
    }
    /// }}} @}
}

/**
 * NOTE: For functional accesses, we hack the ZeroTagAccess flag of
 * the request to mean that we have done one, even if this access
 * isn't actually a zero-tag access. I.e. if a cache we visited
 * earlier has done an access for the tags.
 */
///xxxvoid
///xxxZeroCache::functionalAccess(PacketPtr pkt, bool from_cpu_side)
///xxx{
///xxx    if (!pkt->isZeroTagAccess() && pkt->isWrite() && pkt->isRequest()) {
///xxx        pkt->req->setFlags(Request::ZERO_TAG_ACCESS);
///xxx        Packet read_tag_pkt = XXX;
///xxx        functionalAccess(read_tag_pkt, from_cpu_side);
///xxx        Packet update_tag_pkt = XXX;
///xxx        update_tag_pkt->setData(read_tag_pkt->getConstDataPtr(), size, offset);
///xxx        XXX set bits
///xxx        functionalAccess(update_tag_pkt, from_cpu_side);
///xxx    }
///xxx    Cache::functionalAccess(pkt, from_cpu_side);
///xxx}

bool
ZeroCache::accessZeroTag(PacketPtr pkt, ZeroBlk *&zblk, Cycles &lat,
                         PacketList &writebacks)
{
    zblk = zeroTags->accessZeroBlock(
        (TagAddr){pkt->getAddr()},
        pkt->isSecure(), lat);
    DPRINTF(ZeroCache, "%s for %s %s\n", __func__, pkt->print(),
            zblk ? "hit " + zblk->print() : "miss");
    if (zblk) {
        if (pkt->needsWritable() ? zblk->isWritable() :
                                   zblk->isReadable()) {
            incHitCount(pkt);
            satisfyRequest(pkt, zblk);
            maintainZeroClusivity(zblk);
            return true;
        } else {
            panic("Not yet implemented in %s (zero-tag cache): %s\n",
                  __func__, pkt->print());
        }
    } else {
        incMissCount(pkt);
        return false;
    }
}

void
ZeroCache::maintainZeroClusivity(ZeroBlk *zblk)
{
    if (zblk && zblk->isValid()) {
        // TODO: RMK35: Kick normal blocks which are also
        // zero
    }
}

ZeroBlk *
ZeroCache::findZeroBlock(TagAddr tag_addr, bool is_secure) const
{
    ZeroBlk *zblk = zeroTags->findZeroBlock(tag_addr, is_secure);
    if (!zblk &&
        tempZeroBlock->isValid() &&
         (tempZeroBlock->getAddr() ==
          // Align to block size
          tag_addr.addr - (tag_addr.addr%zeroBlockSize)) &&
         tempZeroBlock->isSecure() == is_secure) {
        zblk = tempZeroBlock;
    }
    return zblk;
}

CacheBlk *
ZeroCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks)
{
    assert((pkt->isZeroTagAccess() && isZeroTagAddr(pkt->getAddr())) ||
           (!pkt->isZeroTagAccess() && !isZeroTagAddr(pkt->getAddr())));
    CacheBlk *allocatedBlock = Cache::allocateBlock(pkt, writebacks);
    /// The request into the tag region should ensure that the zero
    /// block is in the cache
    if (false/*XXX*/&&allocatedBlock && !pkt->isZeroTagAccess()) {
        const Addr data_addr = pkt->getAddr();
        const TagAddr tag_addr = dataToTagAddr(data_addr);
        const bool is_secure = pkt->isSecure();
        ZeroBlk *zeroBlock = findZeroBlock(tag_addr, is_secure);
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
    if (pkt->req->isRegionZero()) {
        zeroRegionInstr(pkt, blk);
    } else if (pkt->isZeroTagAccess()) {
        assert(pkt->isRequest());
        assert(blk && blk->isValid());
        if (pkt->isWrite()) {
            assert(blk->isWritable());
            // TODO: Should never need, as cannot do LL/SC?
            // Or can we; what happens if we store a zero,
            // and that ends up upgrading the line?
            if (blk->checkWrite(pkt)) {
                blk->setDataFromPacket(pkt, zeroBlockSize);
            }
            blk->status |= BlkDirty;
            DPRINTF(ZeroCache, "%s for %s (write)\n", __func__, pkt->print());
        } else if (pkt->isRead()) {
            assert(pkt->hasRespData());
            pkt->setDataFromBlock(blk->getConstDataPtr(), blkSize);
        }
    } else {
        Cache::satisfyRequest(pkt, blk, deferred_response, pending_downgrade);
        Addr data_addr = pkt->getAddr();
        TagAddr tag_addr = dataToTagAddr(data_addr);
        ZeroBlk *zblk = findZeroBlock(tag_addr, pkt->isSecure());
        if (zblk && pkt->isRead()) {
            if (zblk->isEntryZero(tag_addr)) {
                // XXX YYY TODO: RMK35 std::memset(pkt->getPtr<uint8_t*>(), 0, pkt->getSize());
            }
        }
    }
}

void
ZeroCache::invalidateBlock(CacheBlk *blk)
{
    ZeroBlk *zblk = dynamic_cast<ZeroBlk*>(blk);
    if (zblk) {
        DPRINTF(ZeroCache, "%s: %s\n", __func__, blk->print());
    }
    if (false/*XXX*/&&!zblk && blk->isValid()) {
        Addr data_addr = regenerateBlkAddr(blk);
        assert(!isZeroTagAddr(data_addr));
        TagAddr tag_addr = dataToTagAddr(data_addr);
        ZeroBlk *tag_blk = findZeroBlock(tag_addr, blk->isSecure());
        assert(tag_blk && tag_blk->isValid()); // Zero-tags should be inclusive of data blocks
        tag_blk->invalidateEntry(tag_addr);
        tag_blk->status |= BlkDirty;
    }
    Cache::invalidateBlock(blk);
}

void
ZeroCache::maintainClusivity(bool from_cache, CacheBlk *blk)
{
    ZeroBlk * zblk = dynamic_cast<ZeroBlk*>(blk);
    if (zblk) {
        // TODO: RMK35
    } else
        Cache::maintainClusivity(from_cache, blk);
}

// TODO: RMK35
void
ZeroCache::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    // all header delay should be paid for by the crossbar, unless
    // this is a prefetch response from above
    panic_if(pkt->headerDelay != 0 && pkt->cmd != MemCmd::HardPFResp,
             "%s saw a non-zero packet delay\n", name());

    const bool is_error = pkt->isError();

    if (is_error) {
        DPRINTF(Cache, "%s: Cache received %s with error\n", __func__,
                pkt->print());
    }

    DPRINTF(Cache, "%s: Handling response %s\n", __func__,
            pkt->print());

    // if this is a write, we should be looking at an uncacheable
    // write
    if (pkt->isWrite()) {
        assert(pkt->req->isUncacheable());
        handleUncacheableWriteResp(pkt);
        return;
    }

    // we have dealt with any (uncacheable) writes above, from here on
    // we know we are dealing with an MSHR due to a miss or a prefetch
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->popSenderState());
    assert(mshr);

    if (mshr == noTargetMSHR) {
        // we always clear at least one target
        clearBlocked(Blocked_NoTargets);
        noTargetMSHR = nullptr;
    }

    // Initial target is used just for stats
    MSHR::Target *initial_tgt = mshr->getTarget();
    int stats_cmd_idx = initial_tgt->pkt->cmdToIndex();
    Tick miss_latency = curTick() - initial_tgt->recvTime;

    if (pkt->req->isUncacheable()) {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_uncacheable_lat[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    } else {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_miss_latency[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    }

    PacketList writebacks;

    bool is_fill = !mshr->isForward &&
        (pkt->isRead() || pkt->cmd == MemCmd::UpgradeResp);

    CacheBlk *blk;
    if (pkt->isZeroTagAccess()) {
        Addr addr = pkt->getAddr();
        assert(isZeroTagAddr(addr));
        TagAddr tag_addr = (TagAddr){addr};
        blk = zeroTags->findZeroBlock(tag_addr, pkt->isSecure());
    } else {
        blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());
    }

    if (is_fill && !is_error) {
        DPRINTF(Cache, "Block for addr %#llx being updated in Cache\n",
                pkt->getAddr());

        blk = handleFill(pkt, blk, writebacks, mshr->allocOnFill());
        assert(blk != nullptr);
    }

    if (blk && blk->isValid() && pkt->isClean() && !pkt->isInvalidate()) {
        // The block was marked not readable while there was a pending
        // cache maintenance operation, restore its flag.
        blk->status |= BlkReadable;

        // This was a cache clean operation (without invalidate)
        // and we have a copy of the block already. Since there
        // is no invalidation, we can promote targets that don't
        // require a writable copy
        mshr->promoteReadable();
    }

    if (blk && blk->isWritable() && !pkt->req->isCacheInvalidate()) {
        // If at this point the referenced block is writable and the
        // response is not a cache invalidate, we promote targets that
        // were deferred as we couldn't guarrantee a writable copy
        mshr->promoteWritable();
    }

    serviceMSHRTargets(mshr, pkt, blk, writebacks);

    if (mshr->promoteDeferredTargets()) {
        // avoid later read getting stale data while write miss is
        // outstanding.. see comment in timingAccess()
        if (blk) {
            //blk->status &= ~BlkReadable;
        }
        mshrQueue.markPending(mshr);
        schedMemSideSendEvent(clockEdge() + pkt->payloadDelay);
    } else {
        // while we deallocate an mshr from the queue we still have to
        // check the isFull condition before and after as we might
        // have been using the reserved entries already
        const bool was_full = mshrQueue.isFull();
        mshrQueue.deallocate(mshr);
        if (was_full && !mshrQueue.isFull()) {
            clearBlocked(Blocked_NoMSHRs);
        }

        // Request the bus for a prefetch if this deallocation freed enough
        // MSHRs for a prefetch to take place
        if (prefetcher && mshrQueue.canPrefetch()) {
            Tick next_pf_time = std::max(prefetcher->nextPrefetchReadyTime(),
                                         clockEdge());
            if (next_pf_time != MaxTick)
                schedMemSideSendEvent(next_pf_time);
        }
    }

    // if we used temp block, check to see if its valid and then clear it out
    if (blk == tempBlock && tempBlock->isValid()) {
        evictBlock(blk, writebacks);
        if (tempZeroBlock->isValid()) {
            evictBlock(tempZeroBlock, writebacks);
        }
    }

    const Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // copy writebacks to write buffer
    doWritebacks(writebacks, forward_time);

    DPRINTF(CacheVerbose, "%s: Leaving with %s\n", __func__, pkt->print());
    delete pkt;
}

Tick
ZeroCache::recvAtomic(PacketPtr pkt)
{
    panic("Not yet implemented: %s\n", __func__);
}

CacheBlk*
ZeroCache::handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                      bool allocate)
{
    assert(pkt->isResponse() || pkt->cmd == MemCmd::WriteLineReq);
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
#if TRACING_ON
    CacheBlk::State old_state = blk ? blk->status : 0;
#endif

    // When handling a fill, we should have no writes to this line.
    assert(addr == pkt->getBlockAddr(blkSize));
    assert(!writeBuffer.findMatch(addr, is_secure));

    if (!blk) {
        // better have read new data...
        assert(pkt->hasData());

        // only read responses and write-line requests have data;
        // note that we don't write the data here for write-line - that
        // happens in the subsequent call to satisfyRequest
        assert(pkt->isRead() || pkt->cmd == MemCmd::WriteLineReq);

        // need to do a replacement if allocating, otherwise we stick
        // with the temporary storage
        blk = allocate ? allocateBlock(pkt, writebacks) : nullptr;

        if (!blk) {
            // No replaceable block or a mostly exclusive
            // cache... just use temporary storage to complete the
            // current request and then get rid of it
            assert(!tempBlock->isValid());
            if (pkt->isZeroTagAccess()) {
                blk = tempZeroBlock;
            } else {
                blk = tempBlock;
            }
            blk->insert(addr, is_secure, 0, 0);
            DPRINTF(Cache, "using temp block for %#llx (%s)\n", addr,
                    is_secure ? "s" : "ns");
        }

        // we should never be overwriting a valid block
        assert(!blk->isValid());
    } else {
        // existing block... probably an upgrade
        assert(regenerateBlkAddr(blk) == addr);
        assert(blk->isSecure() == is_secure);
        // either we're getting new data or the block should already be valid
        assert(pkt->hasData() || blk->isValid());
        // don't clear block status... if block is already dirty we
        // don't want to lose that
    }

    blk->status |= BlkValid | BlkReadable;

    // sanity check for whole-line writes, which should always be
    // marked as writable as part of the fill, and then later marked
    // dirty as part of satisfyRequest
    if (pkt->cmd == MemCmd::WriteLineReq) {
        assert(!pkt->hasSharers());
    }

    // here we deal with setting the appropriate state of the line,
    // and we start by looking at the hasSharers flag, and ignore the
    // cacheResponding flag (normally signalling dirty data) if the
    // packet has sharers, thus the line is never allocated as Owned
    // (dirty but not writable), and always ends up being either
    // Shared, Exclusive or Modified, see Packet::setCacheResponding
    // for more details
    if (!pkt->hasSharers()) {
        // we could get a writable line from memory (rather than a
        // cache) even in a read-only cache, note that we set this bit
        // even for a read-only cache, possibly revisit this decision
        blk->status |= BlkWritable;

        // check if we got this via cache-to-cache transfer (i.e., from a
        // cache that had the block in Modified or Owned state)
        if (pkt->cacheResponding()) {
            // we got the block in Modified state, and invalidated the
            // owners copy
            blk->status |= BlkDirty;

            chatty_assert(!isReadOnly, "Should never see dirty snoop response "
                          "in read-only cache %s\n", name());
        }
    }

    DPRINTF(Cache, "Block addr %#llx (%s) moving from state %x to %s\n",
            addr, is_secure ? "s" : "ns", old_state, blk->print());

    // if we got new data, copy it in (checking for a read response
    // and a response that has data is the same in the end)
    if (pkt->isRead()) {
        // sanity checks
        assert(pkt->hasData());
        assert(pkt->getSize() == blkSize);

        blk->setDataFromPacket(pkt, blkSize);
    }
    // We pay for fillLatency here.
    blk->whenReady = clockEdge() + fillLatency * clockPeriod() +
        pkt->payloadDelay;

    return blk;
}

void
ZeroCache::zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk)
{
    /// TODO: Ensure zero-block is loaded (e.g. for partial zero)
    DPRINTF(ZeroCache, "Zeroing memory from 0x%08x of size 0x%08x\n",
            pkt->getAddr(),
            *(uint64_t*)pkt->getConstDataPtr());

    // TODO: RMK35-base: Zero region; invalidate blocks
    Addr start = pkt->getAddr();
    Addr size = (Addr)*(uint64_t*)pkt->getConstDataPtr();
    Addr end = start + size;
    bool is_secure = pkt->isSecure();
    // TODO: RMK35: zero-blocks aren't necessarily covering one
    // cache-block
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
                                               Request::ZERO_TAG_ACCESS,
                                               pkt->req->masterId(),
                                               pkt->req->time());
    req->setExtraData(data_addr);
    Packet *zero_tag_pkt = new Packet(req, MemCmd::ReadZeroTag);
    zero_tag_pkt->allocate();
    DPRINTF(ZeroCache, "Created read for zero-tag %s from %s\n",
            zero_tag_pkt->print(),
            pkt->print());
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
