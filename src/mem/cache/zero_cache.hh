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

#ifndef __MEM_CACHE_ZERO_CACHE_HH__
#define __MEM_CACHE_ZERO_CACHE_HH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "params/ZeroCache.hh"

class CacheBlk;
class ZeroBlk;
class TempZeroBlk;
class ZeroTags;

typedef struct { Addr addr; } TagAddr;

class ZeroCache : public Cache
{
  protected:
    ZeroTags *zeroTags;
    TempZeroBlk *tempZeroBlock;
    unsigned zeroBlockSize;
    AddrRange zeroTagRegion;
    uint8_t *blockOfZeros;

  public:
    ZeroCache(const ZeroCacheParams *p);
    virtual ~ZeroCache();

    virtual bool isZeroTagAddr(Addr addr) const;
    Addr tagToDataAddr(TagAddr tag_addr);
    TagAddr dataToTagAddr(Addr data_addr);
    TagAddr alignTagAddrToBlock(TagAddr tag_addr);

    virtual void zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk);
    virtual bool accessZeroTag(PacketPtr pkt, ZeroBlk *&blk,
                               Cycles &lat, PacketList &writebacks);
    virtual void maintainZeroClusivity(ZeroBlk *zblk);
    /* Wrapper around looking in the tags and using the temp zero
    block */
    virtual ZeroBlk *
    findZeroBlock(TagAddr tag_addr, bool is_secure) const;

    PacketPtr packetToReadTagPacket(PacketPtr pkt);

    void recvTimingReq(PacketPtr pkt) override;

    /** This is updated for:
     * 1. If the request is a zero-tag access, handle it specially
     * 2. If the block is not in the normal cache, but is backed by a
     *    virtual zero-block (as the zero-tag cache indicates the
     *    block is all zero), use that. It may allocate a block, if it
     *    is a write to something that was previously all-zero.
     */
    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;

    /*
     * Handle a timing request that missed in the cache
     *
     * This is modified so that it adds the data-load request as a
     * deferred target if we are also waiting on the zero-tag.
     *
     * @param ptk The request packet
     * @param blk The referenced block
     * @param forward_time The tick at which we can process dependent requests
     * @param request_time The tick at which the block lookup is compete
     */
    void handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk,
                             Tick forward_time,
                             Tick request_time) override;

    /**
     * Take an MSHR, turn it into a suitable downstream packet, and
     * send it out. This construct allows a queue entry to choose a suitable
     * approach based on its type.
     *
     * Overridden, to deal with the case when the MSHR is from a
     * missed zero-tag request, but the packet was a deferred target
     * to the normal (non-zero-tag) region.
     *
     * @param mshr The MSHR to turn into a packet and send
     * @return True if the port is waiting for a retry
     */
    bool sendMSHRQueuePacket(MSHR* mshr) override;

    /** For updating zero-tags when loading in the program to run */
    ///xxxvoid functionalAccess(PacketPtr pkt, bool from_cpu_side) override;
    CacheBlk *allocateBlock(const PacketPtr pkt,
                            PacketList &writebacks) override;
    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response = false,
                        bool pending_downgrade = false) override;
    void invalidateBlock(CacheBlk *blk) override;
    /**
     * These are overridden to deal with tempBlock/tempZeroBlk
     * @{
     */
    void recvTimingResp(PacketPtr pkt) override;
    Tick recvAtomic(PacketPtr pkt) override;
    CacheBlk*
    handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
               bool allocate) override;
    //TODO: RMK35: void evictBlock(CacheBlk *blk, PacketList &writebacks) override;
    // TODO: Snoop requests
    void maintainClusivity(bool from_cache, CacheBlk *blk) override;
    /**@}*/

    void regStats() override;

  protected: /// Statistics
    Stats::Scalar zeroReplacements;
};

#endif // __MEM_CACHE_ZERO_CACHE_HH__
