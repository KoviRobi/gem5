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

#ifndef __MEM_CACHE_ZER_CACHEH__
#define __MEM_CACHE_ZER_CACHEH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "params/ZeroCache.hh"

class CacheBlk;
class ZeroBlk;
class ZeroTags;

typedef struct { Addr addr; } TagAddr;

class ZeroCache : public Cache
{
  protected:
    ZeroTags *zeroTags;
    unsigned zeroBlockSize;
    AddrRange zeroTagRegion;

  public:
    ZeroCache(const ZeroCacheParams *p);
    virtual ~ZeroCache();

    virtual bool isZeroTagAddr(Addr addr) const;
    Addr tagToDataAddr(TagAddr tag_addr);
    TagAddr dataToTagAddr(Addr data_addr);
    TagAddr alignTagAddrToBlock(TagAddr tag_addr);

    virtual void zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk);

    PacketPtr packetToReadTagPacket(PacketPtr pkt);

    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;
    CacheBlk *allocateBlock(const PacketPtr pkt,
                            PacketList &writebacks) override;
    void satisfyRequest(PacketPtr pkt, CacheBlk *blk, bool deferred_response,
                        bool pending_downgrade) override;
    void invalidateBlock(CacheBlk *blk) override;

    void regStats() override;

  protected: /// Statistics
    Stats::Scalar zeroReplacements;
};

#endif // __MEM_CACHE_ZER_CACHEH__
