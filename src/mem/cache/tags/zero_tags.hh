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

#ifndef __MEM_CACHE_TAGS_ZERO_TAGS_HH__
#define __MEM_CACHE_TAGS_ZERO_TAGS_HH__

#include "mem/cache/tags/base_set_assoc.hh"
#include "mem/cache/zero_cache.hh"
#include "mem/packet.hh"
#include "params/ZeroCacheDataTags.hh"
#include "params/ZeroTags.hh"

class CacheBlk;
class ZeroBlk;
class ZeroTags;

class ZeroCacheDataTags : public BaseSetAssoc
{
  protected:
    const uint8_t *zeroBlk;

  public:
    ZeroCacheDataTags(const ZeroCacheDataTagsParams *p);
    ~ZeroCacheDataTags();
    void trySatisfyMigration(CacheBlk *blk) override;
};

class ZeroTags : public BaseSetAssoc
{
  protected:
    // unsigned zeroSetShift;
    // unsigned zeroTagShift;
    // unsigned zeroSetMask;
    const unsigned zeroBlkSize;
    std::vector<ZeroBlk> zeroBlks;
    const unsigned numZeroSets;
    std::vector<CacheSet<ZeroBlk>> zeroSets;
    unsigned zeroSetShift;
    unsigned zeroSetMask;
    unsigned zeroTagShift;
    ZeroCache *_cache;
    Addr zeroTagRegionStart;
    Addr zeroTagRegionEnd;

  public:
    ZeroTags(const ZeroTagsParams *p);
    ~ZeroTags();

    // ZeroBlk *loadZeroBlock(Addr addr);
    void invalidateZeroBlock(ZeroBlk *blk);
    void insertZeroBlock(const PacketPtr pkt, ZeroBlk *blk);
    ZeroBlk *accessZeroBlock(Addr addr, bool is_secure, Cycles &lat);
    ZeroBlk *findZeroBlock(Addr addr, bool is_secure) const;
    ZeroBlk *findZeroVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const;
    const std::vector<ZeroBlk*> getPossibleZeroLocations(Addr addr) const;

    Addr extractTag(Addr addr) const override;
    unsigned extractZeroSet(Addr addr) const;
    Addr extractZeroTag(Addr addr) const;
    Addr regenerateBlkAddr(const CacheBlk* blk) const override;
    /**
     * Converts the address for a zero-tag block
     * (i.e. ZeroBlk::getAddr, or the regenerateBlkAddr(zeroBlk)
     * above) into the first address that the given zero-tag block is
     * tagging (i.e. the address for the first zero-tag entry; the
     * next entry tags the address at the next data cache block size,
     * as a data-cache block size is the smalles tagged unit).
     */
    Addr tagToBlockAddr(Addr tagAddr) const;

    unsigned getZeroBlockSize() const;

    void trySatisfyMigration(CacheBlk *blk) override;

  protected:
    /* TODO: stats */
};

#endif //__MEM_CACHE_TAGS_ZERO_TAGS_HH__
