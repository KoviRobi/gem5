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

#include "base/statistics.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/tags/base_set_assoc.hh"
#include "mem/cache/zero_cache.hh"
#include "mem/packet.hh"
#include "params/ZeroTags.hh"

class CacheBlk;
class ZeroBlk;
class ZeroTags;

class ZeroTags : public BaseSetAssoc
{
  protected:
    const unsigned zeroBlkSize;
    std::vector<ZeroBlk> zeroBlks;
    std::unique_ptr<uint8_t[]> zeroDataBlks;
    const unsigned numZeroSets;
    std::vector<CacheSet<ZeroBlk>> zeroSets;
    /// Note, these are for *tag addresses*
    /// (i.e. getZeroCache()->zeroTagRegionStart <= addr &&
    /// addr < getZeroCache()->zeroTagRegionEnd)
    unsigned zeroSetShift;
    unsigned zeroSetMask;
    unsigned zeroTagShift;
    ZeroCache *_zeroCache;

    /** Replacement policy */
    BaseReplacementPolicy *zeroReplacementPolicy;

  public:
    ZeroTags(const ZeroTagsParams *p);
    ~ZeroTags();

    /// This is different from the setCache, rather than overriding
    /// it, for a somewhat contrived reason: the BaseCache constructor
    /// calls BaseTags::setCache(this), and in that BaseCache
    /// constructor, the type of this is BaseCache*, and cannot be
    /// cast to ZeroCache*.
    virtual void setZeroCache(ZeroCache *cache);
    ZeroCache *getZeroCache() const;

    /// Override the old behaviour if it is accessing the tag region
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const override;
    CacheBlk *accessBlock(Addr addr, bool is_secure, Cycles &lat) override;
    CacheBlk *findBlock(Addr addr, bool is_secure) const override;
    void invalidate(CacheBlk *blk) override;
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override;

    /**
     * Print all tags used
     */
    std::string print() override;
    /** Debug print (DPRINTF), because of debugger escaping \n and \t
    */
    virtual void dprint();

    /** 'Inserts' a zero block (sets all the entries to false), and
     * sets is_secure, tag, etc.  After you do this, use
     * blk->setEntryValid/setEntryWay/setEntryZero
     * @param tagAddr indicates whether the address of pkt is a tag
     * addr, or a data addr (i.e. is between
     * getZeroCache()->zeroTagRegionStart/zeroTagRegionEnd) */
    virtual void insertZeroBlock(const PacketPtr pkt, ZeroBlk *blk);
    virtual ZeroBlk *accessZeroBlock(TagAddr tag_addr, bool is_secure, Cycles &lat);
    virtual ZeroBlk *findZeroBlock(TagAddr tag_addr, bool is_secure) const;
    virtual void invalidateZeroBlock(ZeroBlk *blk);
    virtual ZeroBlk *findZeroVictim(TagAddr tag_addr, const bool is_secure,
                            std::vector<CacheBlk*>& evict_blks) const;
    virtual const std::vector<ZeroBlk*>
    getPossibleZeroLocations(TagAddr tag_addr) const;

    virtual unsigned extractZeroSet(TagAddr addr) const;
    virtual Addr extractZeroTag(TagAddr addr) const;
    virtual TagAddr regenerateZeroBlkAddr(const ZeroBlk* blk) const;

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override;
    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override;
    virtual void forEachZeroBlk(std::function<void(ZeroBlk &)> visitor);

    virtual unsigned getZeroBlockSize() const;

    void trySatisfyMigration(CacheBlk *blk) override;

  public: /// Statistics
    void regStats() override;
  protected: /// Statistics
    Stats::AverageVector zeroTagOccupancies;
    Stats::Average zeroTagsInUse;
    Stats::Scalar zeroTagAccesses;
    Stats::Scalar zeroMigrations;
};

#endif //__MEM_CACHE_TAGS_ZERO_TAGS_HH__
