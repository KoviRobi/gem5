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

#include "mem/cache/tags/zero_tags.hh"

#include <cstring>
#include <vector>

#include "base/logging.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/zero_blk.hh"

ZeroTags::ZeroTags(const ZeroTagsParams *p) :
    BaseSetAssoc(p),
    zeroBlkSize(p->zero_tag_block_size),
    zeroBlks(p->size / zeroBlkSize),
    zeroDataBlks(new uint8_t[p->size]),
    numZeroSets(p->size / (zeroBlkSize * p->assoc)),
    zeroSets(numZeroSets),
    _zeroCache(nullptr)
{
    /// Note, these are for *tag addresses*
    /// (i.e. getZeroCache()->isZeroTagRegion(addr)
    zeroSetShift = floorLog2(zeroBlkSize);
    zeroSetMask = numZeroSets - 1;
    zeroTagShift = zeroSetShift + floorLog2(numZeroSets);

    unsigned blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numZeroSets; ++i) {
        zeroSets[i].assoc = assoc;

        zeroSets[i].blks.resize(assoc);

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // Select block within the set to be linked
            ZeroBlk*& zblk = zeroSets[i].blks[j];

            // Locate next cache block
            zblk = &zeroBlks[blkIndex];

            // Associate a data chunk to the block and take ownership
            zblk->setDataPtr(&zeroDataBlks[getZeroBlockSize()*blkIndex]);
            // Each tag *bit* corresponds to an entry, and
            // getZeroBlockSize() returns bytes of tags
            zblk->setOwner(this);

            // Associate a replacement data entry to the block
            zblk->replacementData = replacementPolicy->instantiateEntry();

            // Setting the tag to j is just to prevent long chains in the
            // hash table; won't matter because the block is invalid
            zblk->tag = j;

            // Set its set and way
            zblk->set = i;
            zblk->way = j;

            // Update block index
            ++blkIndex;
        }
    }
}

ZeroTags::~ZeroTags()
{
}

void
ZeroTags::setZeroCache(ZeroCache *zero_cache)
{
    assert(!_zeroCache);
    _zeroCache = dynamic_cast<ZeroCache*>(zero_cache);
    assert(_zeroCache);
}

ZeroCache *
ZeroTags::getZeroCache() const
{
    return _zeroCache;
}

CacheBlk *
ZeroTags::findVictim(Addr addr, bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const
{
    if (getZeroCache()->isZeroTagAddr(addr)) {
        /// Since addr is already in the tag region, it doesn't need
        /// to be converted
        TagAddr tag_addr = (TagAddr){addr};
        return findZeroVictim(tag_addr, is_secure, evict_blks);
    } else {
        return BaseSetAssoc::findVictim(addr, is_secure, evict_blks);
    }
}

CacheBlk *
ZeroTags::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    if (getZeroCache()->isZeroTagAddr(addr)) {
        /// Since addr is already in the tag region, it doesn't need
        /// to be converted
        TagAddr tag_addr = (TagAddr){addr};
        return accessZeroBlock(tag_addr, is_secure, lat);
    } else {
        return BaseSetAssoc::accessBlock(addr, is_secure, lat);
    }
}

CacheBlk *
ZeroTags::findBlock(Addr addr, bool is_secure) const
{
    if (getZeroCache()->isZeroTagAddr(addr)) {
        /// Since addr is already in the tag region, it doesn't need
        /// to be converted
        TagAddr tag_addr = (TagAddr){addr};
        return findZeroBlock(tag_addr, is_secure);
    } else {
        return BaseSetAssoc::findBlock(addr, is_secure);
    }
}

void
ZeroTags::invalidate(CacheBlk *blk)
{
    Addr addr = blk->getAddr();
    if (getZeroCache()->isZeroTagAddr(addr)) {
        zeroTagsInUse--;
    }
    BaseSetAssoc::invalidate(blk);
}

void
ZeroTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    Addr addr = pkt->getAddr();
    if (getZeroCache()->isZeroTagAddr(addr)) {
        ZeroBlk *zblk = dynamic_cast<ZeroBlk*>(blk);
        assert(zblk != nullptr);
        insertZeroBlock(pkt, zblk);
    } else {
        BaseSetAssoc::insertBlock(pkt, blk);
    }
}

unsigned
ZeroTags::getZeroBlockSize() const
{
    return zeroBlkSize;
}

void
ZeroTags::insertZeroBlock(const PacketPtr pkt, ZeroBlk *blk)
{
    Addr tag_addr = pkt->getAddr();
    assert(getZeroCache()->isZeroTagAddr(tag_addr));
    assert(!blk->isValid());

    // Deal with what we are bringing in
    MasterID master_id = pkt->req->masterId();
    assert(master_id < _cache->system->maxMasters());
    blk->insert(extractZeroTag((TagAddr){tag_addr}), pkt->isSecure(),
                master_id, pkt->req->taskId());

    // Statistics
    zeroTagOccupancies[master_id]++;
    zeroTagsInUse++;
    zeroTagAccesses++;
}

ZeroBlk*
ZeroTags::accessZeroBlock(TagAddr tag_addr, bool is_secure, Cycles &lat)
{
    ZeroBlk *zblk = findZeroBlock(tag_addr, is_secure);
    zeroTagAccesses++;
    if (zblk != nullptr) {
        // If a cache hit
        lat = accessLatency;
        // Check if the block to be accessed is available. If not,
        // apply the accessLatency on top of block->whenReady.
        if (zblk->whenReady > curTick() &&
            _cache->ticksToCycles(zblk->whenReady - curTick()) >
            accessLatency) {
            lat = _cache->ticksToCycles(zblk->whenReady - curTick()) +
                accessLatency;
        }

        replacementPolicy->touch(zblk->replacementData);
    } else {
        lat = lookupLatency;
    }
    return zblk;
}

ZeroBlk *
ZeroTags::findZeroBlock(TagAddr tag_addr, bool is_secure) const
{
    Addr tag = extractZeroTag(tag_addr);
    unsigned set = extractZeroSet(tag_addr);
    ZeroBlk *blk = zeroSets[set].findBlk(tag, is_secure);
    return blk;
}

ZeroBlk*
ZeroTags::findZeroVictim(TagAddr tag_addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const
{
    // Get possible locations for the victim block
    std::vector<ZeroBlk*> locations = getPossibleZeroLocations(tag_addr);

    // Choose replacement victim from replacement candidates
    // TODO: Policy to select block with least valid non-zero data
    ZeroBlk* victim = static_cast<ZeroBlk*>(
        replacementPolicy->getVictim(
            std::vector<ReplaceableEntry*>(
                locations.begin(), locations.end())));

    victim->maintainInclusitivity(this, evict_blks);
    evict_blks.push_back(victim);

    DPRINTF(CacheRepl, "set %x, way %x: selecting blk for replacement\n",
            victim->set, victim->way);

    return victim;
}

const std::vector<ZeroBlk*>
ZeroTags::getPossibleZeroLocations(TagAddr tag_addr) const
{
    return zeroSets[extractZeroSet(tag_addr)].blks;
}

Addr
ZeroTags::extractZeroTag(TagAddr tag_addr) const
{
    return tag_addr.addr >> zeroTagShift;
}

unsigned
ZeroTags::extractZeroSet(TagAddr tag_addr) const
{
    return (tag_addr.addr >> zeroSetShift) & zeroSetMask;
}

TagAddr
ZeroTags::regenerateZeroBlkAddr(const ZeroBlk* blk) const
{
    return (TagAddr){ (blk->tag << zeroTagShift)
            | ((Addr)blk->set << zeroSetShift) };
}

// TODO:
void
ZeroTags::forEachBlk(std::function<void(CacheBlk &)> visitor)
{
    BaseSetAssoc::forEachBlk(visitor);
}

bool
ZeroTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    return BaseSetAssoc::anyBlk(visitor);
}

void
ZeroTags::trySatisfyMigration(CacheBlk *blk)
{
}

/// Statistics
void
ZeroTags::regStats()
{
    BaseTags::regStats();
    zeroTagOccupancies
        .init(_cache->system->maxMasters())
        .name(name() + ".zero_occ_blocks")
        .desc("Average occupied blocks per requestor in the zero-tag cache")
        .flags(Stats::nozero | Stats::nonan);
    zeroTagsInUse
        .name(name() + ".zero_tags_in_use")
        .desc("Cycle average of zero-tags in use");
    zeroTagAccesses
        .name(name() + ".zero_tag_accesses")
        .desc("Number of zero-tag accesses");
    zeroMigrations
        .name(name() + ".zero_migrations")
        .desc("Number of times a cache-line was all zero, and thus moved "
              "to the zero-tag cache");
}

ZeroTags *
ZeroTagsParams::create()
{
    return new ZeroTags(this);
}
