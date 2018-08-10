/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a base set associative tag store.
 */

#include "mem/cache/tags/base_set_assoc.hh"

#include <string>

#include "base/intmath.hh"

BaseSetAssoc::BaseSetAssoc(const Params *p)
    :BaseTags(p), assoc(p->assoc), allocAssoc(p->assoc),
     blks(p->size / p->block_size),
     numSets(p->size / (p->block_size * p->assoc)),
     sequentialAccess(p->sequential_access),
     sets(p->size / (p->block_size * p->assoc)),
     replacementPolicy(p->replacement_policy)
{
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }
    if (!isPowerOf2(numSets)) {
        fatal("# of sets must be non-zero and a power of 2");
    }
    if (assoc <= 0) {
        fatal("associativity must be greater than zero");
    }

    setShift = floorLog2(blkSize);
    setMask = numSets - 1;
    tagShift = setShift + floorLog2(numSets);

    unsigned blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].assoc = assoc;

        sets[i].blks.resize(assoc);

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // Select block within the set to be linked
            BlkType*& blk = sets[i].blks[j];

            // Locate next cache block
            blk = &blks[blkIndex];

            // Associate a data chunk to the block
            blk->setDataPtr(&dataBlks[blkSize*blkIndex]);
            blk->setOwner(this);

            // Associate a replacement data entry to the block
            blk->replacementData = replacementPolicy->instantiateEntry();

            // Setting the tag to j is just to prevent long chains in the
            // hash table; won't matter because the block is invalid
            blk->tag = j;

            // Set its set and way
            blk->set = i;
            blk->way = j;

            // Update block index
            ++blkIndex;
        }
    }
}

void
BaseSetAssoc::invalidate(CacheBlk *blk)
{
    BaseTags::invalidate(blk);

    // Decrease the number of tags in use
    tagsInUse--;

    // Invalidate replacement data
    replacementPolicy->invalidate(blk->replacementData);
}

CacheBlk*
BaseSetAssoc::findBlock(Addr addr, bool is_secure) const
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag, is_secure);
    return blk;
}

ReplaceableEntry*
BaseSetAssoc::findBlockBySetAndWay(int set, int way) const
{
    return sets[set].blks[way];
}

BaseSetAssoc *
BaseSetAssocParams::create()
{
    return new BaseSetAssoc(this);
}


CacheBlk*
BaseSetAssoc::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    BlkType *blk = findBlock(addr, is_secure);

    // Access all tags in parallel, hence one in each way.  The data side
    // either accesses all blocks in parallel, or one block sequentially on
    // a hit.  Sequential access with a miss doesn't access data.
    tagAccesses += allocAssoc;
    if (sequentialAccess) {
        if (blk != nullptr) {
            dataAccesses += 1;
        }
    } else {
        dataAccesses += allocAssoc;
    }

    if (blk != nullptr) {
        // If a cache hit
        lat = accessLatency;
        // Check if the block to be accessed is available. If not,
        // apply the accessLatency on top of block->whenReady.
        if (blk->whenReady > curTick() &&
            cache->ticksToCycles(blk->whenReady - curTick()) >
            accessLatency) {
            lat = cache->ticksToCycles(blk->whenReady - curTick()) +
                accessLatency;
        }

        // Update number of references to accessed block
        blk->refCount++;

        // Update replacement data of accessed block
        replacementPolicy->touch(blk->replacementData);
    } else {
        // If a cache miss
        lat = lookupLatency;
    }

    return blk;
}

CacheBlk*
BaseSetAssoc::findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const
{
    // Get possible locations for the victim block
    std::vector<CacheBlk*> locations = getPossibleLocations(addr);

    // Choose replacement victim from replacement candidates
    CacheBlk* victim = static_cast<CacheBlk*>(
        replacementPolicy->getVictim(
            std::vector<ReplaceableEntry*>(
                locations.begin(), locations.end())));

    // There is only one eviction for this replacement
    evict_blks.push_back(victim);

    DPRINTF(CacheRepl, "set %x, way %x: selecting blk for replacement\n",
            victim->set, victim->way);

    return victim;
}

const std::vector<CacheBlk*>
BaseSetAssoc::getPossibleLocations(Addr addr) const
{
    return sets[extractSet(addr)].blks;
}

void
BaseSetAssoc::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    // Insert block
    BaseTags::insertBlock(pkt, blk);

    // Increment tag counter
    tagsInUse++;

    // Update replacement policy
    replacementPolicy->reset(blk->replacementData);
}

void
BaseSetAssoc::setWayAllocationMax(int ways)
{
    fatal_if(ways < 1, "Allocation limit must be greater than zero");
    allocAssoc = ways;
}

int
BaseSetAssoc::getWayAllocationMax() const
{
    return allocAssoc;
}

Addr
BaseSetAssoc::extractTag(Addr addr) const
{
    return (addr >> tagShift);
}

Addr
BaseSetAssoc::regenerateBlkAddr(const CacheBlk* blk) const
{
    return ((blk->tag << tagShift) | ((Addr)blk->set << setShift));
}

void
BaseSetAssoc::forEachBlk(std::function<void(CacheBlk &)> visitor)
{
    for (CacheBlk& blk : blks) {
        visitor(blk);
    }
}

bool
BaseSetAssoc::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    for (CacheBlk& blk : blks) {
        if (visitor(blk)) {
            return true;
        }
    }
    return false;
}

int
BaseSetAssoc::extractSet(Addr addr) const
{
    return ((addr >> setShift) & setMask);
}
