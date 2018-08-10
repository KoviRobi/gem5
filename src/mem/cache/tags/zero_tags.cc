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

#include "base/logging.hh"
#include "debug/ZeroTags.hh"

ZeroTagsData::ZeroTagsData(const ZeroTagsDataParams *p) :
    BaseSetAssoc(p), zeroBlk(new uint8_t[getBlockSize()]()) {}
ZeroTagsData::~ZeroTagsData()
{
    delete[] zeroBlk;
}

void
ZeroTagsData::trySatisfyMigration(CacheBlk *blk)
{
    if (std::memcmp(zeroBlk, blk->getConstDataPtr(), getBlockSize())) {
        DPRINTF(ZeroTags, "Detected zero block %s\n", blk->print());
    }
}

ZeroTagsData *
ZeroTagsDataParams::create()
{
    return new ZeroTagsData(this);
}

ZeroTags::ZeroTags(const ZeroTagsParams *p) :
    BaseSetAssoc(p), dataTags(p->data_tags)
{
}

ZeroTags::~ZeroTags()
{
}

void
ZeroTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    dataTags->insertBlock(pkt, blk);
}

void
ZeroTags::invalidate(CacheBlk *blk)
{
    dataTags->invalidate(blk);
}

CacheBlk*
ZeroTags::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    return dataTags->accessBlock(addr, is_secure, lat);
}

CacheBlk*
ZeroTags::findBlock(Addr addr, bool is_secure) const
{
    return dataTags->findBlock(addr, is_secure);
}

CacheBlk*
ZeroTags::findVictim(Addr addr, const bool is_secure,
                     std::vector<CacheBlk*>& evict_blks) const
{
    return dataTags->findVictim(addr, is_secure, evict_blks);
}
const std::vector<CacheBlk*>
ZeroTags::getPossibleLocations(Addr addr) const
{
    return dataTags->getPossibleLocations(addr);
}

void
ZeroTags::setWayAllocationMax(int ways)
{
    dataTags->setWayAllocationMax(ways);
}

int
ZeroTags::getWayAllocationMax() const
{
    return dataTags->getWayAllocationMax();
}

Addr
ZeroTags::extractTag(Addr addr) const
{
    return dataTags->extractTag(addr);
}

Addr
ZeroTags::regenerateBlkAddr(const CacheBlk* blk) const
{
    return dataTags->regenerateBlkAddr(blk);
}

void
ZeroTags::forEachBlk(std::function<void(CacheBlk &)> visitor)
{
    dataTags->forEachBlk(visitor);
}

bool
ZeroTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    return dataTags->anyBlk(visitor);
}

void
ZeroTags::trySatisfyMigration(CacheBlk *blk)
{
}

ZeroTags *
ZeroTagsParams::create()
{
    return new ZeroTags(this);
}
