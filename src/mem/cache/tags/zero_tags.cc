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

ZeroTags::ZeroTags(const ZeroTagsParams *p)
    : BaseTags(p), normalDataTags(p->normal_data_tags),
      zeroBlkSize(p->zero_block_size), zeroBlkMask(zeroBlkSize-1)
{
}

void
ZeroTags::setCache(BaseCache *_cache)
{
    BaseTags::setCache(_cache);
    normalDataTags->setCache(_cache);
}

CacheBlk *
ZeroTags::findBlock(Addr addr, bool is_secure) const
{
    return normalDataTags->findBlock(addr, is_secure);
}

void
ZeroTags::invalidate(CacheBlk *blk)
{
    normalDataTags->invalidate(blk);
}

void
ZeroTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    normalDataTags->insertBlock(pkt, blk);
}

Addr
ZeroTags::regenerateBlkAddr(const CacheBlk* blk) const
{
    return normalDataTags->regenerateBlkAddr(blk);
}

int
ZeroTags::extractBlkOffset(Addr addr) const
{
    return normalDataTags->extractBlkOffset(addr);
}

Addr
ZeroTags::extractTag(Addr addr) const
{
    return normalDataTags->extractTag(addr);
}

ReplaceableEntry*
ZeroTags::findBlockBySetAndWay(int set, int way) const
{
    return normalDataTags->findBlockBySetAndWay(set, way);
}

CacheBlk*
ZeroTags::accessBlock(Addr addr, bool is_secure, Cycles &lat)
{
    return normalDataTags->accessBlock(addr, is_secure, lat);
}

CacheBlk*
ZeroTags::findVictim(Addr addr, const bool is_secure,
                                std::vector<CacheBlk*>& evict_blks) const
{
    return normalDataTags->findVictim(addr, is_secure, evict_blks);
}

void
ZeroTags::forEachBlk(std::function<void(CacheBlk &)> visitor)
{
    normalDataTags->forEachBlk(visitor);
}

bool
ZeroTags::anyBlk(std::function<bool(CacheBlk &)> visitor)
{
    return normalDataTags->anyBlk(visitor);
}

ZeroTags *
ZeroTagsParams::create()
{
    return new ZeroTags(this);
}
