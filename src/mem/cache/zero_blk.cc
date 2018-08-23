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

#ifndef __MEM_CACHE_ZERO_BLK_H__
#define __MEM_CACHE_ZERO_BLK_H__

#include "mem/cache/tags/zero_tags.hh"
#include "mem/cache/zero_blk.hh"

ZeroBlk::ZeroBlk() : CacheBlk() {}
ZeroBlk::~ZeroBlk() {}

void
ZeroBlk::setOwner(BaseTags *owner)
{
    ZeroTags *zeroTagsOwner = dynamic_cast<ZeroTags*>(owner);
    assert(zeroTagsOwner);
    assert(_owner == nullptr);
    _owner = zeroTagsOwner;
    _blkSize = _owner->getZeroBlockSize();

}

ZeroTags *
ZeroBlk::getOwner() const
{
    return _owner;
}

void
ZeroBlk::setEntries(unsigned size)
{
    entries.resize(size);
}

void
ZeroBlk::setEntryValid(Addr addr, bool valid)
{
    return;
}

void
ZeroBlk::setEntryWay(Addr addr, unsigned way)
{
    return;
}

void
ZeroBlk::setEntryZero(Addr addr, bool zero)
{
    int entry_count = entries.size();
    entries[addr%entry_count].is_zero = zero;
}

bool
ZeroBlk::isEntryValid(Addr addr)
{
    int entry_count = entries.size();
    return entries[addr%entry_count].is_valid;
}

bool
ZeroBlk::isEntryZero(Addr addr)
{
    int entry_count = entries.size();
    return entries[addr%entry_count].is_zero;
}

unsigned
ZeroBlk::getEntryWay(Addr addr)
{
    int entry_count = entries.size();
    return entries[addr%entry_count].way;
}

std::vector<CacheBlk*>
ZeroBlk::getValidNonzeros(std::vector<CacheSet<CacheBlk>> sets)
{
    std::vector<CacheBlk*> blks;
    Addr zblk_addr = getAddr();
    Addr entry_addr = getOwner()->tagToBlockAddr(zblk_addr);
    for (const auto &entry : entries) {
        if (entry.is_valid && ~entry.is_zero) {
            CacheBlk *blk = sets[getOwner()->extractSet(entry_addr)]
                .blks[entry.way];
            blks.push_back(blk);
        }
        // this->getBlockSize() might be the size of the zero block
        entry_addr += getOwner()->getBlockSize();
    }
    return blks;
}

#endif // __MEM_CACHE_ZERO_BLK_H__
