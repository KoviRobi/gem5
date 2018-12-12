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

#include "mem/cache/zero_blk.hh"

ZeroBlk::ZeroBlk() : CacheBlk(), _zeroTagsOwner(nullptr) {}
ZeroBlk::~ZeroBlk() {}

void
ZeroBlk::setOwner(BaseTags *owner)
{
    CacheBlk::setOwner(owner);
    ZeroTags *zeroTagsOwner = dynamic_cast<ZeroTags*>(owner);
    assert(_zeroTagsOwner == nullptr);
    assert(zeroTagsOwner);
    _zeroTagsOwner = zeroTagsOwner;
    _blkSize = _zeroTagsOwner->getZeroBlockSize();
    // blkSize is in bytes, not in bits
    ways.resize(8*getBlockSize());
}

ZeroTags *
ZeroBlk::getOwner() const
{
    return _zeroTagsOwner;
}

Addr
ZeroBlk::getAddr() const
{
    return getOwner()->regenerateZeroBlkAddr(this).addr;
}

/// TODO: RMK35: TagAddr?
void
ZeroBlk::insert(const Addr tag, const bool is_secure, const int src_master_ID,
                const uint32_t task_ID)
{
    CacheBlk::insert(tag, is_secure, src_master_ID, task_ID);
    for (auto &entry : ways) {
        entry = ENTRY_NOT_VALID;
    }
}

/// TODO: RMK35: Make a note that this 'tag_addr' isn't the 'tag
/// address' in cache terminology, but an address in the zero-tag
/// region
void
ZeroBlk::setEntryWay(TagAddr tag_addr, short way)
{
    ways[tag_addr.addr % getBlockSize()] = way;
}

void
ZeroBlk::invalidateEntry(TagAddr tag_addr)
{
    ways[tag_addr.addr % getBlockSize()] = ENTRY_NOT_VALID;
}

void
ZeroBlk::setEntryZero(TagAddr tag_addr, bool zero)
{
    // TODO: RMK35 Remove hardcoded number
    int byte_index = (tag_addr.addr / 8) % getBlockSize();
    int bit_mask = 1 << (tag_addr.addr % 8);
    if (zero)
        _data[byte_index] &= ~bit_mask;
    else
        _data[byte_index] |= bit_mask;
}

bool
ZeroBlk::isEntryZero(TagAddr tag_addr)
{
    int byte_index = (tag_addr.addr / 8) % getBlockSize();
    int bit_mask = 1 << (tag_addr.addr % 8);
    return _data[byte_index] & bit_mask;
}

bool
ZeroBlk::isEntryValid(TagAddr tag_addr)
{
    return ways[tag_addr.addr % getBlockSize()] != ENTRY_NOT_VALID;
}

short
ZeroBlk::getEntryWay(TagAddr tag_addr)
{
    return ways[tag_addr.addr % getBlockSize()];
}

void
ZeroBlk::maintainInclusitivity(const BaseTags *tags,
                               std::vector<CacheBlk *> &evict_blks)
{
    Addr tag_addr = getAddr();
    // TODO: RMK35 Remove hardcoded number
    Addr data_addr = tag_addr << 6;
    // TODO: RMK35: I am not sure about this
    int set = getOwner()->extractSet(data_addr);
    for (int i = 0; i < getBlockSize(); i += 8) {
        uint8_t tags8 = _data[i];
        for (int j = 0; j < 8; ++j) {
            if ((ways[8*i+j] != ENTRY_NOT_VALID) && ((tags8 & (1<<j)) != 0)) {
                CacheBlk *blk = static_cast<CacheBlk*>(
                    tags->findBlockBySetAndWay(set, ways[8*i+j]));
                evict_blks.push_back(blk);
            }
        }
    }
}

#endif // __MEM_CACHE_ZERO_BLK_H__
