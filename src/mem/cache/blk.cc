/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2007 The Regents of The University of Michigan
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
 */

#include "mem/cache/blk.hh"

#include "base/cprintf.hh"

CacheBlk::CacheBlk() : _data(nullptr)
{
    invalidate();
}

CacheBlk::~CacheBlk() {}

const uint8_t*
CacheBlk::getConstDataPtr() const
{
    return _data;
}

void
CacheBlk::setDataPtr(uint8_t *ptr)
{
    assert(_data == nullptr);
    _data = ptr;
}

void
CacheBlk::setData(const void *src, const Addr size,
                  const Addr offset)
{
    std::memcpy(_data+offset, src, size);
}

void
CacheBlk::setDataFromPacket(const Packet *pkt, const unsigned blk_size)
{
    assert(pkt->getOffset(blk_size) + pkt->getSize() <= blk_size);
    setData(pkt->getConstDataPtr(), pkt->getSize(), pkt->getOffset(blk_size));
}

void
CacheBlk::executeAtomic(Packet *pkt, unsigned blk_size)
{
    (*(pkt->getAtomicOp()))(_data+pkt->getOffset(blk_size));
}

void
CacheBlk::insert(const Addr tag, const bool is_secure,
                 const int src_master_ID, const uint32_t task_ID)
{
    // Set block tag
    this->tag = tag;

    // Set source requestor ID
    srcMasterId = src_master_ID;

    // Set task ID
    task_id = task_ID;

    // Set insertion tick as current tick
    tickInserted = curTick();

    // Insertion counts as a reference to the block
    refCount = 1;

    // Set secure state
    if (is_secure) {
        status = BlkSecure;
    } else {
        status = 0;
    }
}

bool
CacheBlk::isWritable() const
{
    const State needed_bits = BlkWritable | BlkValid;
    return (status & needed_bits) == needed_bits;
}

bool
CacheBlk::isReadable() const
{
    const State needed_bits = BlkReadable | BlkValid;
    return (status & needed_bits) == needed_bits;
}

bool
CacheBlk::isValid() const
{
    return (status & BlkValid) != 0;
}

void
CacheBlk::invalidate()
{
    tag = MaxAddr;
    task_id = ContextSwitchTaskId::Unknown;
    status = 0;
    whenReady = MaxTick;
    refCount = 0;
    srcMasterId = Request::invldMasterId;
    tickInserted = MaxTick;
    lockList.clear();
}

bool
CacheBlk::isDirty() const
{
    return (status & BlkDirty) != 0;
}

bool
CacheBlk::wasPrefetched() const
{
    return (status & BlkHWPrefetched) != 0;
}

bool
CacheBlk::isSecure() const
{
    return (status & BlkSecure) != 0;
}

void
CacheBlk::trackLoadLocked(PacketPtr pkt)
{
    assert(pkt->isLLSC());
    auto l = lockList.begin();
    while (l != lockList.end()) {
        if (l->intersects(pkt->req))
            l = lockList.erase(l);
        else
            ++l;
    }

    lockList.emplace_front(pkt->req);
}

void
CacheBlk::clearLoadLocks(const RequestPtr &req)
{
    auto l = lockList.begin();
    while (l != lockList.end()) {
        if (l->intersects(req) && l->contextId != req->contextId()) {
            l = lockList.erase(l);
        } else {
            ++l;
        }
    }
}

std::string
CacheBlk::print() const
{
    /**
     *  state       M   O   E   S   I
     *  writable    1   0   1   0   0
     *  dirty       1   1   0   0   0
     *  valid       1   1   1   1   0
     *
     *  state   writable    dirty   valid
     *  M       1           1       1
     *  O       0           1       1
     *  E       1           0       1
     *  S       0           0       1
     *  I       0           0       0
     *
     * Note that only one cache ever has a block in Modified or
     * Owned state, i.e., only one cache owns the block, or
     * equivalently has the BlkDirty bit set. However, multiple
     * caches on the same path to memory can have a block in the
     * Exclusive state (despite the name). Exclusive means this
     * cache has the only copy at this level of the hierarchy,
     * i.e., there may be copies in caches above this cache (in
     * various states), but there are no peers that have copies on
     * this branch of the hierarchy, and no caches at or above
     * this level on any other branch have copies either.
     **/
    unsigned state = isWritable() << 2 | isDirty() << 1 | isValid();
    char s = '?';
    switch (state) {
      case 0b111: s = 'M'; break;
      case 0b011: s = 'O'; break;
      case 0b101: s = 'E'; break;
      case 0b001: s = 'S'; break;
      case 0b000: s = 'I'; break;
      default:    s = 'T'; break; // @TODO add other types
    }
    return csprintf("state: %x (%c) valid: %d writable: %d readable: %d "
                    "dirty: %d tag: %x", status, s, isValid(),
                    isWritable(), isReadable(), isDirty(), tag);
}

bool
CacheBlk::checkWrite(PacketPtr pkt)
{
    assert(pkt->isWrite());

    // common case
    if (!pkt->isLLSC() && lockList.empty())
        return true;

    const RequestPtr &req = pkt->req;

    if (pkt->isLLSC()) {
        // it's a store conditional... have to check for matching
        // load locked.
        bool success = false;

        auto l = lockList.begin();
        while (!success && l != lockList.end()) {
            if (l->matches(pkt->req)) {
                // it's a store conditional, and as far as the
                // memory system can tell, the requesting
                // context's lock is still valid.
                success = true;
                lockList.erase(l);
            } else {
                ++l;
            }
        }

        req->setExtraData(success ? 1 : 0);
        // clear any intersected locks from other contexts (our LL
        // should already have cleared them)
        clearLoadLocks(req);
        return success;
    } else {
        // a normal write, if there is any lock not from this
        // context we clear the list, thus for a private cache we
        // never clear locks on normal writes
        clearLoadLocks(req);
        return true;
    }
}

TempCacheBlk::TempCacheBlk(unsigned size) : CacheBlk()
{
    _data = new uint8_t[size];
}

TempCacheBlk::~TempCacheBlk() { delete [] _data; };

void
TempCacheBlk::setDataPtr(uint8_t *ptr)
{
    fatal("TempCacheBlk::setDataPtr should never be called, as it"
          " has a dynamically allocated _data field");
}

void
TempCacheBlk::invalidate()
{
    CacheBlk::invalidate();
    _addr = MaxAddr;
}

void
TempCacheBlk::insert(const Addr addr, const bool is_secure,
            const int src_master_ID, const uint32_t task_ID)
{
    // Set block address
    _addr = addr;

    // Set secure state
    if (is_secure) {
        status = BlkSecure;
    } else {
        status = 0;
    }
}

Addr
TempCacheBlk::getAddr() const
{
    return _addr;
}

CacheBlkPrintWrapper::CacheBlkPrintWrapper(CacheBlk *_blk) : blk(_blk) {}
CacheBlkPrintWrapper::~CacheBlkPrintWrapper() {}

void
CacheBlkPrintWrapper::print(std::ostream &os, int verbosity,
                            const std::string &prefix) const
{
    ccprintf(os, "%sblk %c%c%c%c\n", prefix,
             blk->isValid()    ? 'V' : '-',
             blk->isWritable() ? 'E' : '-',
             blk->isDirty()    ? 'M' : '-',
             blk->isSecure()   ? 'S' : '-');
}
