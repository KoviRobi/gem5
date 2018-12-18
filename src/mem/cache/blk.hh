/*
 * Copyright (c) 2012-2018 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Andreas Sandberg
 */

/** @file
 * Definitions of a simple cache block class.
 */

#ifndef __MEM_CACHE_BLK_HH__
#define __MEM_CACHE_BLK_HH__

#include <cassert>
#include <cstdint>
#include <iosfwd>
#include <list>
#include <string>

#include "base/printable.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/data_container.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

class BaseTags;

/**
 * Cache block status bit assignments
 */
enum CacheBlkStatusBits : unsigned {
    /** valid, readable */
    BlkValid =          0x01,
    /** write permission */
    BlkWritable =       0x02,
    /** read permission (yes, block can be valid but not readable) */
    BlkReadable =       0x04,
    /** dirty (modified) */
    BlkDirty =          0x08,
    /** block was a hardware prefetch yet unaccessed*/
    BlkHWPrefetched =   0x20,
    /** block holds data from the secure memory space */
    BlkSecure =         0x40,
};

/**
 * A Basic Cache block.
 * Contains the tag, status, and a pointer to data.
 */
class CacheBlk : public ReplaceableEntry, public DataContainer
{
  protected:
    /**
     * Contains a copy of the data in this block for easy access. This is used
     * for efficient execution when the data could be actually stored in
     * another format (COW, compressed, sub-blocked, etc). In all cases the
     * data stored here should be kept consistant with the actual data
     * referenced by this block.
     */
    uint8_t *_data;
    BaseTags *_owner;
    unsigned _blkSize;

  public:
    /** Task Id associated with this block */
    uint32_t task_id;

    /** Data block tag value. */
    Addr tag;

    /** block state: OR of CacheBlkStatusBit */
    typedef unsigned State;

    /** The current status of this block. @sa CacheBlockStatusBits */
    State status;

    /** Which curTick() will this block be accessible */
    Tick whenReady;

    /**
     * The set and way this block belongs to.
     * @todo Move this into subclasses when we fix CacheTags to use them.
     */
    int set, way;

    /** Number of references to this block since it was brought in. */
    unsigned refCount;

    /** holds the source requestor ID for this block. */
    int srcMasterId;

    /** Tick on which the block was inserted in the cache. */
    Tick tickInserted;

  protected:
    /**
     * Represents that the indicated thread context has a "lock" on
     * the block, in the LL/SC sense.
     */
    class Lock {
      public:
        ContextID contextId;     // locking context
        Addr lowAddr;      // low address of lock range
        Addr highAddr;     // high address of lock range

        // check for matching execution context, and an address that
        // is within the lock
        bool matches(const RequestPtr &req) const
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() -1;
            return (contextId == req->contextId()) &&
                   (req_low >= lowAddr) && (req_high <= highAddr);
        }

        // check if a request is intersecting and thus invalidating the lock
        bool intersects(const RequestPtr &req) const
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() - 1;

            return (req_low <= highAddr) && (req_high >= lowAddr);
        }

        Lock(const RequestPtr &req)
            : contextId(req->contextId()),
              lowAddr(req->getPaddr()),
              highAddr(lowAddr + req->getSize() - 1)
        {
        }
    };

    /** List of thread contexts that have performed a load-locked (LL)
     * on the block since the last store. */
    std::list<Lock> lockList;

  public:
    CacheBlk();

    CacheBlk(const CacheBlk&) = delete;
    CacheBlk& operator=(const CacheBlk&) = delete;
    virtual ~CacheBlk();

    /**
     * Sets the owner of this cache block.
     */
    virtual void setOwner(BaseTags *owner);

    /**
     * Gets the owner of this cache block.
     */
    virtual BaseTags *getOwner() const;

    /**
     * Get block's address.
     *
     * @return addr Address value.
     */
    virtual Addr getAddr() const;

    virtual unsigned getBlockSize() const;

    /**
     * Get this cache block's data pointer
     * @return The cache block's data pointer
     */
    const uint8_t* getConstDataPtr() const override;

    /**
     * Set this cache block's data pointer, if it has not yet been set.
     * @param ptr The address at which the data for this cache block starts at.
     */
    virtual void setDataPtr(uint8_t *ptr);

    /**
     * This sets the data from src to this object, first byte written
     * to offset, and last byte written to offset+size-1 of this
     * object.
     * @param src Data source (should contain at least size many
     * bytes, not checked).
     * @param size Number of bytes to copy
     * @param offset Offset to the first byte of this block's data to which
     *               src will be copied.
     */
    void setData(const void *src, const Addr size,
                 const Addr offset = 0) override;

    /**
     * This clears the data of this object by setting it all to zero.
     */
    virtual void clearData();

    /**
     * Sets the block's data from the packet.
     * @param pkt The packet containing the data.
     * @param blk_size The size of this cache-block.
     */
    void setDataFromPacket(const Packet *pkt, const unsigned blk_size);

    /**
     * Executes the atomic operation contained in the packet. An
     * atomic operation both reads and writes the data in the cache
     * block, so this is both a getter and a setter.
     * @param pkt The packet containing the data.
     * @param blk_size The size of this cache-block.
     */
    void executeAtomic(Packet *pkt, unsigned blk_size);

    /**
     * Checks the write permissions of this block.
     * @return True if the block is writable.
     */
    bool isWritable() const;

    /**
     * Checks the read permissions of this block.  Note that a block
     * can be valid but not readable if there is an outstanding write
     * upgrade miss.
     * @return True if the block is readable.
     */
    bool isReadable() const;

    /**
     * Checks that a block is valid.
     * @return True if the block is valid.
     */
    bool isValid() const;

    /**
     * Invalidate the block and clear all state.
     */
    virtual void invalidate();

    /**
     * Check to see if a block has been written.
     * @return True if the block is dirty.
     */
    bool isDirty() const;

    /**
     * Check if this block was the result of a hardware prefetch, yet to
     * be touched.
     * @return True if the block was a hardware prefetch, unaccesed.
     */
    bool wasPrefetched() const;

    /**
     * Check if this block holds data from the secure memory space.
     * @return True if the block holds data from the secure memory space.
     */
    bool isSecure() const;

    /**
     * Set member variables when a block insertion occurs. Resets reference
     * count to 1 (the insertion counts as a reference), and touch block if
     * it hadn't been touched previously. Sets the insertion tick to the
     * current tick. Does not make block valid.
     *
     * @param tag Block address tag.
     * @param is_secure Whether the block is in secure space or not.
     * @param src_master_ID The source requestor ID.
     * @param task_ID The new task ID.
     */
    virtual void insert(const Addr tag, const bool is_secure,
                        const int src_master_ID, const uint32_t task_ID);

    /**
     * Track the fact that a local locked was issued to the
     * block. Invalidate any previous LL to the same address.
     */
    void trackLoadLocked(PacketPtr pkt);

    /**
     * Clear the any load lock that intersect the request, and is from
     * a different context.
     */
    void clearLoadLocks(const RequestPtr &req);

    /**
     * Pretty-print a tag, and interpret state bits to readable form
     * including mapping to a MOESI state.
     *
     * @return string with basic state information
     */
    std::string print() const;

    /**
     * Handle interaction of load-locked operations and stores.
     * @return True if write should proceed, false otherwise.  Returns
     * false only in the case of a failed store conditional.
     */
    bool checkWrite(PacketPtr pkt);
};

/**
 * Special instance of CacheBlk for use with tempBlk that deals with its
 * block address regeneration.
 * @sa Cache
 */
class TempCacheBlk final : public CacheBlk
{
  private:
    /**
     * Copy of the block's address, used to regenerate tempBlock's address.
     */
    Addr _addr;

  public:
    /**
     * Creates a temporary cache block, with its own storage.
     * @param size The size (in bytes) of this cache block.
     */
    TempCacheBlk(unsigned size);
    TempCacheBlk(const TempCacheBlk&) = delete;
    TempCacheBlk& operator=(const TempCacheBlk&) = delete;
    ~TempCacheBlk();

    void setDataPtr(uint8_t *ptr) override;

    /**
     * Invalidate the block and clear all state.
     */
    void invalidate() override;

    void insert(const Addr addr, const bool is_secure,
                const int src_master_ID=0, const uint32_t task_ID=0) override;

    /**
     * Get block's address.
     *
     * @return addr Address value.
     */
    Addr getAddr() const override;
};

/**
 * Simple class to provide virtual print() method on cache blocks
 * without allocating a vtable pointer for every single cache block.
 * Just wrap the CacheBlk object in an instance of this before passing
 * to a function that requires a Printable object.
 */
class CacheBlkPrintWrapper : public Printable
{
    CacheBlk *blk;
  public:
    CacheBlkPrintWrapper(CacheBlk *_blk);
    virtual ~CacheBlkPrintWrapper();
    void print(std::ostream &o, int verbosity = 0,
               const std::string &prefix = "") const;
};

#endif //__MEM_CACHE_BLK_HH__
