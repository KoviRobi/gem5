/*
 * Copyright (c) 2018 Robert Kovacsics
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

#include "mem/ruby/structures/ZeroCacheMemory.hh"

#include "base/logging.hh"
#include "debug/RubyZeroCache.hh"

using namespace std;

ostream&
operator<<(ostream& out, const ZeroCacheMemory& obj)
{
    obj.print(out);
    out << flush;
    return out;
}

ZeroCacheMemory *
RubyZeroCacheParams::create()
{
    return new ZeroCacheMemory(this);
}

ZeroCacheMemory::ZeroCacheMemory(const Params *p)
    : CacheMemory(p)
{
    zeroData.clear();
}

ZeroCacheMemory::~ZeroCacheMemory() { }

AbstractCacheEntry*
ZeroCacheMemory::allocate(Addr address, AbstractCacheEntry *entry, bool touch)
{
    return CacheMemory::allocate(address, entry, touch);
}

void
ZeroCacheMemory::deallocate(Addr address)
{
    return CacheMemory::deallocate(address);
}

AbstractCacheEntry *
ZeroCacheMemory::lookup(Addr address)
{
    return CacheMemory::lookup(address);
}

const AbstractCacheEntry *
ZeroCacheMemory::lookup(Addr address) const
{
    return CacheMemory::lookup(address);
}

void
ZeroCacheMemory::verifyZero(Addr address, DataBlock& data)
{
    Addr line_address = makeLineAddress(address);
    assert(address == line_address);
    const AbstractCacheEntry *entry = lookup(line_address);
    bool has_zero_entry = zeros.find(line_address) != zeros.end();
    if (entry && has_zero_entry) {
        warn_once(
                  "Warning: entries exist in both the zero "
                  "and the non-zero cache!\n"
                  "This is a bug in the protocol. Address (%#x)\n",
                  line_address);
    } else if (entry) {
        assert(((AbstractCacheEntry*)entry)->getDataBlk() == data);
        if (zeroData.equal(data))
            warn_once(
                      "Warning: This entry should already be "
                      "in the zero cache!\n"
                      "This is a bug in the protocol. Address (%#x)\n",
                      line_address);
    } else if (has_zero_entry) {
        if (!zeroData.equal(data))
            warn_once(
                      "Warning: This entry should not be in "
                      "the zero cache!\n"
                      "This is a bug in the protocol. Address (%#x)\n",
                      line_address);
    }
}

void
ZeroCacheMemory::migrateZero(Addr address, DataBlock& data) {
    Addr line_address = makeLineAddress(address);
    assert(address == line_address);
    AbstractCacheEntry *entry = lookup(line_address);
    assert(entry->getDataBlk() == data);
    auto zero_entry = zeros.find(line_address);
    bool is_zero = zeroData.equal(data);
    if (zero_entry == zeros.end() && is_zero) {
        DPRINTF(RubyZeroCache, " :: zero :: True :: %#x\n", line_address);
        zeros.insert(zero_entry, line_address);
    } else if (zero_entry != zeros.end() &&!is_zero) {
        DPRINTF(RubyZeroCache, " :: zero :: False :: %#x\n", line_address);
        zeros.erase(zero_entry);
    }
}
