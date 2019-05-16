/*
 * Copyright (c) 2019 Robert Kovacsics
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
 * Authors: Robert Kovacsics
 */

#include "mem/cache/contiguous_bits.hh"

#include <climits>
#include <iostream>

#include "base/logging.hh"

Bit::Bit(unsigned char *_address, unsigned char _offset)
    : address(_address),
      mask(1<<(CHAR_BIT - 1 - _offset)) {}

Bit::~Bit() {}

Bit
Bit::operator=(const bool value) {
    unsigned char c = *address;
    *address = value? (c|mask) : (c&(~mask));
    return *this;
}

Bit::operator bool() const {
    return ((*address)&mask) != 0;
}

#define COMPOUND_OP(SYM)                            \
Bit                                                 \
Bit::operator SYM##= (const bool value) {           \
    this->operator=((bool)(*this) SYM value);       \
    return *this;                                   \
}
COMPOUND_OP(&);
COMPOUND_OP(|);
COMPOUND_OP(^);
#undef COMPOUND_OP

ContiguousBits::ContiguousBits(uint8_t *_data, std::size_t _size)
    : data(_data), size(_size) {}
ContiguousBits::ContiguousBits() : data(nullptr), size(0) {}

ContiguousBits::~ContiguousBits() {}

void
ContiguousBits::setData(uint8_t *_data, std::size_t _size) {
    data = _data;
    size = _size;
}

Bit
ContiguousBits::operator[](std::size_t index) const {
    assert(index < size);
    return Bit(&data[index>>3],
               index&7);
}
