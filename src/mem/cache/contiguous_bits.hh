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

#ifndef __MEM_CACHE_CONTIGUOUS_BITS_HH__
#define __MEM_CACHE_CONTIGUOUS_BITS_HH__

#include <cstdint>

class Bit {
  private:
    // Using chars avoids endianness problems (on any platforms where
    // char is the smallest addressable unit at least)
    unsigned char* const address;
    const unsigned char mask;

  public:
    Bit(unsigned char *address, unsigned char offset);
    ~Bit();

    Bit operator=(const bool value);
    operator bool() const;
    Bit operator&=(const bool value);
    Bit operator|=(const bool value);
    Bit operator^=(const bool value);
};

class ContiguousBits {
  private:
    // Using chars avoids endianness problems (on any platforms where
    // char is the smallest addressable unit at least).
    uint8_t *data;
    std::size_t size;

  public:
    ContiguousBits(uint8_t *_data, std::size_t size);
    /// When using setData later on
    ContiguousBits();
    virtual ~ContiguousBits();

    /**
     * Change the underlying data
     */
    void setData(uint8_t *_data, std::size_t _size);

    /**
     * @param offset is in bits, not in bytes!
     * @return the Bit object encapsulating the boolean value
     *     (castable to bool, and assignable).
     */
    Bit operator[](std::size_t offset) const;
};

#endif // __MEM_CACHE_CONTIGUOUS_BITS_HH__
