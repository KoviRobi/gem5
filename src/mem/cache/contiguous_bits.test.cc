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

#include <gtest/gtest.h>

#include "mem/cache/contiguous_bits.hh"

#define T_EQ(IDX) EXPECT_EQ((bool)(cb[IDX]), expected[IDX])
#define T_EQ_8 T_EQ(0); T_EQ(1); T_EQ(2); T_EQ(3); \
    T_EQ(4); T_EQ(5); T_EQ(6); T_EQ(7)


TEST(ContiguousBits, Index)
{
    unsigned char array[1] = { 0b10110100 };
    bool expected[1*8] = { 1,0,1,1,0,1,0,0 };
    ContiguousBits cb(array, 1*8);

    T_EQ_8;
}

TEST(ContiguousBits, Assign)
{
    unsigned char array[1] = { 0 };
    bool expected[1*8] = { 1,0,1,1,0,1,0,0 };
    ContiguousBits cb(array, 1*8);

    for (int i = 0; i < 1*8; ++i)
        cb[i] = expected[i];

    T_EQ_8;
}

TEST(ContiguousBits, AndAssign)
{
    unsigned char array[1] = { 0b11010001 };
    bool mask[1*8]         = { 1,0,1,1,0,1,0,0 };
    bool expected[1*8]     = { 1,0,0,1,0,0,0,0 };
    ContiguousBits cb(array, 1*8);

    for (int i = 0; i < 1*8; ++i)
        cb[i] &= mask[i];

    T_EQ_8;
}

TEST(ContiguousBits, OrAssign)
{
    unsigned char array[1] = { 0b11010001 };
    bool mask[1*8]         = { 1,0,1,1,0,1,0,0 };
    bool expected[1*8]     = { 1,1,1,1,0,1,0,1 };
    ContiguousBits cb(array, 1*8);

    for (int i = 0; i < 1*8; ++i)
        cb[i] |= mask[i];

    T_EQ_8;
}

TEST(ContiguousBits, XorAssign)
{
    unsigned char array[1] = { 0b11010001 };
    bool mask[1*8]         = { 1,0,1,1,0,1,0,0 };
    bool expected[1*8]     = { 0,1,1,0,0,1,0,1 };
    ContiguousBits cb(array, 1*8);

    for (int i = 0; i < 1*8; ++i)
        cb[i] ^= mask[i];

    T_EQ_8;
}
