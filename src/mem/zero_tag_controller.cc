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

#include "mem/zero_tag_controller.hh"

ZeroTagController::ZeroTagController(const ZeroTagControllerParams* p) :
    MemObject(p),
    zeroTagGranularityBytes(p->zero_tag_granularity_bytes),
    dataRegion(p->data_region),
    zeroTagRegion(p->zero_tag_region),
    masterPort(name() + "-master", *this),
    slavePort(name() + "-slave", *this)
{
    // Ensure we have enough zero-tags to cover all the data (at
    // least, hopefully only as much as needed)
    assert(dataRegion.size() <=
           zeroTagRegion.size()*zeroTagGranularityBytes);
}

void
ZeroTagController::init()
{
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Zero-tag controller is not connected on both sides.\n");
}

BaseMasterPort&
ZeroTagController::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "master") {
        return masterPort;
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

BaseSlavePort&
ZeroTagController::getSlavePort(const std::string& if_name, PortID idx)
{
    if (if_name == "slave") {
        return slavePort;
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

void
ZeroTagController::recvFunctional(PacketPtr pkt)
{
    if (pkt->isZeroTagAccess()) {
        Addr orig_addr = pkt->getAddr();
        pkt->setAddr(remapAddr(pkt));
        masterPort.sendFunctional(pkt);
        pkt->setAddr(orig_addr);
    } else
        masterPort.sendFunctional(pkt);
}

void
ZeroTagController::recvFunctionalSnoop(PacketPtr pkt)
{
    if (dataRegion.contains(pkt->getAddr())) {
        slavePort.sendFunctionalSnoop(pkt);
    } else
        fatal("Not yet implemented %s %s\n",
              __func__, pkt->print());
}

Tick
ZeroTagController::recvAtomic(PacketPtr pkt)
{
    if (pkt->isZeroTagAccess()) {
        Addr orig_addr = pkt->getAddr();
        pkt->setAddr(remapAddr(pkt));
        Tick ret_tick = masterPort.sendAtomic(pkt);
        pkt->setAddr(orig_addr);
        return ret_tick;
    } else
        return masterPort.sendAtomic(pkt);
}

Tick
ZeroTagController::recvAtomicSnoop(PacketPtr pkt)
{
    if (dataRegion.contains(pkt->getAddr())) {
        return slavePort.sendAtomicSnoop(pkt);
    } else
        fatal("Not yet implemented %s %s\n",
              __func__, pkt->print());
}

bool
ZeroTagController::recvTimingReq(PacketPtr pkt)
{
    if (pkt->isZeroTagAccess()) {
        Addr orig_addr = pkt->getAddr();

        pkt->setAddr(remapAddr(pkt));

        // Attempt to send the packet
        bool successful = masterPort.sendTimingReq(pkt);

        // If not successful, restore the address and sender state
        if (!successful) {
            pkt->setAddr(orig_addr);
        }

        return successful;
    } else
        return masterPort.sendTimingReq(pkt);
}

bool
ZeroTagController::recvTimingResp(PacketPtr pkt)
{
    if (pkt->isZeroTagAccess()) {
        return slavePort.sendTimingResp(pkt);
    } else
        return slavePort.sendTimingResp(pkt);
}

void
ZeroTagController::recvTimingSnoopReq(PacketPtr pkt)
{
    if (dataRegion.contains(pkt->getAddr())) {
        slavePort.sendTimingSnoopReq(pkt);
    } else
        fatal("Not yet implemented %s %s\n",
              __func__, pkt->print());
}

bool
ZeroTagController::recvTimingSnoopResp(PacketPtr pkt)
{
    if (dataRegion.contains(pkt->getAddr())) {
        return masterPort.sendTimingSnoopResp(pkt);
    } else
        fatal("Not yet implemented %s %s\n",
              __func__, pkt->print());
}

AddrRangeList
ZeroTagController::getAddrRanges() const
{
    return masterPort.getAddrRanges();
}

bool
ZeroTagController::isSnooping() const
{
    return slavePort.isSnooping();
}

void
ZeroTagController::recvReqRetry()
{
    slavePort.sendRetryReq();
}

void
ZeroTagController::recvRespRetry()
{
    masterPort.sendRetryResp();
}

void
ZeroTagController::recvRangeChange()
{
    slavePort.sendRangeChange();
}

ZeroTagController*
ZeroTagControllerParams::create()
{
    return new ZeroTagController(this);
}

Addr
ZeroTagController::remapAddr(PacketPtr pkt) const
{
    if (pkt->isZeroTagAccess()) {
        Addr data_addr = pkt->getAddr();
        assert(dataRegion.contains(data_addr));
        Addr offset = data_addr - dataRegion.start();
        // We return the zero-tag byte addr
        Addr zero_tag_byte =
            zeroTagRegion.start() +
            offset/(// Zero-tag bit address
                zeroTagGranularityBytes
                // Zero-tag byte address
                *8);
        // Packet-size aligned zero-tag address
        return zero_tag_byte - (zero_tag_byte%pkt->getSize());
    } else {
        return pkt->getAddr();
    }
}
