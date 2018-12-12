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

#ifndef __MEM_ZERO_TAG_CONTROLLER_HH__
#define __MEM_ZERO_TAG_CONTROLLER_HH__

#include "mem/mem_object.hh"
#include "params/ZeroTagController.hh"

/**
 * The zero-tag controller maps requests tagged with the
 * ZERO_TAG_ACCESS flag into the zero-tag memory region
 */
class ZeroTagController : public MemObject
{

  public:

    ZeroTagController(const ZeroTagControllerParams* p);

    ~ZeroTagController() { }

    virtual BaseMasterPort& getMasterPort(const std::string& if_name,
                                          PortID idx = InvalidPortID);

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID);

    virtual void init();

  protected:
    int zeroTagGranularityBytes;
    AddrRange dataRegion;
    AddrRange zeroTagRegion;

    class ControllerMasterPort : public MasterPort
    {

      public:

        ControllerMasterPort(const std::string& _name,
                         ZeroTagController& _controller)
            : MasterPort(_name, &_controller),
              controller(_controller)
        { }

      protected:

        ZeroTagController& controller;

        void recvFunctionalSnoop(PacketPtr pkt)
        {
            controller.recvFunctionalSnoop(pkt);
        }

        Tick recvAtomicSnoop(PacketPtr pkt)
        {
            return controller.recvAtomicSnoop(pkt);
        }

        bool recvTimingResp(PacketPtr pkt)
        {
            return controller.recvTimingResp(pkt);
        }

        void recvTimingSnoopReq(PacketPtr pkt)
        {
            controller.recvTimingSnoopReq(pkt);
        }

        void recvRangeChange()
        {
            controller.recvRangeChange();
        }

        bool isSnooping() const
        {
            return controller.isSnooping();
        }

        void recvReqRetry()
        {
            controller.recvReqRetry();
        }

    };

    /** Instance of master port, facing the memory side */
    ControllerMasterPort masterPort;

    class ControllerSlavePort : public SlavePort
    {

      public:

        ControllerSlavePort(const std::string& _name,
                        ZeroTagController& _controller)
            : SlavePort(_name, &_controller),
              controller(_controller)
        { }

      protected:

        void recvFunctional(PacketPtr pkt)
        {
            controller.recvFunctional(pkt);
        }

        Tick recvAtomic(PacketPtr pkt)
        {
            return controller.recvAtomic(pkt);
        }

        bool recvTimingReq(PacketPtr pkt)
        {
            return controller.recvTimingReq(pkt);
        }

        bool recvTimingSnoopResp(PacketPtr pkt)
        {
            return controller.recvTimingSnoopResp(pkt);
        }

        AddrRangeList getAddrRanges() const
        {
            return controller.getAddrRanges();
        }

        void recvRespRetry()
        {
            controller.recvRespRetry();
        }

      private:

        ZeroTagController& controller;

    };

    /** Instance of slave port, i.e. on the CPU side */
    ControllerSlavePort slavePort;

    void recvFunctional(PacketPtr pkt);

    void recvFunctionalSnoop(PacketPtr pkt);

    Tick recvAtomic(PacketPtr pkt);

    Tick recvAtomicSnoop(PacketPtr pkt);

    bool recvTimingReq(PacketPtr pkt);

    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    bool recvTimingSnoopResp(PacketPtr pkt);

    AddrRangeList getAddrRanges() const;

    bool isSnooping() const;

    void recvReqRetry();

    void recvRespRetry();

    void recvRangeChange();

    Addr remapAddr(PacketPtr pkt) const;
};

#endif //__MEM_ZERO_TAG_CONTROLLER_HH__
