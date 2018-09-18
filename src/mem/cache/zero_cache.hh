// TODO licence header

#ifndef __MEM_CACHE_ZER_CACHEH__
#define __MEM_CACHE_ZER_CACHEH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "params/ZeroCache.hh"

class CacheBlk;
class ZeroBlk;
class ZeroTags;

typedef struct { Addr addr; } TagAddr;

class ZeroCache : public Cache
{
  protected:
    ZeroTags *zeroTags;
    unsigned zeroBlockSize;
    Addr zeroTagRegionStart;
    Addr zeroTagRegionEnd;

  public:
    ZeroCache(const ZeroCacheParams *p);
    virtual ~ZeroCache();

    Addr tagToDataAddr(TagAddr tag_addr);
    TagAddr dataToTagAddr(Addr data_addr);
    TagAddr alignTagAddrToBlock(TagAddr tag_addr);

    virtual void zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk);
    // virtual ZeroBlk *allocateZeroBlock(const PacketPtr pkt,
    //                                    PacketList &writebacks);
    // virtual ZeroBlk *findOrAllocateZeroBlock(const PacketPtr pkt,
    //                                          PacketList &writebacks);

    PacketPtr packetToReadTagPacket(PacketPtr pkt);

    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;
    CacheBlk *allocateBlock(const PacketPtr pkt, PacketList &writebacks) override;
    CacheBlk *handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                         bool allocate) override;
    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response, bool pending_downgrade) override;
    void invalidateBlock(CacheBlk *blk) override;

    void regStats() override;

  protected: /// Statistics
    Stats::Scalar zeroReplacements;
};

#endif // __MEM_CACHE_ZER_CACHEH__
