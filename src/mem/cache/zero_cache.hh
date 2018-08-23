// TODO licence header

#ifndef __MEM_CACHE_ZER_CACHEH__
#define __MEM_CACHE_ZER_CACHEH__

#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "params/ZeroCache.hh"

class CacheBlk;
class ZeroBlk;
class ZeroTags;

class ZeroCache : public Cache
{
  protected:
    std::vector<CacheBlk*> migrateToZeroBlocks;
    std::vector<CacheBlk*> migrateToDataBlocks;
    ZeroTags *zeroTags;
    unsigned zeroBlockSize;
    Addr zeroTagRegionStart;
    Addr zeroTagRegionEnd;

  public:
    ZeroCache(const ZeroCacheParams *p);
    virtual ~ZeroCache();

    virtual bool doMigrations(PacketList &writebacks);
    virtual bool zeroRegionInstr(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                    PacketList writebacks);
    virtual bool zeroTagAccessToMemoryAccess(PacketPtr pkt, CacheBlk *blk,
                                     Cycles lat, PacketList &writebacks);
    virtual ZeroBlk *allocateZeroBlock(const PacketPtr pkt,
                                       PacketList &writebacks);
    virtual ZeroBlk *findOrAllocateZeroBlock(const PacketPtr pkt,
                                             PacketList &writebacks);

    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;
    CacheBlk *allocateBlock(const PacketPtr pkt, PacketList &writebacks) override;
    CacheBlk *handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                         bool allocate) override;
    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response, bool pending_downgrade) override;
    void recvTimingResp(PacketPtr pkt) override;
    void handleTimingReqMiss(PacketPtr pkt, CacheBlk *blk, Tick forward_time,
                             Tick request_time) override;
    void invalidateBlock(CacheBlk *blk) override;

    void regStats() override;

  protected:
    virtual bool isInZeroTagRegion(Addr addr);

  protected:
    // Statistics
    Stats::Scalar zeroReplacements;
};

#endif // __MEM_CACHE_ZER_CACHEH__
