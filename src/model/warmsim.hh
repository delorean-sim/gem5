/*
 * Copyright (c) 2015 Nikos Nikoleris
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
 * Authors: Nikos Nikoleris
 */

#ifndef __MODEL_WARMSIM_HH__
#define __MODEL_WARMSIM_HH__

#include <map>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "model/histogram.hh"
#include "model/mem_sampler.hh"
#include "model/statcache.hh"
#include "params/WarmSim.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class WarmSim;

class WarmSim : public SimObject
{
  public:
    WarmSim(const WarmSimParams *p);

    void notifyHit(PacketPtr pkt, bool wasPrefeched = false);
    void notifyMSHRHit(PacketPtr pkt, bool wasPrefeched = false);
    void notifyMiss(PacketPtr pkt, bool isConflict = false);

    bool isMiss(PacketPtr pkt);

    void regStats() override;

    void enable();
    void disable();
    void record() { recording = true; }
    void save(const char *ofile);
    void load(const char *ifile);

    void dump_mr();

    void hitOnColdMissEnable() {
        hackHitOnColdMiss = true;
    }

    void hitOnColdMissDisable() {
        hackHitOnColdMiss = false;
    }

  public:
    typedef std::map<unsigned, double> InstMissRatio;
    typedef std::map<Addr, InstMissRatio> MissRatio;
    typedef struct {
        Addr instAddr, effAddr, effPhysAddr;
        unsigned size;
    } AccessMiss;
    typedef std::list<AccessMiss> AccessesMiss;

  private:
    System *system;
    uint64_t numBlocks;
    WarmSim *nextLevel;
    bool enabled = false;
    bool modeling = false;
    bool recording = false;
    bool hackHitOnColdMiss = false;

    std::list<StatStack> models;

    MissRatio missRatio;
    AccessesMiss accessesMiss;

    bool useAccesses;

    // Should we only run the model on the LLC?
    const bool onlyOnLLC;

    void recordAccess(PacketPtr pkt);

    /**
     * Number of hits per thread for each type of command. @sa *
     * Packet::Command
     */
    Stats::Vector hits[MemCmd::NUM_MEM_CMDS];
    /** Number of hits for demand accesses. */
    Stats::Formula demandHits;
    /** Number of hit for all accesses. */
    Stats::Formula overallHits;

    /**
     * Number of misses per thread for each type of command. @sa
     * Packet::Command
     */
    Stats::Vector misses[MemCmd::NUM_MEM_CMDS];
    /** Number of misses for demand accesses. */
    Stats::Formula demandMisses;
    /** Number of misses for all accesses. */
    Stats::Formula overallMisses;

    /** The number of accesses per command and thread. */
    Stats::Formula accesses[MemCmd::NUM_MEM_CMDS];
    /** The number of demand accesses. */
    Stats::Formula demandAccesses;
    /** The number of overall accesses. */
    Stats::Formula overallAccesses;

    /** The miss rate per command and thread. */
    Stats::Formula missRate[MemCmd::NUM_MEM_CMDS];
    /** The miss rate of all demand accesses. */
    Stats::Formula demandMissRate;
    /** The miss rate for all accesses. */
    Stats::Formula overallMissRate;

    Stats::SparseHistogram mshr_misses_pc;
    Stats::SparseHistogram overall_accesses_pc;
};

#endif // __MODEL_WARMSIM_HH__
