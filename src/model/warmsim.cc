/*
 * Copyright (c) 2011-2019 Nikos Nikoleris
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

#include "model/warmsim.hh"

#include <fstream>

#include "base/random.hh"
#include "debug/WarmSim.hh"
#include "debug/WarmSimDebug.hh"
#include "model/mem_sampler.hh"
#include "model/statcache.hh"
#include "params/WarmSim.hh"
#include "proto/reuse.pb.h"

WarmSim::WarmSim(const WarmSimParams *p)
    : SimObject(p),
      system(p->system),
      numBlocks(p->cache_size / p->block_size),
      nextLevel(p->nextLevel),
      onlyOnLLC(p->onlyOnLLC)
{
}

void
WarmSim::load(const char *ifile)
{
    modeling = true;
    models.push_back(StatStack());
    StatStack &statstack = models.back();
    useAccesses |= statstack.load(ifile);
}

void
WarmSim::notifyHit(PacketPtr pkt, bool prefetched)
{
    if (!enabled || pkt->req->isInstFetch() || !pkt->req->hasPC()) {
        // if we have encountered an instruction fetch or since this is
        // not the icache we assume that it will hit
        // if
        return;
    }

    if (prefetched) {
        isMiss(pkt);
        return;
    }

    Addr pc = pkt->req->getPC();
    DPRINTF(WarmSimDebug, "Recording hit: %#x\n", pc);

    hits[pkt->cmdToIndex()][pkt->req->masterId()]++;
    overall_accesses_pc.sample(pc);

    if (nextLevel) {
        nextLevel->notifyHit(pkt);
    }
}

void
WarmSim::notifyMSHRHit(PacketPtr pkt, bool prefetched)
{
    notifyHit(pkt, prefetched);
}

void
WarmSim::notifyMiss(PacketPtr pkt, bool isConflict)
{
    if (!enabled || pkt->req->isInstFetch() || !pkt->req->hasPC()) {
        // if we have encountered an instruction fetch or since this is
        // not the icache we assume that it will hit
        return;
    }

    // record _all_ accesses that miss in the dcache, even if the miss
    // is a capacity/conflict miss in the dcache we will need to
    // handle it in the L2
    if (recording) {
        recordAccess(pkt);
        return;
    }

    Addr pc = pkt->req->getPC();
    Addr eff_addr = pkt->req->getVaddr();
    DPRINTF(WarmSimDebug, "Recording%s miss: %#llx by %#x\n",
            isConflict ? " conflict" : "", eff_addr, pc);

    misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
    mshr_misses_pc.sample(pkt->req->getPC());
    overall_accesses_pc.sample(pc);
}

void
WarmSim::recordAccess(PacketPtr pkt)
{
    AccessMiss am;
    am.instAddr = pkt->req->getPC();
    am.effAddr = pkt->req->getVaddr();
    am.effPhysAddr = pkt->req->getPaddr();
    am.size = pkt->req->getSize();
    accessesMiss.push_back(am);
    DPRINTF(WarmSim, "Recording access pc: %#llx vaddr: %#llx paddr: %#llx "
            "size: %u\n", am.instAddr, am.effAddr, am.effPhysAddr, am.size);
}

bool
WarmSim::isMiss(PacketPtr pkt)
{
    Addr eff_phys_addr = pkt->req->getPaddr();
    DPRINTF(WarmSim, "Request for addr %#llx", eff_phys_addr);
    if (!enabled || !modeling) {
        DPRINTFR(WarmSim, "Model disabled\n");
        return true;
    }

    if (pkt->req->isInstFetch()) {
        DPRINTFR(WarmSim, "Inst fetch: turn into hit\n");
        // if we have encountered an instruction fetch since this is
        // not the icache we assume that it will hit
        return false;
    }

    if (!pkt->req->hasPC()) {
        DPRINTFR(WarmSim, "No PC\n");
        // if this is not a direct request but rather a writeback stop
        // here without handling it
        return true;
    }

    if (hackHitOnColdMiss) {
        DPRINTFR(WarmSim, "Turn into hit\n");
        return false;
    }

    if (nextLevel && onlyOnLLC) {
        // if there is a cache below consider this access a miss, let
        // warmsim handle only the LLC
        DPRINTFR(WarmSim, "Nothing for this cache\n");
        return true;
    }

    if (useAccesses) {
        DPRINTFR(WarmSim, "\n");
        Addr effAddr = pkt->req->getVaddr();
        Addr effPhysAddr = pkt->req->getPaddr();
        // Perform lookups in both model but also allow for an extra
        // lookup where the inst_addr is defined if the first set of
        // lookups fail. The instruction address is provided getSDist
        // returns the stack distance of another access from the same
        // address within the same page
        for (Addr inst_addr : {(Addr)0, pkt->req->getPC()}) {
            for (auto &model : models) {
                Tick sdist = model.getSDist(effAddr, effPhysAddr, inst_addr);
                // MaxTick is returned when the stack distance was not
                // found, try again.
                if (sdist == MaxTick) {
                    continue;
                }
                bool is_miss = sdist > numBlocks;
                return is_miss;
            }
        }
        DPRINTF(WarmSim, "Reuse for %#llx (%#llx) not found\n", effAddr,
                effPhysAddr);
    }

    double mr;
    Addr pc = pkt->req->getPC();
    auto it = missRatio.find(pc);
    if (it == missRatio.end()) {
        auto param = std::pair<Addr, InstMissRatio>(pc, InstMissRatio());
        auto ret = missRatio.insert(param);
        assert(ret.second);
        it = ret.first;
    }

    DPRINTF(WarmSim, "Warmsim pc: %#x for %d blocks, miss ratio: ",
            pc, numBlocks);
    InstMissRatio &instMissRatio = it->second;
    auto mit = instMissRatio.find(numBlocks);
    if (mit == instMissRatio.end()) {
        DPRINTFR(WarmSim, "Not found\n");
        auto &statstack = models.front();
        mr = statstack.getInstMR(pc, numBlocks);
        instMissRatio[numBlocks] = mr;
    } else {
        DPRINTFR(WarmSim, "Found\n");
    }
    mr = instMissRatio[numBlocks];
    bool is_miss = false;
    if (mr > 0) {
        double current_mr = 0.;
        long misses, accesses;

        misses = mshr_misses_pc.get(pc);
        accesses = overall_accesses_pc.get(pc);
        if (accesses != 0) {
            current_mr = misses * 1. / accesses;
        }

        is_miss = current_mr < mr;
        DPRINTF(WarmSim, "WarmSim pc: %#x misses: %llu, accesses: %llu, "
                "mr: %lf\n", pc, misses, accesses, current_mr);
    }
    DPRINTF(WarmSim, "StatStack mr: %lf -> %s\n", mr,
            is_miss ? "miss" :  "hit");
    return is_miss;
}

void
WarmSim::enable()
{
    enabled = true;
}


void
WarmSim::disable()
{
    enabled = false;
    dump_mr();
}

void
WarmSim::save(const char *ofile)
{
    if (accessesMiss.empty()) {
        return;
    }

    ProtoReuseProfile::Accesses pAccesses;

    for (const auto &am : accessesMiss) {
        ProtoReuseProfile::Access *pAccess =
            pAccesses.add_accesses();
        pAccess->set_pc(am.instAddr);
        pAccess->set_addr(am.effAddr);
        pAccess->set_physaddr(am.effPhysAddr);
        pAccess->set_size(am.size);
    }

    using namespace std;
    fstream output(ofile, ios::out | ios::trunc | ios::binary);
    if (!pAccesses.SerializeToOstream(&output))
        panic("Failed to write address book.");

    google::protobuf::ShutdownProtobufLibrary();
}

void
WarmSim::regStats()
{
    SimObject::regStats();

    using namespace Stats;

    // Hit statistics
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const std::string &cstr = cmd.toString();

        hits[access_idx]
            .init(system->maxMasters())
            .name(name() + "." + cstr + "_hits")
            .desc("number of " + cstr + " hits")
            .flags(total | nozero | nonan)
            ;
        for (int i = 0; i < system->maxMasters(); i++) {
            hits[access_idx].subname(i, system->getMasterName(i));
        }
    }

    // These macros make it easier to sum the right subset of commands and
    // to change the subset of commands that are considered "demand" vs
    // "non-demand"
#define SUM_DEMAND(s)                                                   \
    (s[MemCmd::ReadReq] + s[MemCmd::WriteReq] + s[MemCmd::ReadExReq])

    // should writebacks be included here?  prior code was inconsistent...
#define SUM_NON_DEMAND(s)                               \
    (s[MemCmd::SoftPFReq] + s[MemCmd::HardPFReq])

    demandHits
        .name(name() + ".demand_hits")
        .desc("number of demand (read+write) hits")
        .flags(total | nozero | nonan)
        ;
    demandHits = SUM_DEMAND(hits);
    for (int i = 0; i < system->maxMasters(); i++) {
        demandHits.subname(i, system->getMasterName(i));
    }

    overallHits
        .name(name() + ".overall_hits")
        .desc("number of overall hits")
        .flags(total | nozero | nonan)
        ;
    overallHits = demandHits + SUM_NON_DEMAND(hits);
    for (int i = 0; i < system->maxMasters(); i++) {
        overallHits.subname(i, system->getMasterName(i));
    }

    // Miss statistics
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const std::string &cstr = cmd.toString();

        misses[access_idx]
            .init(system->maxMasters())
            .name(name() + "." + cstr + "_misses")
            .desc("number of " + cstr + " misses")
            .flags(total | nozero | nonan)
            ;
        for (int i = 0; i < system->maxMasters(); i++) {
            misses[access_idx].subname(i, system->getMasterName(i));
        }
    }

    demandMisses
        .name(name() + ".demand_misses")
        .desc("number of demand (read+write) misses")
        .flags(total | nozero | nonan)
        ;
    demandMisses = SUM_DEMAND(misses);
    for (int i = 0; i < system->maxMasters(); i++) {
        demandMisses.subname(i, system->getMasterName(i));
    }

    overallMisses
        .name(name() + ".overall_misses")
        .desc("number of overall misses")
        .flags(total | nozero | nonan)
        ;
    overallMisses = demandMisses + SUM_NON_DEMAND(misses);
    for (int i = 0; i < system->maxMasters(); i++) {
        overallMisses.subname(i, system->getMasterName(i));
    }

    // access formulas
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const std::string &cstr = cmd.toString();

        accesses[access_idx]
            .name(name() + "." + cstr + "_accesses")
            .desc("number of " + cstr + " accesses(hits+misses)")
            .flags(total | nozero | nonan)
            ;
        accesses[access_idx] = hits[access_idx] + misses[access_idx];

        for (int i = 0; i < system->maxMasters(); i++) {
            accesses[access_idx].subname(i, system->getMasterName(i));
        }
    }

    demandAccesses
        .name(name() + ".demand_accesses")
        .desc("number of demand (read+write) accesses")
        .flags(total | nozero | nonan)
        ;
    demandAccesses = demandHits + demandMisses;
    for (int i = 0; i < system->maxMasters(); i++) {
        demandAccesses.subname(i, system->getMasterName(i));
    }

    overallAccesses
        .name(name() + ".overall_accesses")
        .desc("number of overall (read+write) accesses")
        .flags(total | nozero | nonan)
        ;
    overallAccesses = overallHits + overallMisses;
    for (int i = 0; i < system->maxMasters(); i++) {
        overallAccesses.subname(i, system->getMasterName(i));
    }

    // miss rate formulas
    for (int access_idx = 0; access_idx < MemCmd::NUM_MEM_CMDS; ++access_idx) {
        MemCmd cmd(access_idx);
        const std::string &cstr = cmd.toString();

        missRate[access_idx]
            .name(name() + "." + cstr + "_miss_rate")
            .desc("miss rate for " + cstr + " accesses")
            .flags(total | nozero | nonan)
            ;
        missRate[access_idx] = misses[access_idx] / accesses[access_idx];

        for (int i = 0; i < system->maxMasters(); i++) {
            missRate[access_idx].subname(i, system->getMasterName(i));
        }
    }

    demandMissRate
        .name(name() + ".demand_miss_rate")
        .desc("miss rate for demand accesses")
        .flags(total | nozero | nonan)
        ;
    demandMissRate = demandMisses / demandAccesses;
    for (int i = 0; i < system->maxMasters(); i++) {
        demandMissRate.subname(i, system->getMasterName(i));
    }

    overallMissRate
        .name(name() + ".overall_miss_rate")
        .desc("miss rate for overall accesses")
        .flags(total | nozero | nonan)
        ;
    overallMissRate = overallMisses / overallAccesses;
    for (int i = 0; i < system->maxMasters(); i++) {
        overallMissRate.subname(i, system->getMasterName(i));
    }

    mshr_misses_pc
        .init(0)
        .precision(0)
        .name(name() + ".mshr_misses_pc")
        .desc("MSHR misses per instruction")
        .flags(nozero | total);

    overall_accesses_pc
        .init(0)
        .precision(0)
        .name(name() + ".overall_accesses_pc")
        .desc("Overall accesses per instruction")
        .flags(nozero | total);
}

void
WarmSim::dump_mr()
{
    DPRINTF(WarmSim, "Miss ratio for pc\n");
    for (const auto &it : missRatio) {
        Addr pc M5_VAR_USED = it.first;
        DPRINTF(WarmSim, "%#lx -> ", pc);
        const InstMissRatio instMissRatio = it.second;
        for (const auto & mit : instMissRatio) {
            double mr M5_VAR_USED = mit.second;
            DPRINTFR(WarmSim, "%f ", mr);
        }
        DPRINTFR(WarmSim, "\n");
    }
}

WarmSim *
WarmSimParams::create()
{
    return new WarmSim(this);
}
