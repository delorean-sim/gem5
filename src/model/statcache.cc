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

#include "model/statcache.hh"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/WarmSim.hh"
#include "debug/WarmSimDebug.hh"
#include "proto/reuse.pb.h"

void
Vicinity::addReuse(Tick time1, Tick time2, Tick freq)
{
    Tick win_id = time1 / windowSize;
    std::pair<Windows::iterator, bool> ret;
    ret = windows.insert(std::pair<Tick, Window>(win_id, Window()));

    Windows::iterator wit = ret.first;
    Window &win = wit->second;
    Tick reuse = time2 - time1;
    win.reuses.add(reuse, freq);
}

void
Vicinity::addDangling(Tick time1, Tick freq)
{
    Tick win_id = time1 / windowSize;
    std::pair<Windows::iterator, bool> ret;
    ret = windows.insert(std::pair<Tick, Window>(win_id, Window()));

    Windows::iterator wit = ret.first;
    Window &win = wit->second;
    win.reuses.add(MaxTick, freq);
}

void
Vicinity::reset()
{
    windows.clear();
}

void
Vicinity::prepare()
{
    ReuseHist reusesAcc;
    Tick lastWinNo M5_VAR_USED = MaxTick;

    auto wit = windows.rbegin();
    Tick wins = 0;
    Tick count = 50;
    while (wit != windows.rend()) {
        Tick winNo = wit->first;
        Window &win = wit->second;
        assert(winNo < lastWinNo);
        lastWinNo = winNo;

        win.reuses.add(reusesAcc);
        reusesAcc.set(win.reuses);
        auto _wit = std::next(wit);
        if ((reusesAcc.total() < count) && (_wit != windows.rend())) {
            windows.erase(_wit.base());
            continue;
        }
        wins++;
        count += std::min(25 * wins, (Tick)1000);

        DPRINTF(WarmSim, "Preparing winNo %llu: %llu (%llu)\n", winNo,
                reusesAcc.total(), count);
        std::advance(wit, 1);

        auto reuse_hist_ccdf = win.reuses.getCCDF();

        ReuseToStackDist &reuseToStackDist = win.reuseToStackDist;
        reuseToStackDist[0] = 0;
        reuseToStackDist[1] = 1;
        reuseToStackDist[MaxTick] = MaxTick;

        Tick prev_rdist = 0;
        float sdist;
        bool flag = true;
        for (auto &rit : reuse_hist_ccdf) {
            Tick rdist = rit.first;
            float freq = rit.second;

            if (flag) {
                sdist = rdist;
                flag = false;
            } else
                sdist += (rdist - prev_rdist) * freq;
            prev_rdist = rdist;
            reuseToStackDist[rdist] = llround(sdist);
            DPRINTF(WarmSimDebug, "Mapping rdist : %llu (%f) -> %llu\n", rdist,
                    freq, reuseToStackDist[rdist]);
        }
    }
}

Vicinity::ReuseToStackDist &
Vicinity::getSDistMap(Tick rdist, unsigned retry)
{
    Tick dist = 250000000;
    // dist = std::llround(std::exp2(std::ceil(std::log2(rdist))));

    auto it = reuseToVicinitySize.lower_bound(rdist);
    if (it != reuseToVicinitySize.end()) {
        Tick refDist = it->second;
        dist = std::max(refDist, dist);
    }
    dist *= pow(1.1, retry);

    if (dist > time())
        dist = time();

    reuseToVicinitySize[rdist] = dist;
    Tick winNo = (time() - dist) / windowSize;
    DPRINTF(WarmSim, "Looking up winNo %llu", winNo);
    auto wit = windows.lower_bound(winNo);
    if (wit == windows.end()) {
        DPRINTFR(WarmSim, " end");
        std::advance(wit, -1);
    } else {
        Tick winNo1 = wit->first;
        if ((winNo1 != winNo) && (wit != windows.begin()))
            std::advance(wit, -1);
    }
    winNo = wit->first;
    DPRINTFR(WarmSim, " -> matched %llu\n", winNo);

    return wit->second.reuseToStackDist;
}

Tick
Vicinity::getSDist(Tick rdist)
{
    if (rdist == 0 || rdist == 1 || rdist == MaxTick)
        return rdist;

    Tick sdist;
    unsigned retry = 0;
    do {
        ReuseToStackDist &reuseToStackDist = getSDistMap(rdist, retry++);
#ifndef NDEBUG
        for (auto rit : reuseToStackDist)
            DPRINTF(WarmSimDebug, "%llu -> %llu\n", rit.first, rit.second);
#endif

        auto rit = reuseToStackDist.find(rdist);
        if (rit != reuseToStackDist.end()) {
            sdist = rit->second;
            DPRINTF(WarmSim, "Reuse %llu -> sdist %llu: Exact match\n",
                    rdist, sdist);
            return sdist;
        } else {
            auto prevRit = reuseToStackDist.lower_bound(rdist);
            assert(prevRit != reuseToStackDist.end());
            auto nextRit = prevRit--;
            if (nextRit == reuseToStackDist.end())
                continue;

            Tick prevRDist = prevRit->first;
            Tick prevSDist = prevRit->second;
            Tick nextRDist = nextRit->first;
            Tick nextSDist = nextRit->second;

            assert(prevRDist < rdist && nextRDist > rdist);
            sdist = prevSDist + (nextSDist - prevSDist) * 1. /
                (nextRDist - prevRDist) * (rdist - prevRDist);
            reuseToStackDist[rdist] = sdist;

            DPRINTF(WarmSimDebug, "r1=%llu, s1=%llu - r2=%llu, s2=%llu\n",
                    prevRDist, prevSDist, nextRDist, nextSDist);
            DPRINTF(WarmSim, "Reuse %llu -> sdist %llu\n", rdist, sdist);
            return sdist;
        }
    } while (1);
}

void
Vicinity::dump(unsigned long count) const
{
    for (const auto &wit : windows) {
        DPRINTF(WarmSim, "Window %llu\n", wit.first);
        const Window &win = wit.second;
        const ReuseHist &reuseHist = win.reuses;
        reuseHist.dump();
        if (!--count)
            break;
    }
}

void
Vicinity::save(ProtoReuseProfile::ReuseProfile &profile)
{
    ProtoReuseProfile::Vicinity *pVicinity =
        profile.mutable_vicinity();

    pVicinity->set_windowsize(windowSize);
    for (const auto &wi : windows) {
        Tick id = wi.first;
        ProtoReuseProfile::Window *pWindow =
            pVicinity->add_window();

        pWindow->set_id(id);

        const ReuseHist &reuseHist = wi.second.reuses;
        for (const auto &rhi : reuseHist) {
            Tick dist = rhi.first;
            float freq = rhi.second;
            ProtoReuseProfile::Reuse *reuse = pWindow->add_reuse();
            reuse->set_dist(dist);
            reuse->set_freq(freq);
        }
    }
}

void
Vicinity::load(ProtoReuseProfile::ReuseProfile &profile)
{
    const ProtoReuseProfile::Vicinity &pVicinity = profile.vicinity();
    windowSize = pVicinity.windowsize();

    for (int i = 0; i < pVicinity.window_size(); i++) {
        const ProtoReuseProfile::Window pWindow = pVicinity.window(i);
        Tick winNo = pWindow.id();
        time(std::max(time(), winNo * windowSize));
        std::pair<Windows::iterator, bool> ret;
        ret = windows.insert(std::pair<Tick, Window>(winNo, Window()));

        Windows::iterator wit = ret.first;
        Window &win = wit->second;

        for (int j = 0; j < pWindow.reuse_size(); j++){
            const ProtoReuseProfile::Reuse &reuse = pWindow.reuse(j);
            Tick dist = reuse.dist();
            float freq = reuse.freq();
            assert(freq >= 0);
            win.reuses.add(dist, freq);
        }
    }
    DPRINTF(WarmSim, "Max time: %llu, window size: %llu\n", _time, windowSize);

    prepare();
}

Vicinity&
Vicinity::operator=(const Vicinity v)
{
    windows = v.windows;
    time(v.time());

    return *this;
}

float
StatStack::getOverallMR(uint64_t numBlocks)
{
    auto mit = overallMR.find(numBlocks);
    if (mit != overallMR.end())
        return mit->second;
    overallMR[numBlocks] = 1;

    unsigned long hits = 0, totalRefs = 0;

    const auto wit = vicinity.windows.begin();
    if (wit == vicinity.windows.end()) {
        return overallMR[numBlocks];
    }

    Vicinity::ReuseToStackDist &reuseToStackDist =
        wit->second.reuseToStackDist;

    for (auto &&it : wit->second.reuses) {
        Tick rdist = it.first;
        float freq = it.second;

        Tick sdist = reuseToStackDist[rdist];
        if (sdist < numBlocks)
            hits += freq;
        totalRefs += freq;
    }
    if (totalRefs != 0)
        overallMR[numBlocks] = 1 - (1. * hits / totalRefs);
    return overallMR[numBlocks];
}

float
StatStack::getInstMR(Addr pc, uint64_t numBlocks)
{
    auto isr_it = instReuseHist.find(pc);
    if (isr_it == instReuseHist.end()) {
        return getOverallMR(numBlocks);
    }

    unsigned long hits = 0, totalRefs = 0;
    ReuseHist &reuseHist = isr_it->second;

    DPRINTF(WarmSim, "Reuse analysis for pc: %#llx\n", pc);
    for (auto &&it : reuseHist) {
        Tick rdist = it.first;
        float freq = it.second;

        Tick sdist = vicinity.getSDist(rdist);
        DPRINTF(WarmSim, "%f reuses of %llu (sdist: %llu)\n", freq, rdist,
                sdist);
        if (sdist < numBlocks)
            hits += freq;
        totalRefs += freq;
    }

    return 1 - (1. * hits / totalRefs);
}

Tick
StatStack::getRDist(Addr eff_addr, Addr eff_phys_addr, Addr inst_addr)
{
    auto ait = accesses.find(eff_phys_addr);
    if (ait != accesses.end()) {
        AccessSample as(ait->second);
        return as.dist;
    }

    // if the reuse for the specified access is not found, use the
    // reuse of a "similar" access, where "similar" is an access from
    // the same instruction that accessed the same page.
    const Addr page_size(0x1000);
    if (inst_addr) {
        for (auto ait: accesses) {
            AccessSample as(ait.second);
            if (as.instAddr == inst_addr) {
                Addr eff_phys_page1(roundDown(eff_phys_addr, page_size));
                Addr eff_phys_page2(roundDown(as.effPhysAddr, page_size));
                if (eff_phys_page1 == eff_phys_page2) {
                    DPRINTF(WarmSim, "Using the closest accesss: %#llx "
                            "(%#llx) by %#x\n", as.effAddr, as.effPhysAddr,
                            inst_addr);
                    return as.dist;
                }
            }
        }
        const Addr mask(0x10000);
        for (auto ait: accesses) {
            AccessSample as(ait.second);
            if (as.instAddr == inst_addr) {
                Addr eff_phys_page1(roundDown(eff_phys_addr, mask));
                Addr eff_phys_page2(roundDown(as.effPhysAddr, mask));
                if (eff_phys_page1 == eff_phys_page2) {
                    DPRINTF(WarmSim, "Using the closest accesss: %#llx "
                            "(%#llx) by %#x\n", as.effAddr, as.effPhysAddr,
                            inst_addr);
                    return as.dist;
                }
            }
        }
    }
    return MaxTick;
}

Tick
StatStack::getSDist(Addr eff_addr, Addr eff_phys_addr, Addr inst_addr)
{
    Tick rdist = getRDist(eff_addr, eff_phys_addr, inst_addr);
    Tick sdist = vicinity.getSDist(rdist);
    if (sdist != MaxTick) {
        DPRINTF(WarmSim, "Address: %#llx (%#llx) rdist: %llu sdist: %llu\n",
                eff_addr, eff_phys_addr, rdist, sdist);
    }
    return sdist;
}

bool
StatStack::load(const char *ifile)
{
    ProtoReuseProfile::ReuseProfile reuseProfile;
    std::fstream input(ifile, std::ios::in | std::ios::binary);
    if (!input)
        panic("File not found: %s\n", ifile);
    if (!reuseProfile.ParseFromIstream(&input))
        panic("Failed to parse file: %s\n", ifile);

    const ProtoReuseProfile::ReuseHist &pReuseHist = reuseProfile.reusehist();
    for (int i = 0; i < pReuseHist.instreusehist_size(); i++) {
        const ProtoReuseProfile::InstReuseHist &pInstReuseHist =
            pReuseHist.instreusehist(i);
        Addr instAddr = pInstReuseHist.addr();

        std::pair<Addr, ReuseHist> param(instAddr, ReuseHist());
        auto ret = instReuseHist.insert(param);
        auto it = ret.first;

        ReuseHist &reuses = it->second;

        for (int j = 0; j < pInstReuseHist.reuse_size(); j++){
            const ProtoReuseProfile::Reuse &reuse = pInstReuseHist.reuse(j);
            Tick dist = reuse.dist();
            float freq = reuse.freq();
            reuses.add(dist, freq);
        }
    }

    bool with_accesses = false;
    DPRINTF(WarmSim, "Loading reuse information for %u accesses\n",
            reuseProfile.accesses_size());
    for (int i = 0; i < reuseProfile.accesses_size(); i++) {
        with_accesses = true;
        const ProtoReuseProfile::Access pAccess =
            reuseProfile.accesses(i);
        AccessSample as;
        as.instAddr = pAccess.pc();
        as.effAddr = pAccess.addr();
        as.effPhysAddr = pAccess.physaddr();
        as.size = pAccess.size();
        as.dist = pAccess.dist();
        DPRINTF(WarmSim, "Loading reuse information for %#llx (%llx): %llu\n",
                as.effAddr, as.effPhysAddr, as.dist);
        auto param = std::pair<Addr, AccessSample>(as.effPhysAddr, as);
        auto ret = accesses.insert(param);
        if (!ret.second) {
            assert(!as.dist);
        }
    }

    vicinity.load(reuseProfile);
    google::protobuf::ShutdownProtobufLibrary();
    return with_accesses;
}
