/*
 * Copyright (c) 2015-2019 Nikos Nikoleris
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

#ifndef __MODEL_STATSTACK_HH__
#define __MODEL_STATSTACK_HH__

#include <map>

#include "base/trace.hh"
#include "debug/WarmSim.hh"
#include "model/histogram.hh"
#include "proto/reuse.pb.h"

typedef Histogram<Tick, Counter> ReuseHist;
typedef std::map<Addr, ReuseHist> InstReuseHist;

class Vicinity
{
  public:
    Vicinity() : _time(0) {};
    Vicinity(Tick _winSize) :
        windowSize(_winSize),
        _time(0)
    {};

    Vicinity& operator=(const Vicinity v);

    void addReuse(Tick time1, Tick time2, Tick freq);
    void addDangling(Tick time1, Tick freq);
    Tick getSDist(Tick rdist);

    void dump(unsigned long count) const;

    void save(ProtoReuseProfile::ReuseProfile &profile);
    void load(ProtoReuseProfile::ReuseProfile &profile);

    typedef std::map<Tick, Tick> ReuseToStackDist;
    typedef std::map<Tick, Tick> ReuseToVicinitySize;

    struct Window {
        ReuseHist reuses;
        ReuseToStackDist reuseToStackDist;
    };
    typedef std::map<Tick, Window> Windows;
    Windows windows;

    void prepare();
    Tick time(void) const { return _time; }
    void time(Tick t) { _time = t; }

    void reset();

  private:
    Tick windowSize;
    Tick _time;
    ReuseToVicinitySize reuseToVicinitySize;

    ReuseToStackDist &getSDistMap(Tick rdist, unsigned retry);
};


class StatStack
{
  public:
    float getInstMR(Addr pc, uint64_t numBlocks);
    void dump_vicinity(unsigned long count=0) {
        vicinity.dump(count);
    }
    bool load(const char *ifile);

    Tick getRDist(Addr eff_addr, Addr eff_phys_addr, Addr inst_addr);
    Tick getSDist(Addr eff_addr, Addr eff_phys_addr, Addr inst_addr=0);
  private:
    float getOverallMR(uint64_t numBlocks);
    InstReuseHist instReuseHist;
    Vicinity vicinity;

    struct AccessSample {
        Addr instAddr;
        Addr effAddr;
        Addr effPhysAddr;
        Addr size;
        Tick dist;
    };
    typedef std::map<Addr, AccessSample> Accesses;
    Accesses accesses;

    std::map<uint64_t, double> overallMR;
};

#endif // __MODEL_STATSTACK_HH__
