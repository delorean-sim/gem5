/*
 * Copyright (c) 2012-2019 Nikos Nikoleris
 * All rights reserved.
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
 * Authors: Nikos Nikoleris
 */

#ifndef __MODEL_MEM_SAMPLER_HH__
#define __MODEL_MEM_SAMPLER_HH__

#include <random>
#include <unordered_map>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "debug/MemSampler.hh"
#include "debug/MemSamplerDebug.hh"
#include "model/histogram.hh"
#include "model/statcache.hh"
#include "model/watchpoints.hh"
#include "params/MemSampler.hh"
#include "proto/reuse.pb.h"
#include "sim/sim_object.hh"

class MemSampler : public SimObject
{
  public:
    // types
    struct Access {
        Addr instAddr;
        Addr effAddr;
        Addr effPhysAddr;
        Addr size;
    };

    struct AccessSample : Access {
        bool targeted;
        bool random;
        Tick weight;
        Tick mtime;
        Tick itime;
    };

    struct TargetedAccess : Access {
        Tick dist;
        Counter count;
        Tick itime;
    };

    struct MemoryReuse {
        AccessSample begin, end;
    };

  public:
    // Methods exposed to the rest of the system
    MemSampler(const MemSamplerParams *params);

    void regStats() override;

    void observeLoad(ThreadContext *tc, Addr inst_addr,
                     Addr phys_addr, Addr virt_addr, int size);
    void observeStore(ThreadContext *tc, Addr inst_addr,
                      Addr phys_addr, Addr virt_addr, int size);

    inline void
    setTime(uint64_t inst_count, uint64_t load_count, uint64_t store_count)
    {
        instCtr = inst_count;
        loadCtr = load_count;
        storeCtr = store_count;
        memInstCtr = load_count + store_count;
    }

    uint64_t
    getNextLoadStop(uint64_t min_period = 0)
    {
        if (loadCtr + min_period > nextLoadSample) {
            uint64_t prev_load_sample = nextLoadSample;
            nextLoadSample = loadCtr + nextSample();
            DPRINTF(MemSampler, "Missed load sample (%d/%d), next at %i\n",
                    loadCtr, prev_load_sample, nextLoadSample);
            numMissedSamplesLoad++;
        }
        return nextLoadSample;
    }
    uint64_t
    getNextStoreStop(uint64_t min_period = 0)
    {
        if (storeCtr + min_period > nextStoreSample) {
            uint64_t prev_store_sample = nextStoreSample;
            nextStoreSample = storeCtr + nextSample();
            DPRINTF(MemSampler, "Missed store sample (%d/%d), next at %i\n",
                    storeCtr, prev_store_sample, nextStoreSample);
            numMissedSamplesStore++;
        }
        return nextStoreSample;
    }

    uint64_t getLoadCount() const { return loadCtr; }
    uint64_t getStoreCount() const { return storeCtr; }

    KvmWatchpoints kvmWatchpoints;

  public:
    // Methods exposed to the user
    void loadAccesses(const char *ifile);
    void save(const char *reuse_file, const char *key_file);
    void stage();
    void setSamplingPeriod(long p, int size);

  private:
    void sampleAccess(Addr inst_addr, Addr virt_addr, Addr phys_pddr,
                      Addr size);
    void sampleAccess(const AccessSample ac);
    void recordReuse(Addr inst_addr, Addr addr, Addr phys_addr, Addr size);

    Counter totalDanglings() {
        return watchpoints.size();
    };

  private:

    std::unordered_map<Addr, AccessSample> watchpoints;

    std::list<MemoryReuse> sampledReuses;
    std::list<MemoryReuse> targetedReuses;

    std::unordered_map<Addr, TargetedAccess> targetedAccesses;
    std::unordered_map<Addr, TargetedAccess> stagedTargetedAccesses;

  private:

    /** Block size of the reuses we are looking for */
    const Addr blockSize;

    const int vicinityWindowSize;

    /**
     * The smallest reuse for the targeted accesses that we are
     * interested in
     */
    const Tick targetedAccessMinReuseInst;

    /** If set the sampler ignores memory accesses from the kernel */
    const bool excludeKernel;

    /** The currect period of the sampling process */
    long samplingPeriod;
    bool samplingEnabled = false;

    /** The desired size of each sample */
    int batchSize = 1;

    /** The current size of the load sample */
    int sampledLoadsInBatch = 0;
    /** The current size of the store sample */
    int sampledStoresInBatch = 0;

  private:

    Vicinity populateVicinity(Tick itime);

  private:
    // Internal counters
    uint64_t instCtr = 0;
    uint64_t loadCtr = 0;
    uint64_t storeCtr = 0;
    uint64_t memInstCtr = 0;
    uint64_t stagedTime = 0;

    uint64_t nextLoadSample;
    uint64_t nextStoreSample;

    std::mt19937 rng;
    std::exponential_distribution<double> expDistr;

    uint64_t nextSample() {
        return std::llround(expDistr(rng));
    }

  public:

    /**
     * @{
     */

    /** Number of reuses we obtained through sampling */
    Stats::Scalar numRandomSamples;
    /** Number of accesses we targeted */
    Stats::Scalar numTargetedAccesses;
    /** Number of obtained sampled reuses */
    Stats::Scalar numSampledReuses;
    /** Number of obtained reuses for targeted accesses */
    Stats::Scalar numTargetedReuses;
    /** Total number of times a targeted watchpoint fired */
    Stats::Scalar numTargetedWatchpointHits;
    /** Number of missed samples - load  */
    Stats::Scalar numMissedSamplesLoad;
    /** Number of missed samples - load  */
    Stats::Scalar numMissedSamplesStore;

    /**
     * @}
     */

};

#endif // __MODEL_MEM_SAMPLER_HH__
