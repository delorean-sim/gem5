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

#include "model/mem_sampler.hh"

#include <fstream>
#include <functional>
#include <string>

#include "arch/utility.hh"
#include "base/compiler.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/MemSampler.hh"
#include "debug/MemSamplerDebug.hh"
#include "model/statcache.hh"
#include "params/MemSampler.hh"
#include "sim/system.hh"

MemSampler::MemSampler(const MemSamplerParams *p)
    : SimObject(p),
      kvmWatchpoints(p->system),
      blockSize(p->blockSize),
      vicinityWindowSize(p->windowSize),
      targetedAccessMinReuseInst(20000),
      excludeKernel(true)
{
    std::random_device rd;
    rng.seed(rd());
}

void
MemSampler::observeLoad(ThreadContext *tc, Addr inst_addr, Addr phys_addr,
                        Addr virt_addr, int size)
{
    if (!TheISA::inUserMode(tc) && excludeKernel) {
        return;
    }

    recordReuse(inst_addr, virt_addr, phys_addr, size);
    if (nextLoadSample == loadCtr) {
        sampleAccess(inst_addr, virt_addr, phys_addr, size);
        if (sampledLoadsInBatch < batchSize) {
            sampledLoadsInBatch++;
            nextLoadSample++;
        }
    }

    if (nextLoadSample <= loadCtr) {
        sampledLoadsInBatch = 0;
        if (samplingEnabled) {
            nextLoadSample += nextSample();
        } else {
            nextLoadSample = (uint64_t)-1;
        }
        DPRINTF(MemSampler, "Next load sample %d\n", nextLoadSample);
    }
}

void
MemSampler::observeStore(ThreadContext *tc, Addr inst_addr, Addr phys_addr,
                         Addr virt_addr, int size)
{
    if (!TheISA::inUserMode(tc) && excludeKernel) {
        return;
    }

    recordReuse(inst_addr, virt_addr, phys_addr, size);
    if (nextStoreSample == storeCtr) {
        sampleAccess(inst_addr, virt_addr, phys_addr, size);
        if (sampledStoresInBatch < batchSize) {
            sampledStoresInBatch++;
            nextStoreSample++;
        }
    }

    if (nextStoreSample <= storeCtr) {
        sampledStoresInBatch = 0;
        if (samplingEnabled) {
            nextStoreSample += nextSample();
        } else {
            nextStoreSample = (uint64_t)-1;
        }
        DPRINTF(MemSampler, "Next store sample %d\n", nextStoreSample);
    }
}

void
MemSampler::sampleAccess(Addr inst_addr, Addr virt_addr,
                         Addr phys_addr, Addr size)
{
    AccessSample ac;
    ac.instAddr = inst_addr;
    ac.effAddr = virt_addr;
    ac.effPhysAddr = phys_addr;
    ac.size = size;
    ac.mtime = memInstCtr;
    ac.itime = instCtr;
    ac.weight = samplingPeriod;
    ac.targeted = false;
    ac.random = true;

    DPRINTF(MemSampler, "Sampling access: (%d/%d): access: %#llx "
            "(+%d) by %#x\n", instCtr, memInstCtr, ac.effPhysAddr,
            ac.size, ac.instAddr);

    sampleAccess(ac);
}

void
MemSampler::sampleAccess(const AccessSample ac)
{
    const Addr blk_phys_addr = roundDown(ac.effPhysAddr, blockSize);
    auto ret = watchpoints.emplace(blk_phys_addr, ac);
    if (!ret.second) {
        // there was already a targeted watchpoint and we happened to
        // randomly sample the same block
        auto &wp = ret.first;
        AccessSample &old_ac = wp->second;
        assert(old_ac.targeted || ac.targeted);
        old_ac.random = old_ac.targeted = true;
    }
    kvmWatchpoints.insert(blk_phys_addr);
}

void
MemSampler::recordReuse(Addr inst_addr, Addr virt_addr, Addr phys_addr,
                        Addr size)
{
    const Addr blk_phys_addr = roundDown(phys_addr, blockSize);
    auto it = watchpoints.find(blk_phys_addr);
    if (it == watchpoints.end()) {
        return;
    }

    AccessSample &ac1 = it->second;

    MemoryReuse reuse;
    reuse.begin = ac1;

    AccessSample &ac2 = reuse.end;
    ac2.instAddr = inst_addr;
    ac2.effAddr = virt_addr;
    ac2.effPhysAddr = phys_addr;
    ac2.size = size;
    ac2.mtime = memInstCtr;
    ac2.itime = instCtr;
    const Tick rdist = memInstCtr - ac1.mtime;

    // At this point, this could be a targeted access or the reuse of
    // a randomly choosen instruction or both

    if (ac1.random) {
        DPRINTF(MemSampler, "Recording smpl reuse - addr: %#llx "
                "pc: %#llx inst count: %llu rdist %llu\n", blk_phys_addr,
                inst_addr, instCtr, rdist);

        numSampledReuses++;
        sampledReuses.push_back(reuse);
        ac1.random = false;
    }

    auto wp = targetedAccesses.find(blk_phys_addr);
    assert(!ac1.targeted || wp != targetedAccesses.end());
    if (ac1.targeted) {
        TargetedAccess &acc = wp->second;

        numTargetedWatchpointHits++;
        if (virt_addr == acc.effAddr && phys_addr == acc.effPhysAddr &&
            inst_addr == acc.instAddr && size == acc.size) {
            // only if there is a match in a couple of properties we
            // record this

            const Tick rdist_inst = instCtr - ac1.itime;

            // if this is the first time, we observed this access or
            // it reuse is small than we expected, we skip it
            if (ac1.mtime != -1 && rdist_inst > targetedAccessMinReuseInst) {
                DPRINTF(MemSampler, "Recording tgt reuse - addr: %#llx "
                        "pc: %#llx inst count: %llu rdist: %llu\n",
                        blk_phys_addr, inst_addr, instCtr, rdist);

                numTargetedReuses++;
                targetedReuses.push_back(reuse);
                acc.count++;
                acc.itime = instCtr;
                acc.dist = memInstCtr - ac1.mtime;
            }
        }

        ac1 = ac2;
        ac1.targeted = true;
    } else {
        // if this is not a targeted access, we don't need the
        // watchpoint any more
        watchpoints.erase(it);
        kvmWatchpoints.remove(phys_addr);
    }
}

void
MemSampler::stage()
{
    assert(stagedTargetedAccesses.empty());
    std::swap(targetedAccesses, stagedTargetedAccesses);
    stagedTime = instCtr;

    for (const auto &reuse: stagedTargetedAccesses) {
        TargetedAccess acc = reuse.second;
        const Addr blk_phys_addr = roundDown(acc.effPhysAddr, blockSize);
        DPRINTF(MemSampler, "Staging access %#x\n", acc.effPhysAddr);
        auto it = watchpoints.find(blk_phys_addr);
        assert(it != watchpoints.end());
        AccessSample &as = it->second;
        assert(as.targeted);
        as.targeted = false;
        if (!as.random) {
            watchpoints.erase(it);
            kvmWatchpoints.remove(blk_phys_addr);
        }
    }
    assert(targetedAccesses.empty());
    for (const auto &wp: watchpoints) {
        const AccessSample &as = wp.second;
        assert(!as.targeted);
    }
}

void
MemSampler::loadAccesses(const char *ifile)
{
    assert(targetedAccesses.empty());

    std::fstream input(ifile, std::ios::in | std::ios::binary);
    panic_if(!input, "File not found: %s\n", ifile);

    ProtoReuseProfile::Accesses pAccesses;
    panic_if(!pAccesses.ParseFromIstream(&input),
             "Failed to parse file: %s\n", ifile);

    for (unsigned i = 0; i < pAccesses.accesses_size(); i++) {
        const ProtoReuseProfile::Access &pAccess = pAccesses.accesses(i);

        TargetedAccess tgt_acc;
        tgt_acc.instAddr = pAccess.pc();
        tgt_acc.effAddr = pAccess.addr();
        tgt_acc.effPhysAddr = pAccess.physaddr();
        tgt_acc.size = pAccess.size();
        tgt_acc.count = 0;
        tgt_acc.dist = 0;
        const Addr blk_phys_addr = roundDown(tgt_acc.effPhysAddr, blockSize);
        auto ret M5_VAR_USED = targetedAccesses.emplace(blk_phys_addr,
                                                        tgt_acc);
        assert(ret.second);

        AccessSample smpl_acc;
        smpl_acc.instAddr = pAccess.pc();
        smpl_acc.effAddr = pAccess.addr();
        smpl_acc.effPhysAddr = pAccess.physaddr();
        smpl_acc.size = pAccess.size();
        smpl_acc.targeted = true;
        smpl_acc.random = false;
        smpl_acc.mtime = -1;

        DPRINTF(MemSampler, "Loading key access: %#llx (+%d) by %#x\n",
                smpl_acc.effPhysAddr, smpl_acc.size, smpl_acc.instAddr);

        sampleAccess(smpl_acc);
        kvmWatchpoints.enable(smpl_acc.effPhysAddr);
    }
    numTargetedAccesses += pAccesses.accesses_size();

    google::protobuf::ShutdownProtobufLibrary();
}

Vicinity
MemSampler::populateVicinity(Tick itime)
{
    Vicinity vicinity(vicinityWindowSize);

    vicinity.reset();
    // Traverse recorded reuse samples and populate the vicinity
    // histogram and a per-instruction bounded buffer with reuse
    // samples
    auto rit = sampledReuses.begin();
    while (rit != sampledReuses.end()) {
        const MemoryReuse &reuse = *rit;
        if (reuse.begin.itime < itime) {
            Tick rdist = reuse.end.mtime - reuse.begin.mtime;
            vicinity.addReuse(reuse.begin.mtime, reuse.end.mtime,
                              reuse.begin.weight);
            DPRINTF(MemSampler, "Adding reuse to vicinity: %llu (w:%llu) - "
                    "from pc: %#llx time: %#llu, to pc: %#llx time: %#llu\n",
                    rdist, reuse.begin.weight, reuse.begin.instAddr,
                    reuse.begin.mtime, reuse.end.instAddr, reuse.end.mtime);

            rit = sampledReuses.erase(rit);
        } else {
            rit++;
        }
    }

    // Add pending watchpoints in the vicinity histogram as infinite
    // reuses, danglings.
    auto wit = watchpoints.begin();
    while (wit != watchpoints.end()) {
        const AccessSample as = wit->second;
        if (as.itime < itime) {
            assert(!as.targeted);
            DPRINTF(MemSampler, "Adding dangling to vicinity: inf (w: %%llu) "
                    "- from pc: %#llx time: %#llu\n", as.instAddr, as.mtime);
            vicinity.addDangling(as.mtime, as.weight);

            wit = watchpoints.erase(wit);
            kvmWatchpoints.remove(as.effPhysAddr);
        } else {
            wit++;
        }
    }

    return vicinity;
}

void
MemSampler::save(const char *reuse_file, const char *accesses_file)
{
    Vicinity vicinity = populateVicinity(stagedTime);
    ProtoReuseProfile::ReuseProfile reuseProfile;
    vicinity.save(reuseProfile);

    ProtoReuseProfile::Accesses pAccesses;
    bool dump_key_accesses = false;
    for (const auto &reuse: stagedTargetedAccesses) {
        TargetedAccess acc = reuse.second;
        ProtoReuseProfile::Access *pAccess;
        if (!acc.count) {
            dump_key_accesses = true;
            DPRINTF(MemSampler, "Key access not found (i:%llu): "
                    "%#llx (%#llx) by %#llx\n", acc.itime,
                    acc.effAddr, acc.effPhysAddr, acc.instAddr);
            pAccess = pAccesses.add_accesses();
        } else {
            DPRINTF(MemSampler, "Saving access (i:%llu): %d "
                    "%#llx (%#llx) by %#llx\n", acc.itime,
                    acc.dist, acc.effAddr, acc.effPhysAddr, acc.instAddr);

            pAccess = reuseProfile.add_accesses();
            pAccess->set_dist(acc.dist);
        }
        pAccess->set_pc(acc.instAddr);
        pAccess->set_addr(acc.effAddr);
        pAccess->set_physaddr(acc.effPhysAddr);
        pAccess->set_size(acc.size);
    }
    stagedTargetedAccesses.clear();

    if (reuse_file) {
        DPRINTF(MemSampler, "Dumping collected reuses until %d to: %s\n",
                stagedTime, reuse_file);
        using namespace std;
        fstream output(reuse_file, ios::out | ios::trunc | ios::binary);
        panic_if(!reuseProfile.SerializeToOstream(&output),
                 "Failed to write %s.", reuse_file);
    }
    if (accesses_file && dump_key_accesses) {
        DPRINTF(MemSampler, "Dumping remaining key accesses to: %s\n",
                accesses_file);
        using namespace std;
        fstream output(accesses_file, ios::out | ios::trunc | ios::binary);
        panic_if(!pAccesses.SerializeToOstream(&output),
                 "Failed to write %s.", accesses_file);

    }
    google::protobuf::ShutdownProtobufLibrary();
}

void
MemSampler::setSamplingPeriod(long period, int size)
{
    DPRINTF(MemSampler, "Changing parameters: period %lld size: %d\n",
            period, size);

    if (size == 0) {
        samplingEnabled = false;
        return;
    }

    samplingPeriod = period;
    batchSize = size;

    double lambda = 1. / samplingPeriod;
    expDistr = std::exponential_distribution<double>(lambda);

    if (!samplingEnabled) {
        nextLoadSample = loadCtr + nextSample();
        nextStoreSample = storeCtr + nextSample();
        samplingEnabled = true;
        DPRINTF(MemSampler, "Enabling sampling next - load: %d store: %d\n",
                nextLoadSample, nextStoreSample);
    }
}

void
MemSampler::regStats()
{
    SimObject::regStats();

    using namespace Stats;

    numRandomSamples
        .name(name() + ".random_samples")
        .desc("Number of times a random sample was taken")
        ;

    numTargetedAccesses
        .name(name() + ".targeted_accesses")
        .desc("Number of accesses that we targeted")
        ;

    numSampledReuses
        .name(name() + ".sampled_reuses")
        .desc("Number of reuses we sampled")
        ;

    numTargetedReuses
        .name(name() + ".targeted_reuses")
        .desc("Number of targeted reuses we obtained")
        ;

    numTargetedWatchpointHits
        .name(name() + ".targeted_watchpoint_hits")
        .desc("Number of time a targeted watchpoint was triggered")
        ;

    numMissedSamplesLoad
      .name(name() + ".missed_samples_load")
      .desc("Number of missed samples - load")
      ;

    numMissedSamplesStore
      .name(name() + ".missed_samples_store")
      .desc("Number of missed samples - store")
      ;

    kvmWatchpoints.regStats();
}

MemSampler*
MemSamplerParams::create()
{
    return new MemSampler(this);
}
