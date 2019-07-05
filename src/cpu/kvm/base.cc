/*
 * Copyright (c) 2012, 2015, 2017 ARM Limited
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
 * Authors: Andreas Sandberg
 */

#include "cpu/kvm/base.hh"

#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <ostream>

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/registers.hh"
#include "arch/utility.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/Kvm.hh"
#include "debug/KvmGuestDebug.hh"
#include "debug/KvmIO.hh"
#include "debug/KvmRun.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "params/BaseKvmCPU.hh"
#include "sim/process.hh"
#include "sim/system.hh"

/* Used by some KVM macros */
#define PAGE_SIZE pageSize

BaseKvmCPU::BaseKvmCPU(BaseKvmCPUParams *params)
    : BaseCPU(params),
      vm(*params->system->getKvmVM()),
      _status(Idle),
      dataPort(name() + ".dcache_port", this),
      instPort(name() + ".icache_port", this),
      alwaysSyncTC(params->alwaysSyncTC),
      threadContextDirty(true),
      kvmStateDirty(false),
      vcpuID(vm.allocVCPUID()), vcpuFD(-1), vcpuMMapSize(0),
      _kvmRun(NULL), mmioRing(NULL),
      pageSize(sysconf(_SC_PAGE_SIZE)),
      tickEvent([this]{ tick(); }, "BaseKvmCPU tick",
                false, Event::CPU_Tick_Pri),
      activeInstPeriod(0),
      activeUserInstPeriod(0),
      skidUserInstLoad(40),
      skidUserInstStore(40),
      perfControlledByTimer(params->usePerfOverflow),
      hostFactor(params->hostFactor),
      singleStepThreshold(100),
      ctrInsts(0),
      ctrUserInsts(0)
{
    if (pageSize == -1)
        panic("KVM: Failed to determine host page size (%i)\n",
              errno);

    if (FullSystem)
        thread = new SimpleThread(this, 0, params->system, params->itb, params->dtb,
                                  params->isa[0]);
    else
        thread = new SimpleThread(this, /* thread_num */ 0, params->system,
                                  params->workload[0], params->itb,
                                  params->dtb, params->isa[0]);

    thread->setStatus(ThreadContext::Halted);
    threadInfo = new KvmExecContext(this, thread);
    tc = thread->getTC();
    threadContexts.push_back(tc);
}

BaseKvmCPU::~BaseKvmCPU()
{
    if (_kvmRun)
        munmap(_kvmRun, vcpuMMapSize);
    close(vcpuFD);
}

void
BaseKvmCPU::init()
{
    BaseCPU::init();

    if (numThreads != 1)
        fatal("KVM: Multithreading not supported");

    tc->initMemProxies(tc);

    // initialize CPU, including PC
    if (FullSystem && !switchedOut())
        TheISA::initCPU(tc, tc->contextId());
}

void
BaseKvmCPU::startup()
{
    const BaseKvmCPUParams * const p(
        dynamic_cast<const BaseKvmCPUParams *>(params()));

    Kvm &kvm(*vm.kvm);

    BaseCPU::startup();

    assert(vcpuFD == -1);

    // Tell the VM that a CPU is about to start.
    vm.cpuStartup();

    // We can't initialize KVM CPUs in BaseKvmCPU::init() since we are
    // not guaranteed that the parent KVM VM has initialized at that
    // point. Initialize virtual CPUs here instead.
    vcpuFD = vm.createVCPU(vcpuID);

    // Map the KVM run structure */
    vcpuMMapSize = kvm.getVCPUMMapSize();
    _kvmRun = (struct kvm_run *)mmap(0, vcpuMMapSize,
                                     PROT_READ | PROT_WRITE, MAP_SHARED,
                                     vcpuFD, 0);
    if (_kvmRun == MAP_FAILED)
        panic("KVM: Failed to map run data structure\n");

    // Setup a pointer to the MMIO ring buffer if coalesced MMIO is
    // available. The offset into the KVM's communication page is
    // provided by the coalesced MMIO capability.
    int mmioOffset(kvm.capCoalescedMMIO());
    if (!p->useCoalescedMMIO) {
        inform("KVM: Coalesced MMIO disabled by config.\n");
    } else if (mmioOffset) {
        inform("KVM: Coalesced IO available\n");
        mmioRing = (struct kvm_coalesced_mmio_ring *)(
            (char *)_kvmRun + (mmioOffset * pageSize));
    } else {
        inform("KVM: Coalesced not supported by host OS\n");
    }

    haveSyncMMU = kvm.capSyncMMU();

    thread->startup();

    Event *startupEvent(
        new EventFunctionWrapper([this]{ startupThread(); }, name(), true));
    schedule(startupEvent, curTick());
}

BaseKvmCPU::Status
BaseKvmCPU::KVMCpuPort::nextIOState() const
{
    return (activeMMIOReqs || pendingMMIOPkts.size())
        ? RunningMMIOPending : RunningServiceCompletion;
}

Tick
BaseKvmCPU::KVMCpuPort::submitIO(PacketPtr pkt)
{
    if (cpu->system->isAtomicMode()) {
        Tick delay = sendAtomic(pkt);
        delete pkt;
        return delay;
    } else {
        if (pendingMMIOPkts.empty() && sendTimingReq(pkt)) {
            activeMMIOReqs++;
        } else {
            pendingMMIOPkts.push(pkt);
        }
        // Return value is irrelevant for timing-mode accesses.
        return 0;
    }
}

bool
BaseKvmCPU::KVMCpuPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(KvmIO, "KVM: Finished timing request\n");

    delete pkt;
    activeMMIOReqs--;

    // We can switch back into KVM when all pending and in-flight MMIO
    // operations have completed.
    if (!(activeMMIOReqs || pendingMMIOPkts.size())) {
        DPRINTF(KvmIO, "KVM: Finished all outstanding timing requests\n");
        cpu->finishMMIOPending();
    }
    return true;
}

void
BaseKvmCPU::KVMCpuPort::recvReqRetry()
{
    DPRINTF(KvmIO, "KVM: Retry for timing request\n");

    assert(pendingMMIOPkts.size());

    // Assuming that we can issue infinite requests this cycle is a bit
    // unrealistic, but it's not worth modeling something more complex in
    // KVM.
    while (pendingMMIOPkts.size() && sendTimingReq(pendingMMIOPkts.front())) {
        pendingMMIOPkts.pop();
        activeMMIOReqs++;
    }
}

void
BaseKvmCPU::finishMMIOPending()
{
    assert(_status = RunningMMIOPending);
    assert(!tickEvent.scheduled());

    _status = RunningServiceCompletion;
    schedule(tickEvent, nextCycle());
}

void
BaseKvmCPU::startupThread()
{
    // Do thread-specific initialization. We need to setup signal
    // delivery for counters and timers from within the thread that
    // will execute the event queue to ensure that signals are
    // delivered to the right threads.
    const BaseKvmCPUParams * const p(
        dynamic_cast<const BaseKvmCPUParams *>(params()));

    vcpuThread = pthread_self();

    // Setup signal handlers. This has to be done after the vCPU is
    // created since it manipulates the vCPU signal mask.
    setupSignalHandler();

    setupCounters();

    if (p->usePerfOverflow)
        runTimer.reset(new PerfKvmTimer(hwCycles,
                                        KVM_KICK_SIGNAL,
                                        p->hostFactor,
                                        p->hostFreq));
    else
        runTimer.reset(new PosixKvmTimer(KVM_KICK_SIGNAL, CLOCK_MONOTONIC,
                                         p->hostFactor,
                                         p->hostFreq));

}

void
BaseKvmCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    numInsts
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        ;

    numUserInsts
        .name(name() + ".committedUserspaceInsts")
        .desc("Number of instructions committed")
        ;

    numUserInstsLoad
        .name(name() + ".userInstsLoad")
        .desc("number of committed userspace instructions with a load")
        ;

    numUserInstsStore
        .name(name() + ".userInstsStore")
        .desc("number of committed userspace instructions with a store")
        ;

    numVMExits
        .name(name() + ".numVMExits")
        .desc("total number of KVM exits")
        ;

    numVMHalfEntries
        .name(name() + ".numVMHalfEntries")
        .desc("number of KVM entries to finalize pending operations")
        ;

    numExitSignal
        .name(name() + ".numExitSignal")
        .desc("exits due to signal delivery")
        ;

    numMMIO
        .name(name() + ".numMMIO")
        .desc("number of VM exits due to memory mapped IO")
        ;

    numCoalescedMMIO
        .name(name() + ".numCoalescedMMIO")
        .desc("number of coalesced memory mapped IO requests")
        ;

    numIO
        .name(name() + ".numIO")
        .desc("number of VM exits due to legacy IO")
        ;

    numHalt
        .name(name() + ".numHalt")
        .desc("number of VM exits due to wait for interrupt instructions")
        ;

    numInterrupts
        .name(name() + ".numInterrupts")
        .desc("number of interrupts delivered")
        ;

    numHypercalls
        .name(name() + ".numHypercalls")
        .desc("number of hypercalls")
        ;

    numPageFaults
        .name(name() + ".numPageFaults")
        .desc("number of page faults")
        ;

}

void
BaseKvmCPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    if (DTRACE(Checkpoint)) {
        DPRINTF(Checkpoint, "KVM: Serializing thread %i:\n", tid);
        dump();
    }

    assert(tid == 0);
    assert(_status == Idle);
    thread->serialize(cp);
}

void
BaseKvmCPU::unserializeThread(CheckpointIn &cp, ThreadID tid)
{
    DPRINTF(Checkpoint, "KVM: Unserialize thread %i:\n", tid);

    assert(tid == 0);
    assert(_status == Idle);
    thread->unserialize(cp);
    threadContextDirty = true;
}

DrainState
BaseKvmCPU::drain()
{
    if (switchedOut())
        return DrainState::Drained;

    DPRINTF(Drain, "BaseKvmCPU::drain\n");

    // The event queue won't be locked when calling drain since that's
    // not done from an event. Lock the event queue here to make sure
    // that scoped migrations continue to work if we need to
    // synchronize the thread context.
    std::lock_guard<EventQueue> lock(*this->eventQueue());

    switch (_status) {
      case Running:
        // The base KVM code is normally ready when it is in the
        // Running state, but the architecture specific code might be
        // of a different opinion. This may happen when the CPU been
        // notified of an event that hasn't been accepted by the vCPU
        // yet.
        if (!archIsDrained())
            return DrainState::Draining;

        // The state of the CPU is consistent, so we don't need to do
        // anything special to drain it. We simply de-schedule the
        // tick event and enter the Idle state to prevent nasty things
        // like MMIOs from happening.
        if (tickEvent.scheduled())
            deschedule(tickEvent);
        _status = Idle;

        M5_FALLTHROUGH;
      case Idle:
        // Idle, no need to drain
        assert(!tickEvent.scheduled());

        // Sync the thread context here since we'll need it when we
        // switch CPUs or checkpoint the CPU.
        syncThreadContext();

        return DrainState::Drained;

      case RunningServiceCompletion:
        // The CPU has just requested a service that was handled in
        // the RunningService state, but the results have still not
        // been reported to the CPU. Now, we /could/ probably just
        // update the register state ourselves instead of letting KVM
        // handle it, but that would be tricky. Instead, we enter KVM
        // and let it do its stuff.
        DPRINTF(Drain, "KVM CPU is waiting for service completion, "
                "requesting drain.\n");
        return DrainState::Draining;

      case RunningMMIOPending:
        // We need to drain since there are in-flight timing accesses
        DPRINTF(Drain, "KVM CPU is waiting for timing accesses to complete, "
                "requesting drain.\n");
        return DrainState::Draining;

      case RunningService:
        // We need to drain since the CPU is waiting for service (e.g., MMIOs)
        DPRINTF(Drain, "KVM CPU is waiting for service, requesting drain.\n");
        return DrainState::Draining;

      default:
        panic("KVM: Unhandled CPU state in drain()\n");
        return DrainState::Drained;
    }
}

void
BaseKvmCPU::drainResume()
{
    assert(!tickEvent.scheduled());

    // We might have been switched out. In that case, we don't need to
    // do anything.
    if (switchedOut())
        return;

    DPRINTF(Kvm, "drainResume\n");
    verifyMemoryMode();

    // The tick event is de-scheduled as a part of the draining
    // process. Re-schedule it if the thread context is active.
    if (tc->status() == ThreadContext::Active) {
        schedule(tickEvent, nextCycle());
        _status = Running;
    } else {
        _status = Idle;
    }
}

void
BaseKvmCPU::notifyFork()
{
    // We should have drained prior to forking, which means that the
    // tick event shouldn't be scheduled and the CPU is idle.
    assert(!tickEvent.scheduled());
    assert(_status == Idle);

    if (vcpuFD != -1) {
        if (close(vcpuFD) == -1)
            warn("kvm CPU: notifyFork failed to close vcpuFD\n");

        if (_kvmRun)
            munmap(_kvmRun, vcpuMMapSize);

        vcpuFD = -1;
        _kvmRun = NULL;

        hwInstructions.detach();
        hwCycles.detach();
    }
}

void
BaseKvmCPU::switchOut()
{
    DPRINTF(Kvm, "switchOut\n");

    BaseCPU::switchOut();

    // We should have drained prior to executing a switchOut, which
    // means that the tick event shouldn't be scheduled and the CPU is
    // idle.
    assert(!tickEvent.scheduled());
    assert(_status == Idle);
}

void
BaseKvmCPU::takeOverFrom(BaseCPU *cpu)
{
    DPRINTF(Kvm, "takeOverFrom\n");

    BaseCPU::takeOverFrom(cpu);

    // We should have drained prior to executing a switchOut, which
    // means that the tick event shouldn't be scheduled and the CPU is
    // idle.
    assert(!tickEvent.scheduled());
    assert(_status == Idle);
    assert(threadContexts.size() == 1);

    // Force an update of the KVM state here instead of flagging the
    // TC as dirty. This is not ideal from a performance point of
    // view, but it makes debugging easier as it allows meaningful KVM
    // state to be dumped before and after a takeover.
    updateKvmState();
    threadContextDirty = false;
}

void
BaseKvmCPU::verifyMemoryMode() const
{
    if (!(system->bypassCaches())) {
        fatal("The KVM-based CPUs requires the memory system to be in the "
              "'noncaching' mode.\n");
    }
}

void
BaseKvmCPU::wakeup(ThreadID tid)
{
    DPRINTF(Kvm, "wakeup()\n");
    // This method might have been called from another
    // context. Migrate to this SimObject's event queue when
    // delivering the wakeup signal.
    EventQueue::ScopedMigration migrate(eventQueue());

    // Kick the vCPU to get it to come out of KVM.
    kick();

    if (thread->status() != ThreadContext::Suspended)
        return;

    thread->activate();
}

void
BaseKvmCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(Kvm, "ActivateContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    numCycles += ticksToCycles(thread->lastActivate - thread->lastSuspend);

    schedule(tickEvent, clockEdge(Cycles(0)));
    _status = Running;
}


void
BaseKvmCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(Kvm, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == Running || _status == RunningServiceCompletion);

    // The tick event may no be scheduled if the quest has requested
    // the monitor to wait for interrupts. The normal CPU models can
    // get their tick events descheduled by quiesce instructions, but
    // that can't happen here.
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    _status = Idle;
}

void
BaseKvmCPU::deallocateContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

void
BaseKvmCPU::haltContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}

ThreadContext *
BaseKvmCPU::getContext(int tn)
{
    assert(tn == 0);
    syncThreadContext();
    return tc;
}


Counter
BaseKvmCPU::totalInsts() const
{
    return ctrInsts;
}

Counter
BaseKvmCPU::totalOps() const
{
    hack_once("Pretending totalOps is equivalent to totalInsts()\n");
    return ctrInsts;
}

void
BaseKvmCPU::dump() const
{
    inform("State dumping not implemented.");
}

void
BaseKvmCPU::tick()
{
    Tick delay(0);
    assert(_status != Idle && _status != RunningMMIOPending);

    switch (_status) {
      case RunningService:
        // handleKvmExit() will determine the next state of the CPU
        delay = handleKvmExit();

        if (tryDrain())
            _status = Idle;
        break;

      case RunningServiceCompletion:
      case Running: {
          const uint64_t nextInstEvent(
              !comInstEventQueue[0]->empty() ?
              comInstEventQueue[0]->nextTick() : UINT64_MAX);
          // Enter into KVM and complete pending IO instructions if we
          // have an instruction event pending.
          const Tick ticksToExecute(
              nextInstEvent > ctrInsts ?
              curEventQueue()->nextTick() - curTick() : 0);

          if (alwaysSyncTC)
              threadContextDirty = true;

          // We might need to update the KVM state.
          syncKvmState();

          // Setup any pending instruction count breakpoints using
          // PerfEvent if we are going to execute more than just an IO
          // completion.
          if (ticksToExecute > 0 && !doSingleStep()) {
              setupInstStop();
              setupUserInstStop();
              setupUserInstLoadStop();
              setupUserInstStoreStop();
          }

          DPRINTF(KvmRun, "Entering KVM...\n");
          if (drainState() == DrainState::Draining) {
              // Force an immediate exit from KVM after completing
              // pending operations. The architecture-specific code
              // takes care to run until it is in a state where it can
              // safely be drained.
              delay = kvmRunDrain();
          } else {
              delay = kvmRun(ticksToExecute);
          }

          // The CPU might have been suspended before entering into
          // KVM. Assume that the CPU was suspended /before/ entering
          // into KVM and skip the exit handling.
          if (_status == Idle)
              break;

          // Entering into KVM implies that we'll have to reload the thread
          // context from KVM if we want to access it. Flag the KVM state as
          // dirty with respect to the cached thread context.
          kvmStateDirty = true;

          if (alwaysSyncTC)
              syncThreadContext();

          // Enter into the RunningService state unless the
          // simulation was stopped by a timer.
          if (_kvmRun->exit_reason !=  KVM_EXIT_INTR) {
              _status = RunningService;
          } else {
              ++numExitSignal;
              _status = Running;
          }

          // Service any pending instruction events. The vCPU should
          // have exited in time for the event using the instruction
          // counter configured by setupInstStop().
          comInstEventQueue[0]->serviceEvents(ctrInsts);
          system->instEventQueue.serviceEvents(system->totalNumInsts);
          comUserInstEventQueue[0]->serviceEvents(ctrUserInsts);
          system->userInstEventQueue.serviceEvents(system->totalNumUserInsts);

          if (tryDrain())
              _status = Idle;
      } break;

      default:
        panic("BaseKvmCPU entered tick() in an illegal state (%i)\n",
              _status);
    }

    // Schedule a new tick if we are still running
    if (_status != Idle && _status != RunningMMIOPending)
        schedule(tickEvent, clockEdge(ticksToCycles(delay)));
}

Tick
BaseKvmCPU::kvmRunDrain()
{
    // By default, the only thing we need to drain is a pending IO
    // operation which assumes that we are in the
    // RunningServiceCompletion or RunningMMIOPending state.
    assert(_status == RunningServiceCompletion ||
           _status == RunningMMIOPending);

    // Deliver the data from the pending IO operation and immediately
    // exit.
    return kvmRun(0);
}

uint64_t
BaseKvmCPU::getHostCycles() const
{
    return hwCycles.read();
}

Tick
BaseKvmCPU::kvmRun(Tick ticks)
{
    Tick ticksExecuted;
    fatal_if(vcpuFD == -1,
             "Trying to run a KVM CPU in a forked child process. "
             "This is not supported.\n");
    DPRINTF(KvmRun, "KVM: Executing for %i ticks\n", ticks);

    if (ticks == 0) {
        // Settings ticks == 0 is a special case which causes an entry
        // into KVM that finishes pending operations (e.g., IO) and
        // then immediately exits.
        DPRINTF(KvmRun, "KVM: Delivering IO without full guest entry\n");

        ++numVMHalfEntries;

        // Send a KVM_KICK_SIGNAL to the vCPU thread (i.e., this
        // thread). The KVM control signal is masked while executing
        // in gem5 and gets unmasked temporarily as when entering
        // KVM. See setSignalMask() and setupSignalHandler().
        kick();

        // Start the vCPU. KVM will check for signals after completing
        // pending operations (IO). Since the KVM_KICK_SIGNAL is
        // pending, this forces an immediate exit to gem5 again. We
        // don't bother to setup timers since this shouldn't actually
        // execute any code (other than completing half-executed IO
        // instructions) in the guest.
        ioctlRun();

        // We always execute at least one cycle to prevent the
        // BaseKvmCPU::tick() to be rescheduled on the same tick
        // twice.
        ticksExecuted = clockPeriod();
    } else if (doSingleStep()) {
        syncThreadContext();
        ThreadContext *tc(thread->getTC());

        StaticInstPtr inst = StaticInst::nullStaticInstPtr;
        StaticInstPtr last_inst = StaticInst::nullStaticInstPtr;
        while (doSingleStep()) {
            // maintain $r0 semantics
            thread->setIntReg(TheISA::ZeroReg, 0);

            TheISA::PCState pc_state = thread->pcState();
            if (isRomMicroPC(pc_state.microPC())) {
                inst = microcodeRom.fetchMicroop(pc_state.microPC(),
                                                 last_inst);
            } else {
                inst = getInst(tc, pc_state);
            }
            execute(inst);
            last_inst = inst;
        }

        updateKvmState();
    } else {
        // This method is executed as a result of a tick event. That
        // means that the event queue will be locked when entering the
        // method. We temporarily unlock the event queue to allow
        // other threads to steal control of this thread to inject
        // interrupts. They will typically lock the queue and then
        // force an exit from KVM by kicking the vCPU.
        EventQueue::ScopedRelease release(curEventQueue());

        if (ticks < runTimer->resolution()) {
            DPRINTF(KvmRun, "KVM: Adjusting tick count (%i -> %i)\n",
                    ticks, runTimer->resolution());
            ticks = runTimer->resolution();
        }

        // Get hardware statistics after synchronizing contexts. The KVM
        // state update might affect guest cycle counters.
        uint64_t baseCycles(getHostCycles());
        uint64_t baseInstrs(hwInstructions.read());
        uint64_t baseUserInstrs(hwUserInstructions.read());
        uint64_t baseUserInstrsLoad(hwUserInstLoad.read());
        uint64_t baseUserInstrsStore(hwUserInstStore.read());

        // Arm the run timer and start the cycle timer if it isn't
        // controlled by the overflow timer. Starting/stopping the cycle
        // timer automatically starts the other perf timers as they are in
        // the same counter group.
        runTimer->arm(ticks);
        if (!perfControlledByTimer)
            hwCycles.start();

        ioctlRun();

        runTimer->disarm();
        if (!perfControlledByTimer)
            hwCycles.stop();

        // The control signal may have been delivered after we exited
        // from KVM. It will be pending in that case since it is
        // masked when we aren't executing in KVM. Discard it to make
        // sure we don't deliver it immediately next time we try to
        // enter into KVM.
        discardPendingSignal(KVM_KICK_SIGNAL);

        const uint64_t hostCyclesExecuted(getHostCycles() - baseCycles);
        const uint64_t simCyclesExecuted(hostCyclesExecuted * hostFactor);
        const uint64_t instsExecuted(hwInstructions.read() - baseInstrs);
        const uint64_t userInstsExecuted(hwUserInstructions.read() -
                                         baseUserInstrs);
        const uint64_t userInstsLoadExecuted(hwUserInstLoad.read() -
                                             baseUserInstrsLoad);
        const uint64_t userInstsStoreExecuted(hwUserInstStore.read() -
                                              baseUserInstrsStore);
        ticksExecuted = runTimer->ticksFromHostCycles(hostCyclesExecuted);

        /* Update statistics */
        numCycles += simCyclesExecuted;;
        numInsts += instsExecuted;
        ctrInsts += instsExecuted;
        numUserInsts += userInstsExecuted;
        ctrUserInsts += userInstsExecuted;
        numUserInstsLoad += userInstsLoadExecuted;
        ctrUserInstsLoad += userInstsLoadExecuted;
        numUserInstsStore += userInstsStoreExecuted;
        ctrUserInstsStore += userInstsStoreExecuted;

        system->totalNumInsts += instsExecuted;
        system->totalNumUserInsts += userInstsExecuted;

        DPRINTF(KvmRun,
                "KVM: Executed %i instructions in %i cycles "
                "(%i ticks, sim cycles: %i).\n",
                instsExecuted, hostCyclesExecuted, ticksExecuted, simCyclesExecuted);
    }

    ++numVMExits;

    return ticksExecuted + flushCoalescedMMIO();
}

void
BaseKvmCPU::kvmNonMaskableInterrupt()
{
    ++numInterrupts;
    if (ioctl(KVM_NMI) == -1)
        panic("KVM: Failed to deliver NMI to virtual CPU\n");
}

void
BaseKvmCPU::kvmInterrupt(const struct kvm_interrupt &interrupt)
{
    ++numInterrupts;
    if (ioctl(KVM_INTERRUPT, (void *)&interrupt) == -1)
        panic("KVM: Failed to deliver interrupt to virtual CPU\n");
}

void
BaseKvmCPU::getRegisters(struct kvm_regs &regs) const
{
    if (ioctl(KVM_GET_REGS, &regs) == -1)
        panic("KVM: Failed to get guest registers\n");
}

void
BaseKvmCPU::setRegisters(const struct kvm_regs &regs)
{
    if (ioctl(KVM_SET_REGS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest registers\n");
}

void
BaseKvmCPU::getSpecialRegisters(struct kvm_sregs &regs) const
{
    if (ioctl(KVM_GET_SREGS, &regs) == -1)
        panic("KVM: Failed to get guest special registers\n");
}

void
BaseKvmCPU::setSpecialRegisters(const struct kvm_sregs &regs)
{
    if (ioctl(KVM_SET_SREGS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest special registers\n");
}

void
BaseKvmCPU::getFPUState(struct kvm_fpu &state) const
{
    if (ioctl(KVM_GET_FPU, &state) == -1)
        panic("KVM: Failed to get guest FPU state\n");
}

void
BaseKvmCPU::setFPUState(const struct kvm_fpu &state)
{
    if (ioctl(KVM_SET_FPU, (void *)&state) == -1)
        panic("KVM: Failed to set guest FPU state\n");
}


void
BaseKvmCPU::setOneReg(uint64_t id, const void *addr)
{
#ifdef KVM_SET_ONE_REG
    struct kvm_one_reg reg;
    reg.id = id;
    reg.addr = (uint64_t)addr;

    if (ioctl(KVM_SET_ONE_REG, &reg) == -1) {
        panic("KVM: Failed to set register (0x%x) value (errno: %i)\n",
              id, errno);
    }
#else
    panic("KVM_SET_ONE_REG is unsupported on this platform.\n");
#endif
}

void
BaseKvmCPU::getOneReg(uint64_t id, void *addr) const
{
#ifdef KVM_GET_ONE_REG
    struct kvm_one_reg reg;
    reg.id = id;
    reg.addr = (uint64_t)addr;

    if (ioctl(KVM_GET_ONE_REG, &reg) == -1) {
        panic("KVM: Failed to get register (0x%x) value (errno: %i)\n",
              id, errno);
    }
#else
    panic("KVM_GET_ONE_REG is unsupported on this platform.\n");
#endif
}

std::string
BaseKvmCPU::getAndFormatOneReg(uint64_t id) const
{
#ifdef KVM_GET_ONE_REG
    std::ostringstream ss;

    ss.setf(std::ios::hex, std::ios::basefield);
    ss.setf(std::ios::showbase);
#define HANDLE_INTTYPE(len)                      \
    case KVM_REG_SIZE_U ## len: {                \
        uint ## len ## _t value;                 \
        getOneReg(id, &value);                   \
        ss << value;                             \
    }  break

#define HANDLE_ARRAY(len)                               \
    case KVM_REG_SIZE_U ## len: {                       \
        uint8_t value[len / 8];                         \
        getOneReg(id, value);                           \
        ccprintf(ss, "[0x%x", value[0]);                \
        for (int i = 1; i < len  / 8; ++i)              \
            ccprintf(ss, ", 0x%x", value[i]);           \
        ccprintf(ss, "]");                              \
      } break

    switch (id & KVM_REG_SIZE_MASK) {
        HANDLE_INTTYPE(8);
        HANDLE_INTTYPE(16);
        HANDLE_INTTYPE(32);
        HANDLE_INTTYPE(64);
        HANDLE_ARRAY(128);
        HANDLE_ARRAY(256);
        HANDLE_ARRAY(512);
        HANDLE_ARRAY(1024);
      default:
        ss << "??";
    }

#undef HANDLE_INTTYPE
#undef HANDLE_ARRAY

    return ss.str();
#else
    panic("KVM_GET_ONE_REG is unsupported on this platform.\n");
#endif
}

void
BaseKvmCPU::syncThreadContext()
{
    if (!kvmStateDirty)
        return;

    assert(!threadContextDirty);

    updateThreadContext();
    kvmStateDirty = false;
}

void
BaseKvmCPU::syncKvmState()
{
    if (!threadContextDirty)
        return;

    assert(!kvmStateDirty);

    updateKvmState();
    threadContextDirty = false;
}

Tick
BaseKvmCPU::handleKvmExit()
{
    DPRINTF(KvmRun, "handleKvmExit (exit_reason: %i)\n", _kvmRun->exit_reason);
    assert(_status == RunningService);

    // Switch into the running state by default. Individual handlers
    // can override this.
    _status = Running;
    switch (_kvmRun->exit_reason) {
      case KVM_EXIT_UNKNOWN:
        return handleKvmExitUnknown();

      case KVM_EXIT_EXCEPTION:
        return handleKvmExitException();

      case KVM_EXIT_IO:
      {
        ++numIO;
        Tick ticks = handleKvmExitIO();
        _status = dataPort.nextIOState();
        return ticks;
      }

      case KVM_EXIT_HYPERCALL:
        ++numHypercalls;
        return handleKvmExitHypercall();

      case KVM_EXIT_HLT:
        /* The guest has halted and is waiting for interrupts */
        DPRINTF(Kvm, "handleKvmExitHalt\n");
        ++numHalt;

        // Suspend the thread until the next interrupt arrives
        thread->suspend();

        // This is actually ignored since the thread is suspended.
        return 0;

      case KVM_EXIT_MMIO:
      {
        /* Service memory mapped IO requests */
        DPRINTF(KvmIO, "KVM: Handling MMIO (w: %u, addr: 0x%x, len: %u)\n",
                _kvmRun->mmio.is_write,
                _kvmRun->mmio.phys_addr, _kvmRun->mmio.len);

        ++numMMIO;
        Tick ticks = doMMIOAccess(_kvmRun->mmio.phys_addr, _kvmRun->mmio.data,
                                  _kvmRun->mmio.len, _kvmRun->mmio.is_write);
        // doMMIOAccess could have triggered a suspend, in which case we don't
        // want to overwrite the _status.
        if (_status != Idle)
            _status = dataPort.nextIOState();
        return ticks;
      }

      case KVM_EXIT_IRQ_WINDOW_OPEN:
        return handleKvmExitIRQWindowOpen();

      case KVM_EXIT_FAIL_ENTRY:
        return handleKvmExitFailEntry();

      case KVM_EXIT_INTR:
        /* KVM was interrupted by a signal, restart it in the next
         * tick. */
        return 0;

      case KVM_EXIT_INTERNAL_ERROR:
        panic("KVM: Internal error (suberror: %u)\n",
              _kvmRun->internal.suberror);

      default:
        dump();
        panic("KVM: Unexpected exit (exit_reason: %u)\n", _kvmRun->exit_reason);
    }
}

Tick
BaseKvmCPU::handleKvmExitIO()
{
    panic("KVM: Unhandled guest IO (dir: %i, size: %i, port: 0x%x, count: %i)\n",
          _kvmRun->io.direction, _kvmRun->io.size,
          _kvmRun->io.port, _kvmRun->io.count);
}

Tick
BaseKvmCPU::handleKvmExitHypercall()
{
    panic("KVM: Unhandled hypercall\n");
}

Tick
BaseKvmCPU::handleKvmExitIRQWindowOpen()
{
    warn("KVM: Unhandled IRQ window.\n");
    return 0;
}


Tick
BaseKvmCPU::handleKvmExitUnknown()
{
    dump();
    panic("KVM: Unknown error when starting vCPU (hw reason: 0x%llx)\n",
          _kvmRun->hw.hardware_exit_reason);
}

Tick
BaseKvmCPU::handleKvmExitException()
{
    dump();
    panic("KVM: Got exception when starting vCPU "
          "(exception: %u, error_code: %u)\n",
          _kvmRun->ex.exception, _kvmRun->ex.error_code);
}

Tick
BaseKvmCPU::handleKvmExitFailEntry()
{
    dump();
    panic("KVM: Failed to enter virtualized mode (hw reason: 0x%llx)\n",
          _kvmRun->fail_entry.hardware_entry_failure_reason);
}

Tick
BaseKvmCPU::doMMIOAccess(Addr paddr, void *data, int size, bool write)
{
    ThreadContext *tc(thread->getTC());
    syncThreadContext();

    RequestPtr mmio_req = std::make_shared<Request>(
        paddr, size, Request::UNCACHEABLE, dataMasterId());

    mmio_req->setContext(tc->contextId());
    // Some architectures do need to massage physical addresses a bit
    // before they are inserted into the memory system. This enables
    // APIC accesses on x86 and m5ops where supported through a MMIO
    // interface.
    BaseTLB::Mode tlb_mode(write ? BaseTLB::Write : BaseTLB::Read);
    Fault fault(tc->getDTBPtr()->finalizePhysical(mmio_req, tc, tlb_mode));
    if (fault != NoFault)
        warn("Finalization of MMIO address failed: %s\n", fault->name());


    const MemCmd cmd(write ? MemCmd::WriteReq : MemCmd::ReadReq);
    PacketPtr pkt = new Packet(mmio_req, cmd);
    pkt->dataStatic(data);

    if (mmio_req->isMmappedIpr()) {
        // We currently assume that there is no need to migrate to a
        // different event queue when doing IPRs. Currently, IPRs are
        // only used for m5ops, so it should be a valid assumption.
        const Cycles ipr_delay(write ?
                             TheISA::handleIprWrite(tc, pkt) :
                             TheISA::handleIprRead(tc, pkt));
        delete pkt;
        return clockPeriod() * ipr_delay;
    } else {
        // Temporarily lock and migrate to the device event queue to
        // prevent races in multi-core mode.
        EventQueue::ScopedMigration migrate(deviceEventQueue());

        return dataPort.submitIO(pkt);
    }
}

void
BaseKvmCPU::setSignalMask(const sigset_t *mask)
{
    std::unique_ptr<struct kvm_signal_mask> kvm_mask;

    if (mask) {
        kvm_mask.reset((struct kvm_signal_mask *)operator new(
                           sizeof(struct kvm_signal_mask) + sizeof(*mask)));
        // The kernel and the user-space headers have different ideas
        // about the size of sigset_t. This seems like a massive hack,
        // but is actually what qemu does.
        assert(sizeof(*mask) >= 8);
        kvm_mask->len = 8;
        memcpy(kvm_mask->sigset, mask, kvm_mask->len);
    }

    if (ioctl(KVM_SET_SIGNAL_MASK, (void *)kvm_mask.get()) == -1)
        panic("KVM: Failed to set vCPU signal mask (errno: %i)\n",
              errno);
}

int
BaseKvmCPU::ioctl(int request, long p1) const
{
    if (vcpuFD == -1)
        panic("KVM: CPU ioctl called before initialization\n");

    return ::ioctl(vcpuFD, request, p1);
}

Tick
BaseKvmCPU::flushCoalescedMMIO()
{
    if (!mmioRing)
        return 0;

    DPRINTF(KvmIO, "KVM: Flushing the coalesced MMIO ring buffer\n");

    // TODO: We might need to do synchronization when we start to
    // support multiple CPUs
    Tick ticks(0);
    while (mmioRing->first != mmioRing->last) {
        struct kvm_coalesced_mmio &ent(
            mmioRing->coalesced_mmio[mmioRing->first]);

        DPRINTF(KvmIO, "KVM: Handling coalesced MMIO (addr: 0x%x, len: %u)\n",
                ent.phys_addr, ent.len);

        ++numCoalescedMMIO;
        ticks += doMMIOAccess(ent.phys_addr, ent.data, ent.len, true);

        mmioRing->first = (mmioRing->first + 1) % KVM_COALESCED_MMIO_MAX;
    }

    return ticks;
}

/**
 * Dummy handler for KVM kick signals.
 *
 * @note This function is usually not called since the kernel doesn't
 * seem to deliver signals when the signal is only unmasked when
 * running in KVM. This doesn't matter though since we are only
 * interested in getting KVM to exit, which happens as expected. See
 * setupSignalHandler() and kvmRun() for details about KVM signal
 * handling.
 */
static void
onKickSignal(int signo, siginfo_t *si, void *data)
{
}

void
BaseKvmCPU::setupSignalHandler()
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = onKickSignal;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    if (sigaction(KVM_KICK_SIGNAL, &sa, NULL) == -1)
        panic("KVM: Failed to setup vCPU timer signal handler\n");

    sigset_t sigset;
    if (pthread_sigmask(SIG_BLOCK, NULL, &sigset) == -1)
        panic("KVM: Failed get signal mask\n");

    // Request KVM to setup the same signal mask as we're currently
    // running with except for the KVM control signal. We'll sometimes
    // need to raise the KVM_KICK_SIGNAL to cause immediate exits from
    // KVM after servicing IO requests. See kvmRun().
    sigdelset(&sigset, KVM_KICK_SIGNAL);
    setSignalMask(&sigset);

    // Mask our control signals so they aren't delivered unless we're
    // actually executing inside KVM.
    sigaddset(&sigset, KVM_KICK_SIGNAL);
    if (pthread_sigmask(SIG_SETMASK, &sigset, NULL) == -1)
        panic("KVM: Failed mask the KVM control signals\n");
}

bool
BaseKvmCPU::discardPendingSignal(int signum) const
{
    int discardedSignal;

    // Setting the timeout to zero causes sigtimedwait to return
    // immediately.
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 0;

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, signum);

    do {
        discardedSignal = sigtimedwait(&sigset, NULL, &timeout);
    } while (discardedSignal == -1 && errno == EINTR);

    if (discardedSignal == signum)
        return true;
    else if (discardedSignal == -1 && errno == EAGAIN)
        return false;
    else
        panic("Unexpected return value from sigtimedwait: %i (errno: %i)\n",
              discardedSignal, errno);
}

void
BaseKvmCPU::setupCounters()
{
    DPRINTF(Kvm, "Attaching cycle counter...\n");
    PerfKvmCounterConfig cfgCycles(PERF_TYPE_HARDWARE,
                                PERF_COUNT_HW_CPU_CYCLES);
    cfgCycles.disabled(true)
        .pinned(true);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfgCycles.exclude_hv(true)
        .exclude_host(true);

    if (perfControlledByTimer) {
        // We need to configure the cycles counter to send overflows
        // since we are going to use it to trigger timer signals that
        // trap back into m5 from KVM. In practice, this means that we
        // need to set some non-zero sample period that gets
        // overridden when the timer is armed.
        cfgCycles.wakeupEvents(1)
            .samplePeriod(42);
    }

    hwCycles.attach(cfgCycles,
                    0); // TID (0 => currentThread)

    setupInstCounter();
    setupUserInstCounter();
    setupUserInstLoadCounter();
    setupUserInstStoreCounter();
}

bool
BaseKvmCPU::tryDrain()
{
    if (drainState() != DrainState::Draining)
        return false;

    if (!archIsDrained()) {
        DPRINTF(Drain, "tryDrain: Architecture code is not ready.\n");
        return false;
    }

    if (_status == Idle || _status == Running) {
        DPRINTF(Drain,
                "tryDrain: CPU transitioned into the Idle state, drain done\n");
        signalDrainDone();
        return true;
    } else {
        DPRINTF(Drain, "tryDrain: CPU not ready.\n");
        return false;
    }
}

void
BaseKvmCPU::ioctlRun()
{
    if (ioctl(KVM_RUN) == -1) {
        if (errno == EFAULT) {
            exitPageFault = true;
        } else if (errno != EINTR)
            panic("KVM: Failed to start virtual CPU (errno: %i)\n",
                  errno);
    }
}

void
BaseKvmCPU::setupInstStop()
{
    if (comInstEventQueue[0]->empty()) {
        setupInstCounter(0);
    } else {
        const uint64_t next(comInstEventQueue[0]->nextTick());

        assert(next > ctrInsts);
        setupInstCounter(next - ctrInsts);
    }
}

void
BaseKvmCPU::setupInstCounter(uint64_t period)
{
    // No need to do anything if we aren't attaching for the first
    // time or the period isn't changing.
    if (period == activeInstPeriod && hwInstructions.attached())
        return;

    PerfKvmCounterConfig cfgInstructions(PERF_TYPE_HARDWARE,
                                         PERF_COUNT_HW_INSTRUCTIONS);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfgInstructions.exclude_hv(true)
        .exclude_host(true);

    if (period) {
        // Setup a sampling counter if that has been requested.
        cfgInstructions.wakeupEvents(1)
            .samplePeriod(period);
    }

    // We need to detach and re-attach the counter to reliably change
    // sampling settings. See PerfKvmCounter::period() for details.
    if (hwInstructions.attached())
        hwInstructions.detach();
    assert(hwCycles.attached());
    hwInstructions.attach(cfgInstructions,
                          0, // TID (0 => currentThread)
                          hwCycles);

    if (period)
        hwInstructions.enableSignals(KVM_KICK_SIGNAL);

    activeInstPeriod = period;
}

void
BaseKvmCPU::setupUserInstStop()
{
    if (comUserInstEventQueue[0]->empty()) {
        setupUserInstCounter(0);
    } else {
        const uint64_t next(comUserInstEventQueue[0]->nextTick());

        assert(next > ctrUserInsts);
        setupInstCounter(next - ctrUserInsts);
    }
}

void
BaseKvmCPU::setupUserInstCounter(uint64_t period)
{
    // No need to do anything if we aren't attaching for the first
    // time or the period isn't changing.
    if (period == activeUserInstPeriod && hwUserInstructions.attached())
        return;

    PerfKvmCounterConfig cfgUserInstructions(PERF_TYPE_HARDWARE,
                                          PERF_COUNT_HW_INSTRUCTIONS);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfgUserInstructions.exclude_hv(true)
        .exclude_host(true)
        .exclude_kernel(true);

    if (period) {
        // Setup a sampling counter if that has been requested.
        cfgUserInstructions.wakeupEvents(1)
            .samplePeriod(period);
    }

    // We need to detach and re-attach the counter to reliably change
    // sampling settings. See PerfKvmCounter::period() for details.
    if (hwUserInstructions.attached())
        hwUserInstructions.detach();
    assert(hwCycles.attached());
    hwUserInstructions.attach(cfgUserInstructions,
                           0, // TID (0 => currentThread)
                           hwCycles);

    if (period)
        hwUserInstructions.enableSignals(KVM_KICK_SIGNAL);

    activeUserInstPeriod = period;
}

void
BaseKvmCPU::setupUserInstLoadStop()
{
    if (memSampler) {
        assert(memSampler->getLoadCount() == ctrUserInstsLoad);
        uint64_t next = memSampler->getNextLoadStop(1);
        if (next != (uint64_t)-1) {
            assert(next > ctrUserInstsLoad);
            uint64_t period = next - ctrUserInstsLoad;
            if (period > skidUserInstLoad) {
                period -= skidUserInstLoad;
            }
            setupUserInstLoadCounter(period);
        }
    }
}

#define PERF_EVENT_MEM_INST_RETIRED_LOADS 0x81d0

void
BaseKvmCPU::setupUserInstLoadCounter(uint64_t period)
{
    // No need to do anything if we aren't attaching for the first
    // time or the period isn't changing.
    if (period == activeUserInstLoadPeriod && hwUserInstLoad.attached())
        return;

    DPRINTF(KvmGuestDebug, "Simulation will stop after %d user loads\n",
            period);

    PerfKvmCounterConfig cfg(PERF_TYPE_RAW,
                             PERF_EVENT_MEM_INST_RETIRED_LOADS);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfg.exclude_hv(true)
        .exclude_host(true)
        .exclude_kernel(true);

    if (period) {
        cfg.wakeupEvents(1)
            .samplePeriod(period);
    }

    // We need to detach and re-attach the counter to reliably change
    // sampling settings. See PerfKvmCounter::period() for details.
    if (hwUserInstLoad.attached())
        hwUserInstLoad.detach();
    assert(hwCycles.attached());
    hwUserInstLoad.attach(cfg,
                           0, // TID (0 => currentThread)
                           hwCycles);

    if (period)
        hwUserInstLoad.enableSignals(KVM_KICK_SIGNAL);

    activeUserInstLoadPeriod = period;
}

void
BaseKvmCPU::setupUserInstStoreStop()
{
    if (memSampler) {
        assert(memSampler->getStoreCount() == ctrUserInstsStore);
        uint64_t next = memSampler->getNextStoreStop(1);
        if (next != (uint64_t)-1) {
            assert(next > ctrUserInstsStore);
            uint64_t period = next - ctrUserInstsStore;
            if (period > skidUserInstStore) {
                period -= skidUserInstStore;
            }
            setupUserInstStoreCounter(period);
        }
    }
}

#define PERF_EVENT_MEM_INST_RETIRED_STORES 0x82d0

void
BaseKvmCPU::setupUserInstStoreCounter(uint64_t period)
{
    // No need to do anything if we aren't attaching for the first
    // time or the period isn't changing.
    if (period == activeUserInstStorePeriod && hwUserInstStore.attached())
        return;

    DPRINTF(KvmGuestDebug, "Simulation will stop after %d user stores\n",
            period);

    PerfKvmCounterConfig cfg(PERF_TYPE_RAW,
                             PERF_EVENT_MEM_INST_RETIRED_STORES);

    // Try to exclude the host. We set both exclude_hv and
    // exclude_host since different architectures use slightly
    // different APIs in the kernel.
    cfg.exclude_hv(true)
        .exclude_host(true)
        .exclude_kernel(true);

    if (period) {
        // Setup a sampling counter if that has been requested.
        cfg.wakeupEvents(1)
            .samplePeriod(period);
    }

    // We need to detach and re-attach the counter to reliably change
    // sampling settings. See PerfKvmCounter::period() for details.
    if (hwUserInstStore.attached())
        hwUserInstStore.detach();
    assert(hwCycles.attached());
    hwUserInstStore.attach(cfg,
                            0, // TID (0 => currentThread)
                            hwCycles);

    if (period)
        hwUserInstStore.enableSignals(KVM_KICK_SIGNAL);

    activeUserInstStorePeriod = period;
}

Fault
BaseKvmCPU::readMem(Addr addr, uint8_t *data, unsigned size,
                    Request::Flags flags)
{
    // use the CPU's statically allocated read request and packet objects
    RequestPtr req = std::make_shared<Request>();
    req->setContext(vcpuID); // Add thread ID if we add MT

    if (traceData) {
        traceData->setMem(addr, size, flags);
    }

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    req->taskId(taskId());
    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(),
                    thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt(req, Packet::makeReadCmd(req));
            pkt.dataStatic(data);

            if (req->isMmappedIpr()) {
                TheISA::handleIprRead(thread->getTC(), &pkt);
            } else {
                if (system->isMemAddr(pkt.getAddr())) {
                    system->getPhysMem().access(&pkt);
                } else {
                    dataPort.sendAtomic(&pkt);
                }
            }

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        // If there's a fault or we don't need to access a second
        // cache line, stop now.
        if (fault != NoFault || secondAddr <= addr) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
BaseKvmCPU::writeMem(uint8_t *data, unsigned size, Addr addr,
                     Request::Flags flags, uint64_t *res)
{
    Addr cacheBlockMask = ~(cacheLineSize() - 1);
    static uint8_t zero_array[64] = {};

    if (data == NULL) {
        assert(size <= 64);
        assert(flags & Request::CACHE_BLOCK_ZERO);
        // This must be a cache block cleaning request
        data = zero_array;
    }

    RequestPtr req = std::make_shared<Request>();
    req->setContext(vcpuID); // Add thread ID if we add MT

    if (traceData) {
        traceData->setMem(addr, size, flags);
    }

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr) {
        size = secondAddr - addr;
    }

    req->taskId(taskId());
    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(),
                    thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                TheISA::handleLockedWrite(thread, req, cacheBlockMask);
            } else if (req->isSwap()) {
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt = Packet(req, Packet::makeWriteCmd(req));
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    system->getPhysMem().access(&pkt);
                }
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        // If there's a fault or we don't need to access a second
        // cache line, stop now.
        if (fault != NoFault || secondAddr <= addr) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        // Move the pointer we're reading into to the correct location.
        data += size;
        // Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        // And access the right address.
        addr = secondAddr;
    }
}

bool
BaseKvmCPU::doSingleStep()
{
    static int single_step_inst = 0;

    if (exitPageFault) {
        exitPageFault = false;
        single_step_inst = singleStepThreshold;
        return true;
    }

    if (memSampler) {
        uint64_t user_inst_load_stop = memSampler->getNextLoadStop();
        assert(user_inst_load_stop >= ctrUserInstsLoad);
        uint64_t user_inst_store_stop = memSampler->getNextStoreStop();
        assert(user_inst_store_stop >= ctrUserInstsStore);

        bool enable_ss = single_step_inst;
        if (user_inst_load_stop - ctrUserInstsLoad < singleStepThreshold) {
            DPRINTF(KvmGuestDebug, "Single step %i loads left\n",
                    user_inst_load_stop - ctrUserInstsLoad);
            enable_ss = true;
        }
        if (user_inst_store_stop - ctrUserInstsStore < singleStepThreshold) {
            DPRINTF(KvmGuestDebug, "Single step %i stores left\n",
                    user_inst_store_stop - ctrUserInstsStore);
            enable_ss = true;
        }
        if (enable_ss) {
            if (single_step_inst) {
                single_step_inst--;
            }
            return true;
        }
    }

    single_step_inst = 0;

    return false;
}

StaticInstPtr
BaseKvmCPU::getInst(ThreadContext *tc, TheISA::PCState &pc_state)
{
    FSTranslatingPortProxy *vp = &tc->getVirtProxy();

    StaticInstPtr instPtr = nullptr;

    TheISA::Decoder *decoder = tc->getDecoderPtr();
    decoder->reset();
    Addr fetch_pc = pc_state.pc() & BaseCPU::PCMask;
    do {
        TheISA::MachInst inst;
        vp->readBlob(fetch_pc, (uint8_t *)&inst, sizeof(TheISA::MachInst));
        decoder->moreBytes(pc_state, fetch_pc, inst);

        // decode an instruction if one is ready
        instPtr = decoder->decode(pc_state);
        fetch_pc += sizeof(TheISA::MachInst);
        // fetch more data if the decoder didn't return a valid
        // instruction
    } while (!instPtr);
    thread->pcState(pc_state);

    return instPtr;
}


void
BaseKvmCPU::updateCounters(const StaticInstPtr inst)
{
    if (!inst->isMicroop() || inst->isLastMicroop()) {
        numCycles++;
        numInsts++;
        ctrInsts++;

        system->totalNumInsts++;

        if (TheISA::inUserMode(tc)) {
            numUserInsts++;
            ctrUserInsts++;

            system->totalNumUserInsts++;
        }
    }

    if (inst->isMemRef()) {
        if (TheISA::inUserMode(tc)) {
            if (inst->isLoad()) {
                numUserInstsLoad++;
                ctrUserInstsLoad++;
            }
            if (inst->isStore()) {
                numUserInstsStore++;
                ctrUserInstsStore++;
            }
        }
    }

    if (memSampler) {
        memSampler->setTime(ctrUserInsts, ctrUserInstsLoad,
                            ctrUserInstsStore);
    }
}

void
BaseKvmCPU::execute(StaticInstPtr inst)
{
    StaticInstPtr macroInst = StaticInst::nullStaticInstPtr;
    if (inst->isMacroop())
        macroInst = inst;

    TheISA::PCState pc_state = thread->pcState();

    DPRINTF(KvmGuestDebug, "%s inst: %s\n", pc_state,
            inst->disassemble(thread->instAddr()));
    do {
        if (macroInst) {
            inst = macroInst->fetchMicroop(pc_state.microPC());
            DPRINTF(KvmGuestDebug, "%s micro-op: %s\n", pc_state,
                    inst->disassemble(thread->instAddr()));
        }

#if TRACING_ON
        traceData = tracer->getInstRecord(curTick(), thread->getTC(),
                                          inst, pc_state, macroInst);

#endif // TRACING_ON

        Fault fault = inst->execute(threadInfo, traceData);
        updateCounters(inst);

        if (fault != NoFault) {
            if (traceData) {
                if (DTRACE(ExecFaulting)) {
                    traceData->dump();
                }
                delete traceData;
                traceData = nullptr;
            }

            DPRINTF(KvmGuestDebug, "%s faulting inst: %s\n", pc_state,
                    inst->disassemble(thread->instAddr()));

            ThreadContext *tc = thread->getTC();
            fault->invoke(tc, inst);
            DPRINTF(KvmGuestDebug, "Fault at %s going to %s\n", pc_state,
                    tc->pcState());
            break;
        } else {
            pc_state = thread->pcState();
            TheISA::advancePC(pc_state, inst);
            thread->pcState(pc_state);

            if (traceData) {
                traceData->dump();
                delete traceData;
                traceData = nullptr;
            }
        }
    } while (macroInst && !inst->isLastMicroop());
}

void
BaseKvmCPU::handleKvmExitPageFault()
{
    ++numPageFaults;

    syncThreadContext();

    ThreadContext *tc(thread->getTC());
    TheISA::PCState pc_state = thread->pcState();
    DPRINTF(KvmGuestDebug, "Handling page fault %s\n", pc_state);
    assert(!isRomMicroPC(pc_state.microPC()));

    StaticInstPtr inst = getInst(tc, pc_state);
    execute(inst);

    updateKvmState();

    return;
}

