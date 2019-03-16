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

#include "model/watchpoints.hh"

#include <sys/mman.h>

#include "base/intmath.hh"
#include "debug/Watchpoints.hh"

void
KvmWatchpoints::insert(Addr phys_addr)
{
  Addr page_addr = roundDown(phys_addr, pageSize);
  auto it = watchpages.find(page_addr);
  if (it == watchpages.end()) {
      Watchpage wp;
      wp.bStoreAddr = system->getPhysMem().getPMem(page_addr);
      wp.count = 1;
      wp.enabled = false;
      watchpages.emplace(page_addr, wp);

      DPRINTF(Watchpoints, "Added watchpoint on address %#x\n", page_addr);
  } else {
      Watchpage &wp = it->second;
      wp.count++;
  }
}

void
KvmWatchpoints::remove(Addr phys_addr)
{
    const Addr page_addr = roundDown(phys_addr, pageSize);
    auto it = watchpages.find(page_addr);
    if (it != watchpages.end()) {
        Watchpage &wp = it->second;

        assert(wp.count > 0);
        wp.count--;
        if (!wp.count) {
            // if the was the last watchpoint on the page then remove the
            // protection as well
            if (wp.enabled) {
                unprotect(wp.bStoreAddr);
            }
            watchpages.erase(it);

            DPRINTF(Watchpoints, "Removed watchpoint from address %#x\n",
                    page_addr);
        }
    }
}

bool
KvmWatchpoints::enable(Addr phys_addr)
{
    const Addr page_addr = roundDown(phys_addr, pageSize);
    auto it = watchpages.find(page_addr);
    if (it != watchpages.end()) {
        Watchpage &wp = it->second;
        if (!wp.enabled) {
            DPRINTF(Watchpoints, "Enabled watchpoint at address %#x\n",
                    page_addr);
            protect(wp.bStoreAddr);
            wp.enabled = true;
            return true;
        }
    }
    return false;
}

bool
KvmWatchpoints::disable(Addr phys_addr)
{
    const Addr page_addr = roundDown(phys_addr, pageSize);
    auto it = watchpages.find(page_addr);
    if (it != watchpages.end()) {
        numHits.sample(page_addr);

        Watchpage &wp = it->second;
        if (wp.enabled) {
            DPRINTF(Watchpoints, "Disabled watchpoint from address %#x\n",
                    page_addr);
            unprotect(wp.bStoreAddr);
            wp.enabled = false;
            return true;
        }
    }
    return false;
}

void
KvmWatchpoints::clear()
{
    for (const auto &wit : watchpages) {
        const Watchpage &wp = wit.second;
        unprotect(wp.bStoreAddr);
    }
    watchpages.clear();
}

bool
KvmWatchpoints::empty() const
{
    return watchpages.empty();
}

void
KvmWatchpoints::protect(void* page_addr)
{
    const int prot = PROT_NONE;
    if (mprotect(page_addr, pageSize, prot) == -1) {
        panic("mprotect for %x (%x) failed: %s", page_addr, pageSize,
              strerror(errno));
    }
}

void
KvmWatchpoints::unprotect(void* page_addr)
{
    const int prot = PROT_READ | PROT_WRITE;
    if (mprotect(page_addr, pageSize, prot) == -1) {
        panic("munprotect failed: %s", strerror(errno));
    }
}

void*
KvmWatchpoints::physToBStoreAddr(Addr physPageAddr)
{
    Watchpages::iterator wit = watchpages.find(physPageAddr);
    assert(wit != watchpages.end());
    return wit->second.bStoreAddr;
}

void
KvmWatchpoints::dump() const
{
    for (const auto &wit : watchpages) {
        const Watchpage &wp = wit.second;
        inform("Watchpoint(s) on page %#llx (%llx)\n", wp.physAddr);
    }
}

void
KvmWatchpoints::regStats()
{
    using namespace Stats;

    numHits
        .init(0)
        .name(name() + ".hits")
        .desc("number of watchpoint hits")
        .flags(total|nozero);
    ;
}
