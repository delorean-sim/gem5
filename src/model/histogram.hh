/*
 * Copyright (c) 2015-2019 Nikos Nikoleris
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

#ifndef __MODEL_HISTOGRAM_HH__
#define __MODEL_HISTOGRAM_HH__

#include <algorithm>
#include <map>

#include "base/compiler.hh"
#include "base/types.hh"

template<class Dist, class Freq>
class Histogram
{
  public:
    using Data = typename std::map<Dist, Freq>;
    using value_type = typename Data::value_type;
    using iterator = typename Data::iterator;
    using const_iterator = typename Data::const_iterator;

  public:
    Histogram() : _total(0) {};

    void add(Dist dist, Freq freq = 1)
    {
        _total += freq;
        auto ret = data.emplace(dist, 0);
        auto it = ret.first;
        (*it).second += freq;
    }

    void add(const Histogram& hist)
    {
        for (const auto &it : hist) {
            add(it.first, it.second);
        }
    }

    void del(Dist dist, Freq freq = 1)
    {
        _total -= freq;
        auto it = data.find(dist);
        assert(it != data.end());
        (*it).second -= freq;
        assert((*it).second >= 0);
    }

    void set(Histogram hist)
    {
        data.clear();
        add(hist);
    }

    Histogram<Dist, float> getCCDF()
    {
        Histogram<Dist, float> res;
        Freq sum = 0;
        for (auto&& i : data) {
            sum += i.second;
            float p = 1. * sum / total();
            res.add(i.first, 1 - p);
        }
        return res;
    };

    iterator begin() { return data.begin(); }
    iterator end() { return data.end(); }
    const_iterator begin() const { return data.begin(); }
    const_iterator end() const { return data.end(); }

    Freq total() const
    {
        return _total;
    }

    void dump() const
    {
        for (const auto &o : data)
            inform("hist[%llu]=%f\n", o.first, o.second);
    }

  private:

    Data data;
    Freq _total;
};

#endif // __MODEL_HISTOGRAM_HH__
