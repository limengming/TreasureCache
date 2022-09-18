# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Simple test script
#
# "m5 test.py"

import argparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.params import NULL
from m5.util import addToPath, fatal, warn

addToPath('../')

from ruby import Ruby

from common import Options
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import ObjectList
from common import MemConfig
from common.FileSystemConfig import config_filesystem
from common.Caches import *
from common.cpu2000 import *

import spec17_benchmarks
import spectre_benchmark

def get_processes(args):
    """Interprets provided args and returns a list of processes"""

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = args.cmd.split(';')
    if args.input != "":
        inputs = args.input.split(';')
    if args.output != "":
        outputs = args.output.split(';')
    if args.errout != "":
        errouts = args.errout.split(';')
    if args.options != "":
        pargs = args.options.split(';')

    idx = 0
    for wrkld in workloads:
        process = Process(pid = 100 + idx)
        process.executable = wrkld
        process.cwd = os.getcwd()

        if args.env:
            with open(args.env, 'r') as f:
                process.env = [line.rstrip() for line in f]

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if args.smt:
        assert(args.cpu_type == "DerivO3CPU")
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

parser.add_option("-b", "--benchmark", type="string", default="", help="The SPEC benchmark to be loaded.")
parser.add_option("--benchmark_stdout", type="string", default="", help="Absolute path for stdout redirection for the benchmark.")
parser.add_option("--benchmark_stderr", type="string", default="", help="Absolute path for stderr redirection for the benchmark.")

if '--ruby' in sys.argv:
    Ruby.define_options(parser)

args = parser.parse_args()

#multiprocesses = []
numThreads = 1

if options.benchmark:
    print('Selected SPEC_CPU2017 benchmark')
    if options.benchmark == 'perlbench_r':
        print('--> perlbench_r')
        process = spec17_benchmarks.perlbench_r
    elif options.benchmark == 'perlbench_s':
        print('--> perlbench_s')
        process = spec17_benchmarks.perlbench_s
    elif options.benchmark == 'gcc_r':
        print('--> gcc_r')
        process = spec17_benchmarks.gcc_r
    elif options.benchmark == 'gcc_s':
        print('--> gcc_s')
        process = spec17_benchmarks.gcc_s
    elif options.benchmark == 'mcf_r':
        print('--> mcf_r')
        process = spec17_benchmarks.mcf_r
    elif options.benchmark == 'mcf_s':
        print('--> mcf_s')
        process = spec17_benchmarks.mcf_s
    elif options.benchmark == 'omnetpp_r':
        print('--> omnetpp_r')
        process = spec17_benchmarks.omnetpp_r
    elif options.benchmark == 'omnetpp_s':
        print('--> omnetpp_s')
        process = spec17_benchmarks.omnetpp_s
    elif options.benchmark == 'xalancbmk_r':
        print('--> xalancbmk_r')
        process = spec17_benchmarks.xalancbmk_r
    elif options.benchmark == 'xalancbmk_s':
        print('--> xalancbmk_s')
        process = spec17_benchmarks.xalancbmk_s
    elif options.benchmark == 'x264_r':
        print('--> x264_r')
        process = spec17_benchmarks.x264_r
    elif options.benchmark == 'x264_s':
        print('--> x264_s')
        process = spec17_benchmarks.x264_s
    elif options.benchmark == 'deepsjeng_r':
        print('--> deepsjeng_r')
        process = spec17_benchmarks.deepsjeng_r
    elif options.benchmark == 'deepsjeng_s':
        print('--> deepsjeng_s')
        process = spec17_benchmarks.deepsjeng_s
    elif options.benchmark == 'leela_r':
        print('--> leela_r')
        process = spec17_benchmarks.leela_r
    elif options.benchmark == 'leela_s':
        print('--> leela_s')
        process = spec17_benchmarks.leela_s
    elif options.benchmark == 'exchange2_r':
        print('--> exchange2_r')
        process = spec17_benchmarks.exchange2_r
    elif options.benchmark == 'exchange2_s':
        print('--> exchange2_s')
        process = spec17_benchmarks.exchange2_s
    elif options.benchmark == 'xz_r':
        print('--> xz_r')
        process = spec17_benchmarks.xz_r
    elif options.benchmark == 'xz_s':
        print('--> xz_s')
        process = spec17_benchmarks.xz_s
    elif options.benchmark == 'bwaves_r':
        print('--> bwaves_r')
        process = spec17_benchmarks.bwaves_r
    elif options.benchmark == 'bwaves_s':
        print('--> bwaves_s')
        process = spec17_benchmarks.bwaves_s
    elif options.benchmark == 'cactuBSSN_r':
        print('--> cactuBSSN_r')
        process = spec17_benchmarks.cactuBSSN_r
    elif options.benchmark == 'cactuBSSN_s':
        print('--> cactuBSSN_s')
        process = spec17_benchmarks.cactuBSSN_s
    elif options.benchmark == 'namd_r':
        print('--> namd_r')
        process = spec17_benchmarks.namd_r
    elif options.benchmark == 'parest_r':
        print('--> parest_r')
        process = spec17_benchmarks.parest_r
    elif options.benchmark == 'povray_r':
        print('--> povray_r')
        process = spec17_benchmarks.povray_r
    elif options.benchmark == 'lbm_r':
        print('--> lbm_r')
        process = spec17_benchmarks.lbm_r
    elif options.benchmark == 'lbm_s':
        print('--> lbm_s')
        process = spec17_benchmarks.lbm_s
    elif options.benchmark == 'wrf_r':
        print('--> wrf_r')
        process = spec17_benchmarks.wrf_r
    elif options.benchmark == 'wrf_s':
        print('--> wrf_s')
        process = spec17_benchmarks.wrf_s
    elif options.benchmark == 'blender_r':
        print('--> blender_r')
        process = spec17_benchmarks.blender_r
    elif options.benchmark == 'cam4_r':
        print('--> cam4_r')
        process = spec17_benchmarks.cam4_r
    elif options.benchmark == 'cam4_s':
        print('--> cam4_s')
        process = spec17_benchmarks.cam4_s
    elif options.benchmark == 'pop2_s':
        print('--> pop2_s')
        process = spec17_benchmarks.pop2_s
    elif options.benchmark == 'imagick_r':
        print('--> imagick_r')
        process = spec17_benchmarks.imagick_r
    elif options.benchmark == 'imagick_s':
        print('--> imagick_s')
        process = spec17_benchmarks.imagick_s
    elif options.benchmark == 'nab_r':
        print('--> nab_r')
        process = spec17_benchmarks.nab_r
    elif options.benchmark == 'nab_s':
        print('--> nab_s')
        process = spec17_benchmarks.nab_s
    elif options.benchmark == 'fotonik3d_r':
        print('--> fotonik3d_r')
        process = spec17_benchmarks.fotonik3d_r
    elif options.benchmark == 'fotonik3d_s':
        print('--> fotonik3d_s')
        process = spec17_benchmarks.fotonik3d_s
    elif options.benchmark == 'roms_r':
        print('--> roms_r')
        process = spec17_benchmarks.roms_r
    elif options.benchmark == 'roms_s':
        print('--> roms_s')
        process = spec17_benchmarks.roms_s
    elif options.benchmark == 'cons':
        print '--> cons'
        process = cons_benchmark.cons
    else:
        print("No recognized SPEC2017 benchmark selected! Exiting.")
        sys.exit(1)
else:
    print >> sys.stderr, "Need --benchmark switch to specify SPEC CPU2017 workload. Exiting!\n"
    sys.exit(1)

# Set process stdout/stderr
if options.benchmark_stdout:
    process.output = options.benchmark_stdout
    print "Process stdout file: " + process.output
if options.benchmark_stderr:
    process.errout = options.benchmark_stderr
    print "Process stderr file: " + process.errout

if args.bench:
    apps = args.bench.split("-")
    if len(apps) != args.num_cpus:
        print("number of benchmarks not equal to set num_cpus!")
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv['TARGET_ISA'] == 'arm':
                exec("workload = %s('arm_%s', 'linux', '%s')" % (
                        app, args.arm_iset, args.spec_input))
            else:
                exec("workload = %s(buildEnv['TARGET_ISA', 'linux', '%s')" % (
                        app, args.spec_input))
            multiprocesses.append(workload.makeProcess())
        except:
            print("Unable to find workload for %s: %s" %
                  (buildEnv['TARGET_ISA'], app),
                  file=sys.stderr)
            sys.exit(1)
elif args.cmd:
    multiprocesses, numThreads = get_processes(args)
else:
    print("No workload specified. Exiting!\n", file=sys.stderr)
    sys.exit(1)


(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)
CPUClass.numThreads = numThreads

# Check -- do not allow SMT with multiple CPUs
if args.smt and args.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

np = args.num_cpus
mp0_path = multiprocesses[0].executable
system = System(cpu = [CPUClass(cpu_id=i) for i in range(np)],
                mem_mode = test_mem_mode,
                mem_ranges = [AddrRange(args.mem_size)],
                cache_line_size = args.cacheline_size)

if numThreads > 1:
    system.multi_thread = True

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = args.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  args.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = args.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# If elastic tracing is enabled, then configure the cpu and attach the elastic
# trace probe
if args.elastic_trace_en:
    CpuConfig.config_etrace(CPUClass, system.cpu, args)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

if ObjectList.is_kvm_cpu(CPUClass) or ObjectList.is_kvm_cpu(FutureClass):
    if buildEnv['TARGET_ISA'] == 'x86':
        system.kvm_vm = KvmVM()
        for process in multiprocesses:
            process.useArchPT = True
            process.kvmInSE = True
    else:
        fatal("KvmCPU can only be used in SE mode with x86")

# Sanity check
if args.simpoint_profile:
    if not ObjectList.is_noncaching_cpu(CPUClass):
        fatal("SimPoint/BPProbe should be done with an atomic cpu")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

for i in range(np):
    if args.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if args.simpoint_profile:
        system.cpu[i].addSimPointProbe(args.simpoint_interval)

    if args.checker:
        system.cpu[i].addCheckerCpu()

    if args.bp_type:
        bpClass = ObjectList.bp_list.get(args.bp_type)
        system.cpu[i].branchPred = bpClass()

    if args.indirect_bp_type:
        indirectBPClass = \
            ObjectList.indirect_bp_list.get(args.indirect_bp_type)
        system.cpu[i].branchPred.indirectBranchPred = indirectBPClass()

    system.cpu[i].createThreads()

if args.ruby:
    Ruby.create_system(args, False, system)
    assert(args.num_cpus == len(system.ruby._cpu_ports))

    system.ruby.clk_domain = SrcClockDomain(clock = args.ruby_clock,
                                        voltage_domain = system.voltage_domain)
    for i in range(np):
        ruby_port = system.ruby._cpu_ports[i]

        # Create the interrupt controller and connect its ports to Ruby
        # Note that the interrupt controller is always present but only
        # in x86 does it have message ports that need to be connected
        system.cpu[i].createInterruptController()

        # Connect the cpu's cache ports to Ruby
        ruby_port.connectCpuPorts(system.cpu[i])
else:
    MemClass = Simulation.setMemClass(args)
    system.membus = SystemXBar()
    system.system_port = system.membus.slave
    CacheConfig.config_cache(args, system)
    MemConfig.config_mem(args, system)
    config_filesystem(system, args)

system.workload = SEWorkload.init_compatible(mp0_path)

if args.wait_gdb:
    system.workload.wait_for_remote_gdb = True

root = Root(full_system = False, system = system)
Simulation.run(args, root, system, FutureClass)
