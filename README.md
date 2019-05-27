# Introduction

Presented is an implementation of a loosely-timed performance model
for the cache coherency protocols discussed in [1]. The model
presently supports the: MSI, MESI and MOSI protocols. Additional
protocols may be added trivially as they become available.

## Background

In multi-processor systems, multiple agent may maintain copies of the
same cache lines. Coherence protocols are used to maintain the Single
Writer/Multiple Read (SWMR) invariant, whereby ownership of lines are
managed between caches in a correct and performant manner. Although
coherency protocols may have a very significant bearing on the overall
performance of a multiprocessor system, the ability to model their
behavior at a high-level is quite limited. Furthermore, a coherency
system has a large number of parameterizations that unknown trade-off
in terms of performance and/or silicon cost.

The purpose of this work is to sketch out a rough proof of concept
cache coherency performance model using freely available coherency
protocol specifications [1]. The model is not intended to be either
fully representative of a full system, nor is it remotely production
quality.


# Theory of operation

## Loosely-Time Simulation Kernel

CCM implements the notion of a Loosely-Timed (LT) simulation model. In
this approach, each agent has its own local notion of time. During
simulation, each agent is stepped sequentially for some fixed period
of time (known as an Epoch) where it is free to consume and emit
messages to other agents in the system. Upon completion of the Epoch,
the simulation kernel, moves to the next agent. Once all agents have
been executed the current Epoch, simulation time advances and the
process is restarted.

Each agent in the system implements a model to denote the cost of each
operation executed during the Epoch. As the model executes, agents
incur the cost of each operation (such as the issue of a message to
the snoop controller). Operations are themselve indivisible and may
transverse multiple Epochs, in this case, the agent blockes until
simulation time advances beyond the local time of the agent.

An alternative simualtion model is the Discrete-Timed Event (DTE)
model, as seen in libraries such as SystemC or languages such as
Verilog. Although the DTE approach results in a more accurate model,
as it more closely models the notion of time between agents, it
requires a more detailed understanding of the system, beyond that
which was available within the context of this work.

# Useage

Simulations are invoked (at present) by passing a JSON configuration
file to the driver executable. This JSON file describes the agents
present in the platform along with any stimulus to be issued. The
driver may be invoked by passing one of the configurations contained
in the 'cfg' directory to the drive on standard input. Eventually, it
is planned that a parent Python script shall construct the
configuration file, invoke the driver executable and collate emitted
performance merits; although this does not exist at present.

```
build git:(master)> ./driver/ccmd < ../cfgs/msi_simple.json
```

On running the simple example, a load command is issued by 'agent0'
and the line installed in the SHARED state in its cache. Three load
commands are then issued to the same line and they complete
immediately upon the hit to the local cache.

```
[top.agent0@1000]: Transaction TID=0 START
[top.agent0@1010]: Execute: UpdateState
[top.agent0@1010]: Update state; current: IS_D previous: I
[top.agent0@1020]: Execute: EmitGetS
[top.agent0@1020]: Sending GetS to home directory: '{type:GetS, src_id:0, dst_id:4, transaction:'{type:0, addr:1000, tid:0}, is_ack:0}
[top.agent0@1060]: Transaction TID=1 START
[top.snoopfilter@1040]: Execute: SendDataToReq
[top.snoopfilter@1040]: Sending data to requester: '{type:Data, src_id:4, dst_id:0, transaction:'{type:0, addr:1000, tid:0}, is_ack:0, ack_count:0, is_exclusive:0}
[top.snoopfilter@1040]: Execute: AddReqToSharers
[top.snoopfilter@1040]: Add requester to sharers:  before = '{} after = '{0}
[top.snoopfilter@1080]: Execute: UpdateState
[top.snoopfilter@1080]: Update state; current: S previous: I
[top.agent0@1090]: Execute: UpdateState
[top.agent0@1090]: Update state; current: S previous: IS_D
[top.agent0@1100]: Transaction TID=0 END
[top.agent0@1100]: Transaction TID=1 START
[top.agent0@1110]: Transaction TID=1 END
[top.agent0@1110]: Transaction TID=2 START
[top.agent0@1120]: Transaction TID=2 END
[top.agent0@1120]: Transaction TID=3 START
[top.agent0@1130]: Transaction TID=3 END
```

More sophisticated scenarios can be found in the unit tests.

# Remaining work

# References

[1]: Sorin, D. J. et. al "A Primer on Memory Consistency and Cache Coherence", Synthesis Lectures on Computer Architecture, Morgan & Claypool Publishers, 2011.
