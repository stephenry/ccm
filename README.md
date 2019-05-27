# Introduction

Presented is an implementation of a loosely-timed performance model
for the cache coherency protocols discussed in [1].

## Background

# Theory of operation

## Loosely-Time Simulation Kernel

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
build git:(master) ✗ ./driver/ccmd < ../cfgs/msi_simple.json
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
