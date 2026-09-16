# Process simulation components (Visual Components style)

*Add › Process component (VC-style)…* creates components that run on the world clock and exchange parts:

| Component | Behaviour |
|---|---|
| Feeder | Creates parts at an interval (or on signal) |
| Conveyor | Moves parts along its length at a speed; blocks when the next station is full |
| Process | Holds a part for a processing time; optional failure rate and reject output |
| Buffer | FIFO/LIFO storage with capacity |
| Sink | Consumes parts and counts throughput |

Components connect by proximity (output → next input) or explicitly; robots pick and place parts between them
with attach/detach events; **signals** synchronise programs with components (*Wait signal* until a part
arrives). The **Process** tab shows throughput, utilisation, WIP and blocked/starved times.

The packing-line demo (*File › Demo stations*) shows feeder → conveyor → grader with failures → conveyor →
pallet buffer → truck with a palletising robot.
