# Graphical editors: automata and Petri nets, with robot actions

Automata (`des` documents) and Petri nets (`petri` documents) can be drawn instead of typed. The **Diagram** view
of the Control tab is a graphical editor on top of the DSL: the diagram and the text are two views of the same
document — every edit in the diagram regenerates the text (node positions are kept in `layout` lines, so a drawing
survives save / load), and switching back to the diagram re-parses the text. Analysis, exports and the runtime see
one document.

```{image} ../_static/screens/57-editor-automaton.png
:alt: Automaton editor: the youBot base component of example A
:class: screenshot
:width: 900px
```

## Opening the editor

Select a `des` or `petri` model in the Control tab and press **Diagram** (next to the kind selector); **⛶** expands
the editor over the model list and the report. A document
with a syntax error cannot be drawn; the message names the line. For a fresh diagram create the model with
*New ▾ › Automata & supervisory control* or *Petri net* and switch to Diagram — the template is drawn and you edit
from there.

| Tool | Automaton | Petri net |
|---|---|---|
| **Select** | drag states, click a state / an edge to inspect it, double-click the canvas to add a state | drag places / transitions, click to inspect, double-click the canvas to add a place |
| **+ State** / **+ Place** | click on the canvas to add a state (`q`, `q1`, …) | click on the canvas to add a place |
| **+ Transition** | — | click on the canvas to add a transition (a bar) |
| **+ Edge** / **+ Arc** | click the source state, then the target: a dialog asks for the events (comma separated); an edge from a state to itself is a self-loop | click a place then a transition (or a transition then a place); arcs between two places or two transitions are refused |
| **Delete** | click a state or an edge; the `Delete` key removes the selection | click a place / transition / arc |
| **Auto layout** | recomputes the positions of the current automaton | recomputes the positions of the net |
| block selector, **+ Automaton**, **+ Spec**, **Remove** | one diagram per block: plant components and specification automata of the document | — |

`Escape` cancels an edge in progress and returns to Select.

### Inspector

The right-hand panel shows the properties of the selection and writes them into the document:

- **Document** (nothing selected): the model name; for automata the **uncontrollable**, **unobservable** and
  **fault** event lists (uncontrollable events are drawn dashed), the name and the **alphabet** of the current
  block (events a specification forbids without using them in a transition — E4 of the course — are declared
  here); for nets the name and the simulation horizon. `check` lines are kept as text.
- **State**: name (renaming updates every transition, the marking and the layout), **initial**, **marked**.
- **Edge**: the events of the edge (the list may be edited; an empty list removes the edge).
- **Place**: name, tokens, kind (idle / activity / resource / monitor / buffer / other — the kinds the monitor
  synthesis uses), capacity, label.
- **Transition**: name, deterministic delay, exponential rate, immediate flag, weight, priority, label.
- **Arc**: weight, inhibitor (place → transition only), reverse direction.

## Binding robot actions to states and transitions

```{image} ../_static/screens/58b-editor-petri-action.png
:alt: Petri-net editor: the transition t1a of the cell runs a robot program
:class: screenshot
:width: 900px
```

The same station holds the robot programs written in the Program tab, the targets taught in the 3D view, the
zones of the mobile robots and the signals of the process components. The **Entry action** panel of a state and
the **Operation** panel of a Petri transition bind one of them to the node, so that the model is not only
analysed but **executed on the station**:

| Action | What runs | Options |
|---|---|---|
| **run program** | a `Program` item of an arm robot in the program simulator (programs of one simulator run one at a time: a second one waits) | robot, program |
| **move to target** | a MoveJ (or MoveL) of an arm robot to a `Target` item, over the planned duration | robot, target, linear |
| **go to zone** | a mobile robot drives to a zone (the `goto` of the mission runtime) | robot, zone |
| **set signal** | a process signal (feeders, conveyors, machines react to it) | signal, value |
| **wait** | a delay | seconds |
| **emit event** | an uncontrollable event for the automaton / supervisor (an operator button, a sensor) | event |
| **set blackboard** | a key / value visible to conditions and monitors | key, value |

For an automaton state the **done event** names the event fired when the action finishes (it must be enabled in
that state — otherwise the run reports a *model mismatch*). In the text these bindings are `action` lines:

```text
action R.Loading program="Pick part" robot=UR10e done=loaded
action Base.mT goto=table robot=youBot done=b_arrive
action t1a program="Load M1" robot=KUKA
action t2 signal=start_M1 value=true
```

Nodes with an action carry a ▶ mark in the diagram, and the report lists them in a *Bound actions* section
(with a warning for a state or transition that does not exist).

### Running an automaton on the station

**▶ Run on station** executes every `automaton` block of the document as a state machine, in parallel,
synchronised on shared events:

1. entering a state starts its action; when the action succeeds the `done` event is fired;
2. controllable events are fired automatically when they are enabled in every component that shares them,
   allowed by the supervisor (synthesised from the `spec` blocks when there are any) and the components involved
   have finished their entry actions — fairly, the least recently fired event first, so no component starves;
3. uncontrollable events come from the world: arrivals of mobile robots, `emit event` actions, sensors.

The status line shows the state of every component, the running actions, the number of firings, mismatches and
supervisor denials; the run stops when nothing is enabled and nothing runs (quiescent), or with **⏹ Stop**. The
machine-and-robot cell of the test suite reads:

```text
des Machine and robot
automaton M
  initial Idle
  marked Idle
  Idle -load-> Busy
  Busy -done-> Idle
automaton R
  initial Free
  marked Free
  Free -load-> Loading
  Loading -loaded-> Free
uncontrollable done loaded
action R.Loading program="Pick part" robot=UR done=loaded
action M.Busy wait=1 done=done
```

`load` is shared: it fires when the machine is idle and the robot free; the robot's state runs the program, the
machine's state waits its cycle; each fires its own uncontrollable completion event, and the cycle repeats.

### Running a Petri net on the station

For a net, **▶ Run on station** treats transitions as operations: an enabled transition (priority first, then
document order) removes its input tokens and starts its action (or waits `delay=` seconds when it has none); the
output tokens are produced when the action finishes. A transition whose robot is busy simply waits, and the
marking shows where the cell is. The run stops when no transition is enabled and nothing is running — for the
deadlocking cell of example B that is the marking `{a1 b1}` the analysis predicted; with the monitor place added
in the editor the net runs forever.

## Layout and text

Positions are written as `layout` lines (`layout Base H=100,80 mT=220,80 …` per automaton, `layout p1=50,60 …`
for a net); they are ignored by the analysis. Removing them (or **Auto layout**) recomputes the picture. Comments
in the text are not preserved by the diagram writer; keep prose in `check` names or in the model name.
