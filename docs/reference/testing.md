# Testing and module verification

The studio is verified on four layers, all runnable from `studio/` with `npm run test:all` (build, unit tests
with coverage, browser end-to-end, demo scenarios, report):

| Layer | Command | What it checks |
|---|---|---|
| Unit and integration tests | `npm test` (`npm run test:coverage` for coverage) | Every module of `src/` in Node or jsdom: kinematics, programs, posts, file formats, the RoboDK-compatible API, control design, group control, vision, navigation, fleets, the UI panels and the `App` class. |
| A/B tests | part of `npm test` (`tests/ab_variants.test.ts`) | The same job done two ways with the relation between the results asserted: a saved station against its reloaded copy, the simulator against every post processor, MoveL against MoveJ, the optimal solver against the heuristic (Hungarian / greedy / auction / CBBA, CBS / prioritised planning, EDD / SPT / Moore), the mission model against the fleet driven by pose and through the unicycle controller, one group architecture against another, a connected radio graph against a partitioned one, the plant against its supervisor, RTK against dead reckoning, English against Russian. |
| Browser end-to-end | `npm run test:e2e` (after `npm run build`) | Playwright drives the production build in Chromium: every demo station, every menu entry and dialog, every bottom tab, the RoboDK console, teaching and running a program, export with every post, undo / redo, save → reload, the Control and Group tabs (course examples, *Analyse all*, diagram editors, *Run on fleet*), the whole demo-scenario suite from the dialog, the language switch. A page error or a console error anywhere fails the test. |
| Demo scenarios | `npm run scenarios` | The reproducible cases of {doc}`scenarios`: one per navigation method, vision task, control-design method and group-control practicum, each with metrics compared against an expectation. |

`npm run test:report` collects the results into the report below (`studio/docs/test-report.md`).

## How the layers fit together

- **Coverage** tells where a module has no test at all; the report lists the lowest files of every module.
  The renderer (`scene/`) runs only in a browser and is exercised by the end-to-end layer, which does not
  contribute to the coverage figure.
- **A/B pairs** guard properties that a single test cannot: optimality (the exact solver is never worse than the
  heuristic), fidelity (the analysis model predicts what the fleet does), symmetry (save → load → save is the
  identity, EN → RU → EN is the identity) and the difference that a feature is supposed to make (a supervisor
  removes exactly the forbidden behaviour, a no-go zone makes the robot leave, GNSS outage makes the estimate drift).
- **End-to-end** runs catch what unit tests structurally cannot: wiring between menus, dialogs, panels and the
  station; bundling; the WebGL scene; the real timers of the world clock.

## Writing a new test

- A pure function or a model: a unit test in `tests/<module>.test.ts` (Node environment).
- A panel, a dialog or an `App` method: a jsdom test — put `// @vitest-environment jsdom` on the first line and
  build the UI as `tests/ui_harness.ts` does (an `App` without `mount()`, the renderer stubbed).
- A behaviour that only exists in the browser: add a case to `tests/e2e/ui.e2e.test.ts` (menus are clicked by
  label, dialogs by their `.dialog` buttons; keep the heavy demo scenes to the demo test — software GL is slow).
- Two ways of doing the same thing: an A/B pair in `tests/ab_variants.test.ts`, named `A: … · B: … — <relation>`.

## Latest report

```{include} ../../studio/docs/test-report.md
```
