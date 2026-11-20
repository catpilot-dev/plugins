# Arc Smooth toggle (lane_keeping bypass) — spec + plan

Date: 2026-11-20. Owner: PM (bmw session). Status: rev 2 after Director gate 1 REVISE. User decisions 2026-11-20: Q1 override accepted for ONE supervised drive; Q2 (D1) next drive start; Q3 (D4) temporary, BMW-only, dev only, off the release channel.

## Goal
A Driving-panel toggle "Arc Smooth" (default ON) that lets the BMW track the model's raw
`desiredCurvature` instead of lane_keeping's Arc, so a clean Cheddar drive can be run with the
Arc off and switched back without a code deploy.

## Background (verified 2026-11-20)
- Route 549 (Cheddar + Arc): car +38 cm right of lane centre on straights, 3 unsignalled right
  lane departures; raw Cheddar asked left (toward centre), the Arc target did not. The Arc is built
  on Cheddar's plan, which follows the car, so it drops the action head's centring. Open-loop raw
  Cheddar wander is ~1/4 of raw POP, but 7x its own Arc target (measured while the car followed
  the Arc). Only a closed-loop drive with raw tracking answers centring and real wander.
- lane_keeping = one hook, `controls.curvature_correction` (register.py `on_curvature_correction`,
  priority 50). Its telemetry (plugin bus, in the rlog) already carries `kappa_in` (raw) and
  `kappa_ref` (Arc output).
- **Do NOT disable the plugin to get "Arc off":** the BMW Direct controller only installs when
  `plugins.lane_keeping.register.HOOK_READY` is True (bmw/latcontroller.py:98-136, ruling R10);
  without lane_keeping the stock lateral controller runs. A bypass must keep the plugin loaded.
- lane_keeping is enforced (install.sh), declares no params; its config is read from
  `/data/plugins-runtime/lane_keeping/data/` once, at the first hook call of the controls process
  (`_load_config` in the lazy `_anchor` init). controlsd starts per onroad session.
- Driving-panel toggles: `ui.vehicle_settings(items, CP)` hook + `toggle_item` writing the plugin's
  `data/<key>` (pattern: bmw_e9x_e8x/register.py:150-177).
- **Safety-ruling override (Director gate 1):** raw curvature into the deadzone-free Direct tracker
  is a documented anti-goal: specs/2026-10-18-bmw-direct-tracking-design.md:84-85, bmw/latcontroller.py:88-91,
  install.sh:300-304 ("there is no user control"). Origin: 2026-07 (4db0b90/b02be73) — a toggle that
  short-circuited the hook fed raw POP into the tracker; 4d9 "dangerous fast swings"; lash then ~21 deg.
  Why one supervised drive is acceptable now: lash ~5 deg (v21 hub), raw Cheddar 0.1-1 Hz residual
  wander ~1/4 of raw POP (549 vs 548), raw is already tracked after the LC-onset hand-over today.
  Needs the user's explicit sign-off (Q1). T2 updates those three places + lane_keeping DESIGN.md
  (comment/doc only in the BMW file) to say: raw tracking only via this toggle, supervised test only.
- Lateral is frozen (user 2026-11-12); this toggle is the user's own request (2026-11-20) for a
  model experiment. Default ON keeps today's behaviour bit-identical.

## Scope
In: param `LaneKeepArcSmoothTest` ('1'/'0', missing = '1'); bypass in `on_curvature_correction`
(return the input curvature, still run the Arc so its state and telemetry stay live); telemetry
flag `arc_bypass`; Driving-panel toggle via a lane_keeping `ui.vehicle_settings` hook, shown only
when `CP.brand == 'bmw'`;
plugin.json hook + params entry; README/DESIGN note; tests.
Out: any change to the Arc law, the BMW controller, gains, clip; live mid-drive switching (see D1);
Cheddar swap/deploy (separate step, after this lands).

## Decisions
- **D1 (user: next drive start): when the toggle takes effect.** at the next drive
  start (controls process start). The value is read once per onroad session, so a mid-drive tap
  never steps the reference while engaged (Arc vs raw differ by up to ~1.5e-3 1/m on 549). UI text
  says "Takes effect on the next drive." Alternative: live with a 2 s crossfade (polls the param
  ~1 Hz) — more code and a lateral transient path to test; not recommended for a first version.
- D2: bypass returns the hook's input `curvature` unchanged. Downstream (controlsd clip, BMW Direct
  controller) is untouched, so "Arc off" = exactly what controlsd would track without the hook,
  but with the Direct controller still installed.
- D3: Arc keeps computing in bypass. `kappa_ref` keeps its documented meaning (the reference
  returned, = raw in bypass); the Arc's would-be output is published under a new key `kappa_arc_ref`
  (both modes), giving the reverse open-loop comparison on the test drive.
- D4 (user: TEMPORARY, BMW-only): temporary experiment (BMW-only row, dev only, kept off the release channel,
  removed or re-decided after the verdict) vs permanent toggle for all brands. PM recommends temporary.
- Abort path (follows from D1): a mid-drive tap does nothing. The way back is: disengage, stop,
  toggle ON, ignition cycle. Stated in the toggle text and T6.

## Requirements
R1 Default (no file, '1', unreadable) = Arc on, output bit-identical to today (existing tests pass
unchanged). R2 '0' = returned curvature == input curvature for every call, incl. lane changes and
crawl. R3 HOOK_READY stays True in both modes. R4 Value read once per process (D1); telemetry
publishes `arc_bypass` every call. R5 Toggle writes `data/LaneKeepArcSmoothTest` '1'/'0', initial state
from the file, lazy UI imports inside the hook fn (plugin UI rule). R6 Telemetry failure or a bad
param value never breaks the control path (bad value -> Arc on). R7 UI and controls parse
identically: only the exact string '0' = off. R8 Default mode output equals `LaneAnchor.update`
output across a scripted lane change (equality test).

## Tasks
| # | Task | Worker | Verification |
|---|---|---|---|
| T1 | Worktree off `origin/dev` (87796e7), branch `lane-keeping-arc-toggle`. | PM | `git log -1` = 87796e7 |
| T2 | Test-first: register tests for R1-R8 (bypass returns input; default on; bad value on; only exact '0' = off in UI and controls; read-once; HOOK_READY; `arc_bypass` + `kappa_arc_ref` telemetry; default-mode equality vs `LaneAnchor.update` across a lane change); UI hook test with a stub `toggle_item` incl. non-BMW `CP.brand` -> no row. Implement in register.py + plugin.json (`ui.vehicle_settings` hook, params entry). Doc/comment sites: spec 2026-10-18-bmw-direct-tracking-design.md:84-85, bmw/latcontroller.py:88-91 (comment only), install.sh:300-304, lane_keeping DESIGN.md (+README row). | engineer | Full lane_keeping + bmw suites pass; PM reruns; diff reviewed |
| T3 | Local commit (no push). | PM | `git log` |
| T4 | Gate 2 Director on diff + test output. | Director | APPROVE |
| T5 | With the user's go: push dev, deploy plugins to the C3 (install.sh), swap to Cheddar, user reboots, re-seal rootfs, verify branch/DisableUpdates, toggle visible in Driving panel, file written on tap. | PM | On-device checks; `.needs_restart` applied |
| T6 | Drive (user): Cheddar + Arc Smooth OFF, else as 549. Analysis: offset, line touches, sway, LC onset, torque reversals, lash. | user / PM | `arc_bypass` true in the rlog for the whole drive; pass criteria below |
| T7 | Close-out: toggle back ON (or remove per D4), swap model per verdict, reboot, confirm `arc_bypass` false on the next drive's first segment. Until then: hold rolling releases from dev (docs/RELEASE_PROCESS.md merges dev); after the verdict, revert the toggle on dev or re-decide it. | PM / user | rlog check |

## Drive pass criteria (T6)
Mean offset within +-15 cm of lane centre on straights (549: +38, 548: +12); 0 unsignalled line
touches; lane-centre 0.1-1 Hz sway <= 1.3x 3.8 cm; no LC-onset wrong-way worse than 549 raw (4/42);
servo lash p50 not > 5.5 deg. Any weave or drift: driver takes over; the Arc can only be restored by disengage, stop, toggle ON,
ignition cycle (D1).

## Risks and cost
- Raw reference has more 0.1-1 Hz content than the Arc the Direct controller was tuned on: more
  torque reversals, servo gear load. Mitigation: one short drive, lash check after.
- LC onset hand-over (515 fix) lives in the Arc, so it is inactive in bypass. Raw Cheddar showed
  4/42 wrong-way onset dips on 549.
- Toggle left OFF with POP would track raw POP (4x Cheddar's wander). Mitigation: default ON, the
  panel text names the purpose; PM switches it back after the drive.
- Concurrency: a `cheddar-deploy` worktree from the (closed) model-selector session sits at
  87796e7; no model-selector session is running (ListAgents 2026-11-20). This session owns dev and
  the car for this drive.
- Cost: engineer (Sonnet) one task, Director two short reviews. No paid APIs.
