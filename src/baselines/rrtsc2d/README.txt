RRTSC-2D baseline

RRTSC-2D retains the RRT-smoothing-leading-link validation-global
replanning mechanism of RRTSC, while adapting the path representation,
obstacle model, and fixed-link validation to the common 2-D FTL framework
used in this project.

Main entry
  [Paccepted, pathRRT, info] = planRRTSC2D(startPt, goalPt, obstacles, opts)

Core behavior
  Each attempt runs a fresh RRT, converts the raw polyline directly to a
  cubic B-spline without shortcut, validates the dense centerline, and then
  validates all sampled fixed-length chords with exact segment clearance.
  Any rejection discards the candidate for strict acceptance and starts a
  fresh RRT. No local repair or CSSC optimizer is called.

Strict and fallback outputs
  info.success and info.strictAccepted indicate that both validation stages
  met safetyMargin. If the limits are exhausted first, the planner may
  return the best complete rejected candidate with info.outputAvailable=true
  and info.fallbackReturned=true. This degraded output is labelled by
  info.solutionStatus and must not be counted as strict RRTSC success.

Defaults
  defaultRRTSC2DParams.m defines standalone defaults. RRT values mirror
  planRRT2D; L=0.15, dMin=0.02, and degree=3 mirror stable project defaults.
  Formal comparisons must override these fields from the common experiment
  configuration. safetyMargin defaults to dMin.

Final statistics
  The planner does not call evaluateCSSCHighPrecision. The experiment
  runner calls it separately after planning so evaluation time remains
  outside planning time. Formal comparisons use the common 10 s budget from
  demos/getCSSCDemoConfig2D.m; standalone defaults remain defined in
  defaultRRTSC2DParams.m.
