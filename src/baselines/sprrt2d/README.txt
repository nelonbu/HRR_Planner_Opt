Adaptive Sp-RRT-2D baseline
============================

Adaptive Sp-RRT-2D retains the goal-rooted reverse exploration,
fixed-length expansion, bounded local turning angles, alternative-nearest-
node expansion, and angle-preserving shortcut mechanisms of Sp-RRT.
Unlike the original formulation, tree depth is the number of fixed-length
leader-path segments rather than the number of physical robot links. The
segment count grows naturally up to 12, avoiding failure caused solely by
the nominal six-link total length.

Formal defaults:
  nominalLinkCount = 6
  maxSegmentCount = 12
  L = 0.15
  thetaMax = deg2rad(40)
  entranceBias = 0.05
  maxTimeSec = Inf
  pathOptimization.mode = 'non-worsening-angle'

The planner checks only centerline segments and local path turns. It never
calls CSSC optimization and never replans after whole-body evaluation.
evaluateSpRRTPath2D retains the optimized path as a native polyline and
uses analytic segment-circle roots for fixed-chord evaluation.
