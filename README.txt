CSSC_FTL_SemiAnalytic_Project

Main entry point:
  demos/run_demo_cssc2d.m

Initialization:
  root = initCSSCProjectPath;

Core idea:
  A 2D clamped cubic B-spline leader path is checked by fixed-length
  swept terminal segments M=r(u), N=r(v(u)), ||M-N||=L.

Solver modes:
  params.solver.gradMode = 'semi-analytic';   % recommended
  params.solver.gradMode = 'finite-diff';     % debugging/reference

Unified modules:
  src/bspline/      B-spline basis/evaluation utilities
  src/geometry/     obstacle definitions, SDF queries, segment clearance, fixed chord envelope
  src/objective/    objective value and semi-analytic gradient
  src/optimizer/    unified optimizer
  src/visualization plotting utilities

Obstacle interface:
  obsCircle2D(center, radius)
  obsRect2D(center, halfSize, yaw)

Notes:
  The semi-analytic gradient fixes u, v, and the closest-point alpha in the
  current iteration and back-propagates the obstacle clearance gradient to
  B-spline control points using basis weights.
