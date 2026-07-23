CSSC-FTL MATLAB Project - Stable Framework v1

Project goal
  This version freezes the proposed method and experiment interface for
  pilot studies and paper statistics. Avoid adding temporary experiment
  logic to core modules; put exploratory scripts under legacy/.

Proposed method
  RRT initialization -> shortcut -> B-spline path initialization ->
  CSSC fixed-chord swept-envelope optimization -> semi-analytic gradient ->
  bestP return with patience early stop.

  Default:
    params.solver.gradMode = 'semi-analytic';

  Optional reference/debug mode:
    params.solver.gradMode = 'finite-diff';

Main entry scripts
  demos/run_demo_cssc2d.m
    Single proposed-method demo. Saves result.mat through the stable result
    structure and reports high-precision final metrics.

  demos/demo1_scenes_display.m
    Generates representative scene figures for visual inspection.

  demos/demo2_baseline_compare.m
    Preliminary RRT / RRT* / CSSC comparison. Saves per-case result.mat
    files, summary tables, csv files, and boxplot figures.

  demos/demo3_robot_following_simulation2d.m
    Full pipeline from scene generation and CSSC path optimization to
    equal-link FTL robot joint-parameter conversion, reconstruction, and
    animation output.

Initialization
  root = initCSSCProjectPath;

Stable modules
  src/bspline/       B-spline basis and path evaluation.
  src/environment/   Structured and randomized scene generation.
  src/frontend/      RRT, RRT*, shortcut, and polyline-to-B-spline init.
  src/geometry/      Obstacles, SDF, fixed-chord envelope, segment clearance.
  src/objective/     CSSC objective value and semi-analytic gradient.
  src/optimizer/     optimizeCSSC2D proposed optimizer.
  src/robot/         Equal-link FTL robot conversion, reconstruction,
                     visualization, and animation.
  src/evaluation/    evaluateCSSCHighPrecision final statistics evaluator.
  src/output/        result folder and result.mat packing utilities.
  src/visualization/ Result and optimization-process plotting.

Obstacle interface
  Obstacles are MATLAB struct arrays. Supported stable types:
    obsCircle2D(center, radius)
    obsRect2D(center, halfSize, yaw)
    obsPolygon2D(vertices)

Evaluation rule for experiments
  Optimization may use faster internal sampling. Final statistics must use:
    metrics = evaluateCSSCHighPrecision(P, obstacles, opts);

  Use metrics.minClear, metrics.success, and metrics.dMinSatisfied for
  success rate, minimum clearance, and dMin satisfaction rate.

Stable result.mat structure
  result.version
  result.config
  result.seed
  result.obstacles
  result.paths.Pinit
  result.paths.Popt
  result.paths.Pref
  result.paths.pathRRT
  result.optimizerInfo
  result.highPrecisionMetrics
  result.timing
  result.successFlags

Timing/debug policy
  Timing diagnostics remain available but are off by default in formal
  runs. Control them through params.printEvalTiming,
  params.enableTimingDebug, params.enablePointClearance, and
  params.enablePathSample.

Legacy
  legacy/ contains older debug scripts and temporary experiments. They are
  kept for reference but are not stable entry points.


