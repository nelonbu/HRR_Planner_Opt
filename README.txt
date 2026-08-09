CSSC-FTL MATLAB Project - Stable Framework v1

Project goal
  This version freezes the proposed method and experiment interface for
  pilot studies and paper statistics. Avoid adding temporary experiment
  logic to core modules; put exploratory scripts under legacy/.

Proposed method
  RRT initialization -> shortcut ->
  B-spline path initialization -> CSSC fixed-chord swept-envelope
  optimization -> semi-analytic gradient -> best-safe return with patience
  early stop. The optimizer returns the lowest-objective historically safe
  iterate when one exists; otherwise it falls back to the historical
  lowest-objective iterate.

  Baseline experiments use:
    frontendOpts.method = 'rrt';
    [path, info] = planCSSCFrontend2D(startPt, goalPt, obstacles, frontendOpts);

  Default:
    params.solver.gradMode = 'semi-analytic';

  Optional reference/debug mode:
    params.solver.gradMode = 'finite-diff';

Main entry scripts
  demos/getCSSCDemoConfig2D.m
    Shared scene, difficulty, planner, CSSC, and evaluation parameters used
    by the formal 2-D experiments and publication figures.

  demos/simu1_baseline_compare.m
    RRT / RRT* / RRTSC-2D / Adaptive Sp-RRT-2D / CSSC comparison on four
    scene families and three difficulty levels. Saves per-case result.mat,
    atomic checkpoints, provenance, summary tables, csv files, and figures.
    Re-run with the same runName to resume completed case IDs. The saved
    run definition prevents accidental resume with changed parameters.

  demos/simu2_cssc_ablation.m
    Paired CSSC ablation experiment. Core variants compare B-spline Init,
    Point-SDF, CSSC-FD, and CSSC-SAG from the same selected initialization;
    optional stability variants isolate initialization, return, active-set,
    clearance-buffer, and early-stopping choices.

  demos/fig_scenes_display.m
  demos/fig_Qualitative_result.m
  demos/fig_baseline_compare.m
    Publication-figure scripts. They read shared configuration or frozen
    result folders and do not form part of the planning-time measurement.

  demos/demo3_robot_following_simulation2d.m
    Full pipeline from scene generation and CSSC path optimization to
    equal-link FTL robot joint-parameter conversion, reconstruction, and
    animation output.

Initialization
  root = initCSSCProjectPath;

Stable modules
  src/bspline/       B-spline basis and path evaluation.
  src/environment/   Structured and randomized scene generation.
  src/frontend/      RRT, RRT*, RRT-Connect, selectable CSSC frontend,
                     shortcut, and polyline-to-B-spline initialization.
  src/baselines/     Formal comparison baselines. sprrt2d implements
                     reverse constrained tree growth with up to 12
                     fixed-length leader-path segments and deterministic
                     polyline pruning; rrtsc2d implements RRT -> smoothing
                     -> centerline/chord validation -> full global
                     replanning without local repair.
  src/geometry/      Obstacles, SDF, fixed-chord envelope, segment clearance.
  src/objective/     CSSC objective value and semi-analytic gradient.
  src/optimizer/     optimizeCSSC2D proposed optimizer.
  src/robot/         Equal-link FTL robot conversion, reconstruction,
                     visualization, and animation.
  src/evaluation/    Final statistics evaluators for cubic paths and native
                     polylines; polyline fixed chords use analytic roots.
  src/experiment/    Shared initialization and paired-ablation runners.
  src/output/        result folder and result.mat packing utilities.
  src/visualization/ Result and optimization-process plotting.

Obstacle interface
  Obstacles are MATLAB struct arrays. Supported stable types:
    obsCircle2D(center, radius)
    obsRect2D(center, halfSize, yaw)
    obsPolygon2D(vertices)

Evaluation rule for experiments
  Optimization may use faster internal sampling. Final statistics use the
  same high-precision fixed-chord clearance rule while preserving each
  method's native path representation:

    Cubic B-spline:
    metrics = evaluateCSSCHighPrecision(P, obstacles, opts);

    Native polyline:
    metrics = evaluatePolylineHighPrecision2D(path, obstacles, opts);

  Use metrics.minClear and metrics.dMinSatisfied for minimum clearance and
  the paper's whole-body dMin satisfaction rate. ftlGeometricRealizable is
  a separate numerical/geometric validity diagnostic. Where a strict joint
  flag is needed, wholeBodySuccess requires both conditions.

Experiment outputs
  Formal runs are written under results/runs/<run_name>/. Each run contains
  per-case result.mat files, trial_results.csv, grouped summary tables,
  provenance, metric definitions, and a resumable checkpoint. Publication
  figures should read a named frozen run rather than rerun planners.

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
  result.stageHighPrecisionMetrics.frontend
  result.stageHighPrecisionMetrics.initial
  result.stageHighPrecisionMetrics.final
  result.parameters.globalEvaluation
  result.parameters.globalOptimization
  result.parameters.effective
  result.provenance
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


