Legacy folder notes

Files in this folder are historical debug scripts, timing probes, and
temporary experiment drivers. They are preserved for reference but are not
part of the stable v1 experiment interface.

Stable entry scripts are kept in demos/:
  run_demo_cssc2d.m
  demo1_scenes_display.m
  demo2_baseline_compare.m

Current stable core modules are under src/. New pilot experiments should
use evaluateCSSCHighPrecision for final statistics and save comparable
result.mat files through makeCSSCExperimentResult.

Legacy file categories:
  compare_fixedChordEnvelope_multi_cases.m
    Fixed-chord envelope accuracy/timing comparison.

  run_compare_grad_modes.m
    Semi-analytic vs finite-difference gradient comparison.

  run_debug_fixedChordEnvelope.m
  run_debug_segmentClearanceTiming.m
  run_debug_s_channel_seed_grid.m
    Focused debugging utilities.

  run_debug_scene3_s_channel_optimization2d.m
  run_experiment_scene3_s_channel_batch2d.m
    Older scene-3 troubleshooting and batch experiments.

  run_demo_multi_scenario_tests2d.m
  run_demo_structured_scenarios2d.m
    Superseded scene-generation demos.
