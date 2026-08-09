clear; clc; close all;

%% Small-scale pilot validation for the baseline comparison
% Select one benchmark scene, one difficulty level, and a small number of
% paired trials. This script calls simu1_baseline_compare.m, so its result
% directory, trial_results.csv, per-case result.mat files, metrics, and
% method settings are identical to the formal baseline experiment.
%
% Scene IDs:
%   "double_slit", "s_channel", "staggered_baffles", "random_mixed"
% Difficulty IDs:
%   "easy", "normal", "hard"
%
% To visualize the saved paths, set cfg.runName, cfg.sceneId,
% cfg.difficultyId, and cfg.numPanels in demo2_1_check_result.m to the
% values printed by this script.

%% Pilot selection
pilot.sceneId = "s_channel";
pilot.difficultyId = "normal";
pilot.numTrials = 30;
pilot.printEachRun = true;
pilot.saveCaseResults = true;

%% Run the shared baseline experiment
simu1Overrides = struct( ...
    'sceneId', pilot.sceneId, ...
    'difficultyId', pilot.difficultyId, ...
    'numTrials', pilot.numTrials, ...
    'printEachRun', pilot.printEachRun, ...
    'saveCaseResults', pilot.saveCaseResults);

run(fullfile(fileparts(mfilename('fullpath')), ...
    'simu1_baseline_compare.m'));

fprintf('\n[pilot validation complete]\n');
fprintf('  run name      : %s\n', cfg.runName);
fprintf('  scene         : %s\n', cfg.selectedSceneId);
fprintf('  difficulty    : %s\n', cfg.selectedDifficultyId);
fprintf('  trials        : %d\n', cfg.numSeedsPerScene);
fprintf('  result folder : %s\n', cfg.outDir);
fprintf('\n[demo2_1_check_result settings]\n');
fprintf('  cfg.runName = ''%s'';\n', cfg.runName);
fprintf('  cfg.sceneId = "%s";\n', cfg.selectedSceneId);
fprintf('  cfg.difficultyId = "%s";\n', cfg.selectedDifficultyId);

clear simu1Overrides;
