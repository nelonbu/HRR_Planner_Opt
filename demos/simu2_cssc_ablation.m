function output = simu2_cssc_ablation(overrides)
%SIMU2_CSSC_ABLATION Paired ablation experiment for the CSSC pipeline.
%
% Typical use:
%   simu2_cssc_ablation();
%   simu2_cssc_ablation(struct('profile','smoke'));
%   simu2_cssc_ablation(struct('profile','pilot', ...
%       'sceneId','s_channel','difficultyId','hard','numTrials',20));
%   simu2_cssc_ablation(struct('profile','formal', ...
%       'variantGroup','core'));
%
% Profiles: smoke, pilot, formal. Variant groups: core, stability, all.
% Core variants share the exact same selected RRT/B-spline initialization.
% Final safety statistics always use evaluateCSSCHighPrecision.
%
% Main override fields:
%   profile          : 'smoke', 'pilot' (default), or 'formal'. These set
%                      1, 10, and 100 trials per scene group, respectively.
%   variantGroup     : 'core' (default) compares Init/Point-SDF/FD/SAG;
%                      'stability' removes one robustness mechanism from
%                      CSSC-SAG at a time; 'all' runs both groups.
%   sceneId          : '', 'double_slit', 's_channel',
%                      'staggered_baffles', or 'random_mixed'. Empty means
%                      all four scene families.
%   difficultyId     : '', 'easy', 'normal', or 'hard'. Empty means all
%                      three difficulty levels.
%   numTrials        : positive integer overriding the profile trial count.
%   maxPlanningTimeSec: common end-to-end initialization plus optimization
%                      budget assigned to every variant (default: 10 s).
%   runName          : fixed result-folder name. Set this together with
%                      resume=true to continue an interrupted experiment.
%   resume           : reuse a matching checkpoint (default: true).
%   saveCaseResults  : save one stable result.mat per case (default: true).
%   printEachRun     : print one concise line per completed case.
%
% Shared geometry and environment parameters (L, dMin, dPref, scene
% difficulty, RRT settings) come from getCSSCDemoConfig2D. When overriding
% dMin here, also override dPref and csscOptimizationClearanceBuffer when
% needed to keep their intended relationship. Optimizer weights, nU,
% iteration limit, activeTopK, and stopping tolerances are centralized in
% makeParams below so every ablation starts from the same base settings.
%
% Example with explicit controls:
%   opts = struct('profile','pilot', 'variantGroup','stability', ...
%       'sceneId','s_channel', 'difficultyId','hard', ...
%       'numTrials',20, 'maxPlanningTimeSec',10, ...
%       'runName','run_simu2_schannel_hard', 'resume',true);
%   output = simu2_cssc_ablation(opts);

    if nargin < 1 || isempty(overrides)
        overrides = struct();
    end
    try
        projectRoot = initCSSCProjectPath;
    catch
        projectRoot = fileparts(fileparts(mfilename('fullpath')));
        addpath(genpath(fullfile(projectRoot, 'src')));
        addpath(fullfile(projectRoot, 'demos'));
    end

    cfg = makeConfig(projectRoot, overrides);
    variants = makeCSSCAblationVariants2D(cfg.variantGroup);
    specs = makeCSSCAblationSceneSpecs2D(cfg);
    [paramsEval, paramsOpt] = makeParams(cfg);
    prepareFolders(cfg);

    totalCases = numel(specs) * cfg.numTrials * numel(variants);
    definition = struct('schemaVersion', 'simu2-ablation-v1', ...
        'cfg', cfgForDefinition(cfg), 'specs', specs, ...
        'variants', variants, 'paramsEval', paramsEval, ...
        'paramsOpt', paramsOpt);
    [rows, completed, elapsedPrevious] = ...
        initializeCheckpoint(cfg, totalCases, definition);

    fprintf('\n[SIMU2 CSSC ablation]\n');
    fprintf('  profile      : %s\n', cfg.profile);
    fprintf('  variant group: %s (%d variants)\n', ...
        cfg.variantGroup, numel(variants));
    fprintf('  scene groups : %d\n', numel(specs));
    fprintf('  trials/group : %d\n', cfg.numTrials);
    fprintf('  total cases  : %d\n', totalCases);
    fprintf('  output dir   : %s\n\n', cfg.outDir);

    caseId = 0;
    tBatch = tic;
    for iscene = 1:numel(specs)
        spec = specs(iscene);
        fprintf('[scene %d/%d] %s | %s\n', iscene, numel(specs), ...
            char(spec.name), char(spec.difficultyName));

        for itrial = 1:cfg.numTrials
            caseIds = caseId + (1:numel(variants));
            if all(completed(caseIds))
                caseId = caseId + numel(variants);
                continue;
            end

            envSeed = spec.envSeeds(itrial);
            plannerSeed = cfg.plannerSeedBase + ...
                1000 * spec.familyIndex + itrial;
            [obstacles, envInfo] = generateCSSCBenchmarkScene2D( ...
                spec, cfg, envSeed);
            plannerOpts = makePlannerOpts(cfg, envInfo, plannerSeed);
            initOpts = makeInitOpts(cfg, plannerSeed);
            bundle = prepareCSSCInitialBundle2D( ...
                envInfo, obstacles, plannerOpts, paramsOpt, initOpts);

            [selectedMetrics, selectedEvalTime] = ...
                evaluateCandidate(bundle.selected, obstacles, paramsEval);
            [firstMetrics, firstEvalTime] = ...
                evaluateCandidate(bundle.firstAttempt, obstacles, paramsEval);

            for ivariant = 1:numel(variants)
                caseId = caseId + 1;
                if completed(caseId)
                    continue;
                end
                variant = variants(ivariant);
                [candidate, initialMetrics, initialEvalTime, initTime] = ...
                    selectVariantInitialization(variant, bundle, ...
                    selectedMetrics, selectedEvalTime, ...
                    firstMetrics, firstEvalTime);

                row = makeBaseRow(caseId, itrial, spec, variant, ...
                    envSeed, plannerSeed, numel(obstacles));
                tCase = tic;
                try
                    runOpts = struct( ...
                        'finalDMin', cfg.dMin, ...
                        'optimizationBudgetSec', max(0, ...
                        cfg.maxPlanningTimeSec - initTime));
                    runOut = runCSSCAblationVariant2D(variant, ...
                        candidate, obstacles, paramsOpt, paramsEval, ...
                        runOpts, initialMetrics);
                    row = fillRow(row, runOut, bundle, initTime, ...
                        initialEvalTime, cfg);
                    if cfg.saveCaseResults
                        row.resultMatPath = saveCase(cfg, row, spec, ...
                            variant, obstacles, bundle, runOut, ...
                            paramsEval, paramsOpt, envSeed, plannerSeed);
                    end
                catch ME
                    row.stopReason = "exception";
                    row.errorIdentifier = string(ME.identifier);
                    row.errorMessage = string(ME.message);
                    row.sharedInitTimeSec = initTime;
                    row.planTimeSec = initTime;
                end
                row.totalCaseTimeSec = toc(tCase);
                rows(caseId) = row;
                completed(caseId) = true;
                saveCheckpoint(cfg, rows, completed, definition, ...
                    elapsedPrevious + toc(tBatch));

                if cfg.printEachRun
                    fprintf('  [%d/%d] %-18s success=%d c=%.4g t=%.3fs %s\n', ...
                        caseId, totalCases, variant.label, ...
                        row.wholeBodySuccess, row.finalMinClear, ...
                        row.planTimeSec, char(row.stopReason));
                end
            end
        end
    end

    elapsedTotalSec = elapsedPrevious + toc(tBatch);
    if ~all(completed)
        error('SIMU2 ended with %d incomplete cases.', nnz(~completed));
    end

    trialTable = struct2table(rows);
    summaryTable = summarizeResults(trialTable);
    pairedTable = summarizePairedResults(trialTable);
    writetable(trialTable, fullfile(cfg.outDir, 'trial_results.csv'));
    writetable(summaryTable, fullfile(cfg.outDir, ...
        'summary_by_scene_difficulty_variant.csv'));
    writetable(pairedTable, fullfile(cfg.outDir, 'paired_comparison.csv'));
    save(fullfile(cfg.outDir, 'simu2_ablation_results.mat'), ...
        'cfg', 'specs', 'variants', 'paramsEval', 'paramsOpt', ...
        'rows', 'trialTable', 'summaryTable', 'pairedTable', ...
        'elapsedTotalSec', 'definition', '-v7.3');
    plotResults(trialTable, summaryTable, variants, cfg);

    fprintf('\n[SIMU2 summary]\n');
    disp(summaryTable);
    fprintf('[elapsed] %.2f min\n', elapsedTotalSec / 60);
    fprintf('[saved] %s\n', cfg.outDir);

    output = struct('outDir', cfg.outDir, 'cfg', cfg, ...
        'trialTable', trialTable, 'summaryTable', summaryTable, ...
        'pairedTable', pairedTable, 'elapsedTotalSec', elapsedTotalSec);
end

function cfg = makeConfig(projectRoot, overrides)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.profile = "pilot";
    cfg.variantGroup = "core";
    cfg.sceneId = "";
    cfg.difficultyId = "";
    cfg.numTrials = 10;
    cfg.saveCaseResults = true;
    cfg.resume = true;
    cfg.printEachRun = false;

    if isfield(overrides, 'profile')
        cfg.profile = lower(string(overrides.profile));
    end
    switch cfg.profile
        case "smoke"
            cfg.numTrials = 1;
            cfg.sceneId = "double_slit";
            cfg.difficultyId = "easy";
        case "pilot"
            cfg.numTrials = 10;
        case "formal"
            cfg.numTrials = 100;
        otherwise
            error('Unknown simu2 profile: %s', cfg.profile);
    end

    names = fieldnames(overrides);
    for i = 1:numel(names)
        cfg.(names{i}) = overrides.(names{i});
    end
    cfg.profile = lower(string(cfg.profile));
    cfg.variantGroup = lower(string(cfg.variantGroup));
    cfg.sceneId = string(cfg.sceneId);
    cfg.difficultyId = string(cfg.difficultyId);
    cfg.numTrials = double(cfg.numTrials);
    if cfg.numTrials < 1 || cfg.numTrials ~= round(cfg.numTrials)
        error('numTrials must be a positive integer.');
    end
    if strlength(cfg.sceneId) > 0 && ~any(cfg.sceneIds == cfg.sceneId)
        error('Unknown sceneId: %s', cfg.sceneId);
    end
    if strlength(cfg.difficultyId) > 0 && ...
            ~any(cfg.difficultyIds == cfg.difficultyId)
        error('Unknown difficultyId: %s', cfg.difficultyId);
    end

    if isfield(overrides, 'runName') && strlength(string(overrides.runName)) > 0
        cfg.runName = char(string(overrides.runName));
    else
        cfg.runName = sprintf('run_simu2_cssc_ablation_%s', ...
            datestr(now, 'yyyymmdd_HHMMSS'));
    end
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.caseDir = fullfile(cfg.outDir, 'cases');
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.checkpointPath = fullfile(cfg.outDir, 'batch_checkpoint.mat');
    cfg.inflateRadius = cfg.dMin;
end

function prepareFolders(cfg)
    folders = {cfg.outDir, cfg.caseDir, cfg.figDir};
    for i = 1:numel(folders)
        if ~exist(folders{i}, 'dir')
            mkdir(folders{i});
        end
    end
end

function [paramsEval, paramsOpt] = makeParams(cfg)
    paramsEval = struct('L', cfg.L, 'dMin', cfg.dMin, ...
        'dPref', cfg.dPref, 'degree', cfg.degree, ...
        'envOpts', struct('uRange', [0,1], 'vSearchRange', [0,1], ...
        'nU', 240, 'epsV', 1e-6, 'tolDen', 1e-6), ...
        'printEvalTiming', false, 'enablePathSample', true, ...
        'pathSampleN', 900, 'enablePointClearance', false, ...
        'enableObstacleMetadata', false, ...
        'pointClearanceResolution', cfg.pointClearanceResolution);

    paramsOpt = struct();
    paramsOpt.L = cfg.L;
    paramsOpt.dMin = cfg.dMin + cfg.csscOptimizationClearanceBuffer;
    paramsOpt.dPref = cfg.dPref;
    paramsOpt.degree = cfg.degree;
    paramsOpt.envOpts = struct('uRange', [0,1], ...
        'vSearchRange', [0,1], 'nU', 160, ...
        'epsV', 1e-6, 'tolDen', 1e-6);
    paramsOpt.wObs = 60000;
    paramsOpt.wClear = 100;
    paramsOpt.wRef = 0.01;
    paramsOpt.wSmooth = 0.5;
    paramsOpt.wLength = 0.002;
    paramsOpt.wTrust = 0;
    paramsOpt.solver = struct('gradMode', 'semi-analytic', ...
        'objectiveMode', 'cssc-chord');
    paramsOpt.returnPolicy = 'best-safe';
    paramsOpt.numIter = 100;
    paramsOpt.lr = 0.0015;
    paramsOpt.fdStep = 1e-5;
    paramsOpt.gradClip = 5;
    paramsOpt.printInterval = 100;
    paramsOpt.saveInterval = 100;
    paramsOpt.activeTopK = 30;
    paramsOpt.activeClearanceMargin = 0.008;
    paramsOpt.activeMode = 'topk';
    paramsOpt.stop = struct('enable', true, 'minIter', 10, ...
        'window', 8, 'tolRelJ', 1e-2, 'tolGrad', 1e-1, ...
        'tolStep', 1e-3, 'patience', 10, 'tolBestRel', 1e-4, ...
        'requireSafe', true, 'clearanceMargin', 0, ...
        'maxTimeSec', cfg.maxPlanningTimeSec);
    paramsOpt.printEvalTiming = false;
    paramsOpt.enableTimingDebug = false;
    paramsOpt.enablePathSample = false;
    paramsOpt.enablePointClearance = false;
    paramsOpt.enableObstacleMetadata = false;

    if cfg.profile == "smoke"
        paramsOpt.numIter = 3;
    end
end

function opts = makePlannerOpts(cfg, envInfo, seed)
    opts = struct('bounds', envInfo.bounds, 'stepSize', cfg.stepSize, ...
        'goalBias', cfg.goalBias, 'goalTol', cfg.goalTol, ...
        'maxIter', cfg.maxIter, 'maxTimeSec', cfg.maxPlanningTimeSec, ...
        'collisionResolution', cfg.collisionResolution, ...
        'inflateRadius', cfg.inflateRadius, 'seed', seed);
end

function opts = makeInitOpts(cfg, plannerSeed)
    opts = struct('maxCandidates', cfg.csscInitMaxCandidates, ...
        'maxTimeSec', cfg.maxPlanningTimeSec * cfg.csscInitTimeFraction, ...
        'plannerSeed', plannerSeed, ...
        'seedStride', cfg.csscInitSeedStride, ...
        'shortcutSeedBase', cfg.shortcutSeedBase, ...
        'numShortcut', cfg.numShortcut, ...
        'acceptMargin', cfg.csscInitAcceptMargin, ...
        'frontendMethod', cfg.csscFrontendMethod);
end

function [metrics, elapsed] = evaluateCandidate(candidate, obstacles, params)
    t = tic;
    if isstruct(candidate) && isfield(candidate, 'valid') && candidate.valid
        params.knot = candidate.splineInfo.knot;
        metrics = evaluateCSSCHighPrecision(candidate.Pinit, obstacles, params);
    else
        metrics = evaluateCSSCHighPrecision([], obstacles, params);
    end
    elapsed = toc(t);
end

function [candidate, metrics, evalTime, initTime] = ...
        selectVariantInitialization(variant, bundle, selectedMetrics, ...
        selectedEvalTime, firstMetrics, firstEvalTime)
    if strcmpi(variant.initPolicy, 'first-attempt')
        candidate = bundle.firstAttempt;
        metrics = firstMetrics;
        evalTime = firstEvalTime;
        initTime = candidate.cumulativeTimeSec;
    else
        candidate = bundle.selected;
        metrics = selectedMetrics;
        evalTime = selectedEvalTime;
        initTime = bundle.elapsedTimeSec;
    end
end

function row = makeBaseRow(caseId, trial, spec, variant, ...
        envSeed, plannerSeed, numObstacles)
    row = emptyRow();
    row.caseId = caseId;
    row.trialIndex = trial;
    row.sceneId = string(spec.id);
    row.sceneName = string(spec.name);
    row.difficultyIndex = spec.difficultyIndex;
    row.difficultyId = string(spec.difficultyId);
    row.difficultyName = string(spec.difficultyName);
    row.difficultyValue = spec.difficultyValue;
    row.variantId = string(variant.id);
    row.variantLabel = string(variant.label);
    row.variantGroup = string(variant.group);
    row.envSeed = envSeed;
    row.plannerSeed = plannerSeed;
    row.numObstacles = numObstacles;
end

function row = fillRow(row, out, bundle, initTime, initialEvalTime, cfg)
    initial = out.initialMetrics;
    final = out.finalMetrics;
    info = out.optimizerInfo;
    row.initializationSuccess = out.candidate.valid;
    row.initialCandidateCount = bundle.numValidCandidates;
    row.selectedInitialAttempt = out.candidate.attempt;
    row.initialMinClear = metric(initial, 'minClear', nan);
    row.initialClearMargin = row.initialMinClear - cfg.dMin;
    row.initialDMinSatisfied = metric(initial, 'dMinSatisfied', false);
    row.finalMinClear = metric(final, 'minClear', nan);
    row.finalClearMargin = row.finalMinClear - cfg.dMin;
    row.clearanceGain = row.finalMinClear - row.initialMinClear;
    row.envelopeSuccess = metric(final, 'success', false);
    row.dMinSatisfied = metric(final, 'dMinSatisfied', false);
    row.ftlGeometricRealizable = metric( ...
        final, 'ftlGeometricRealizable', false);
    row.wholeBodySuccess = out.success;
    row.recoveredUnsafeInitial = ~row.initialDMinSatisfied && ...
        row.wholeBodySuccess;
    row.initialPathLength = metric(initial, 'pathLength', nan);
    row.finalPathLength = metric(final, 'pathLength', nan);
    row.pathLengthChange = row.finalPathLength - row.initialPathLength;
    row.finalTurnAbsSum = metric(final, 'turnAbsSum', nan);
    row.finalMeanAbsTurn = metric(final, 'meanAbsTurn', nan);
    row.sharedInitTimeSec = initTime;
    row.optimizationTimeSec = out.optimizationTimeSec;
    row.planTimeSec = initTime + out.optimizationTimeSec;
    row.initialEvalTimeSec = initialEvalTime;
    row.finalEvalTimeSec = out.finalEvaluationTimeSec;
    row.numIter = getField(info, 'numIterActual', 0);
    row.objectiveCallCount = objectiveCallCount(info);
    row.bestIter = getField(info, 'bestIter', nan);
    row.bestSafeIter = getField(info, 'bestSafeIter', nan);
    row.returnedSafe = logical(getField(info, 'returnedSafe', false));
    row.stopReason = string(out.stopReason);
end

function value = objectiveCallCount(info)
    value = nan;
    if isstruct(info) && isfield(info, 'timing') && ...
            isfield(info.timing, 'objCalls')
        value = sum(info.timing.objCalls, 'omitnan');
    end
end

function value = metric(s, name, defaultValue)
    value = getField(s, name, defaultValue);
end

function value = getField(s, name, defaultValue)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = defaultValue;
    end
end

function filePath = saveCase(cfg, row, spec, variant, obstacles, ...
        bundle, out, paramsEval, paramsOpt, envSeed, plannerSeed)
    caseConfig = struct('experiment', 'simu2_cssc_ablation', ...
        'profile', cfg.profile, 'sceneSpec', spec, ...
        'variant', variant, 'dMin', cfg.dMin, ...
        'timeBudgetSec', cfg.maxPlanningTimeSec);
    seeds = struct('environment', envSeed, 'planner', plannerSeed, ...
        'selectedCandidate', out.candidate.seed);
    optimizerInfo = out.optimizerInfo;
    optimizerInfo.initializationBundle = bundle;
    timing = struct('sharedInitializationSec', row.sharedInitTimeSec, ...
        'optimizationSec', row.optimizationTimeSec, ...
        'planTimeSec', row.planTimeSec, ...
        'initialEvaluationSec', row.initialEvalTimeSec, ...
        'finalEvaluationSec', row.finalEvalTimeSec);
    flags = struct('plannerSuccess', row.initializationSuccess, ...
        'highPrecisionSuccess', row.envelopeSuccess, ...
        'dMinSatisfied', row.dMinSatisfied, ...
        'wholeBodySuccess', row.wholeBodySuccess, ...
        'ftlGeometricRealizable', row.ftlGeometricRealizable);
    stages = struct('initial', out.initialMetrics, 'final', out.finalMetrics);
    parameters = struct('evaluation', paramsEval, ...
        'optimizationBase', paramsOpt, 'optimizationUsed', out.paramsUsed);
    result = makeCSSCExperimentResult(caseConfig, seeds, obstacles, ...
        out.paths, optimizerInfo, out.finalMetrics, timing, flags, ...
        stages, parameters);
    result.ablation = struct('variant', variant, 'summaryRow', row);

    fileName = sprintf('%s_%s_trial_%03d_%s_result.mat', ...
        char(row.sceneId), char(row.difficultyId), row.trialIndex, ...
        char(row.variantId));
    caseDir = fullfile(cfg.caseDir, erase(fileName, '_result.mat'));
    if ~exist(caseDir, 'dir'); mkdir(caseDir); end
    filePath = string(fullfile(caseDir, 'result.mat'));
    save(char(filePath), 'result', '-v7.3');
end

function row = emptyRow()
    row = struct();
    row.caseId = 0;
    row.trialIndex = 0;
    row.sceneId = "";
    row.sceneName = "";
    row.difficultyIndex = 0;
    row.difficultyId = "";
    row.difficultyName = "";
    row.difficultyValue = nan;
    row.variantId = "";
    row.variantLabel = "";
    row.variantGroup = "";
    row.envSeed = 0;
    row.plannerSeed = 0;
    row.numObstacles = 0;
    row.initializationSuccess = false;
    row.initialCandidateCount = 0;
    row.selectedInitialAttempt = nan;
    row.initialMinClear = nan;
    row.initialClearMargin = nan;
    row.initialDMinSatisfied = false;
    row.finalMinClear = nan;
    row.finalClearMargin = nan;
    row.clearanceGain = nan;
    row.envelopeSuccess = false;
    row.dMinSatisfied = false;
    row.ftlGeometricRealizable = false;
    row.wholeBodySuccess = false;
    row.recoveredUnsafeInitial = false;
    row.initialPathLength = nan;
    row.finalPathLength = nan;
    row.pathLengthChange = nan;
    row.finalTurnAbsSum = nan;
    row.finalMeanAbsTurn = nan;
    row.sharedInitTimeSec = nan;
    row.optimizationTimeSec = nan;
    row.planTimeSec = nan;
    row.initialEvalTimeSec = nan;
    row.finalEvalTimeSec = nan;
    row.totalCaseTimeSec = nan;
    row.numIter = nan;
    row.objectiveCallCount = nan;
    row.bestIter = nan;
    row.bestSafeIter = nan;
    row.returnedSafe = false;
    row.stopReason = "";
    row.errorIdentifier = "";
    row.errorMessage = "";
    row.resultMatPath = "";
end

function summary = summarizeResults(T)
    sceneIds = unique(T.sceneId, 'stable');
    difficultyIds = unique(T.difficultyId, 'stable');
    variantIds = unique(T.variantId, 'stable');
    rows = repmat(emptySummaryRow(), ...
        numel(sceneIds)*numel(difficultyIds)*numel(variantIds), 1);
    k = 0;
    for i = 1:numel(sceneIds)
        for d = 1:numel(difficultyIds)
            for v = 1:numel(variantIds)
                mask = T.sceneId == sceneIds(i) & ...
                    T.difficultyId == difficultyIds(d) & ...
                    T.variantId == variantIds(v);
                if ~any(mask); continue; end
                R = T(mask,:);
                k = k + 1;
                rows(k).sceneId = sceneIds(i);
                rows(k).difficultyId = difficultyIds(d);
                rows(k).variantId = variantIds(v);
                rows(k).variantLabel = R.variantLabel(1);
                rows(k).numTrials = height(R);
                rows(k).numFinite = nnz(isfinite(R.finalMinClear));
                rows(k).wholeBodySuccessRate = mean(R.wholeBodySuccess);
                rows(k).initialSuccessRate = mean(R.initialDMinSatisfied);
                rows(k).recoveryRateAll = mean(R.recoveredUnsafeInitial);
                unsafe = ~R.initialDMinSatisfied & R.initializationSuccess;
                if any(unsafe)
                    rows(k).conditionalRecoveryRate = ...
                        mean(R.wholeBodySuccess(unsafe));
                end
                rows(k).finalMinClearMean = finiteMean(R.finalMinClear);
                rows(k).finalClearMarginMean = finiteMean(R.finalClearMargin);
                rows(k).clearanceGainMean = finiteMean(R.clearanceGain);
                rows(k).pathLengthMean = finiteMean(R.finalPathLength);
                rows(k).planTimeMeanSec = finiteMean(R.planTimeSec);
                rows(k).optimizationTimeMeanSec = ...
                    finiteMean(R.optimizationTimeSec);
                rows(k).numIterMean = finiteMean(R.numIter);
                rows(k).objectiveCallMean = finiteMean(R.objectiveCallCount);
            end
        end
    end
    summary = struct2table(rows(1:k));
end

function row = emptySummaryRow()
    row = struct('sceneId', "", 'difficultyId', "", ...
        'variantId', "", 'variantLabel', "", 'numTrials', 0, ...
        'numFinite', 0, 'wholeBodySuccessRate', nan, ...
        'initialSuccessRate', nan, 'recoveryRateAll', nan, ...
        'conditionalRecoveryRate', nan, 'finalMinClearMean', nan, ...
        'finalClearMarginMean', nan, 'clearanceGainMean', nan, ...
        'pathLengthMean', nan, 'planTimeMeanSec', nan, ...
        'optimizationTimeMeanSec', nan, 'numIterMean', nan, ...
        'objectiveCallMean', nan);
end

function paired = summarizePairedResults(T)
    reference = T(T.variantId == "cssc_sag",:);
    variantIds = unique(T.variantId, 'stable');
    rows = repmat(struct('variantId', "", 'variantLabel', "", ...
        'numPairs', 0, 'successRate', nan, 'referenceSuccessRate', nan, ...
        'successRateDifference', nan, 'referenceOnlySuccess', 0, ...
        'variantOnlySuccess', 0, 'clearanceDifferenceMean', nan, ...
        'timeDifferenceMeanSec', nan), numel(variantIds), 1);
    k = 0;
    for i = 1:numel(variantIds)
        if variantIds(i) == "cssc_sag"; continue; end
        V = T(T.variantId == variantIds(i),:);
        if height(V) ~= height(reference); continue; end
        k = k + 1;
        rows(k).variantId = variantIds(i);
        rows(k).variantLabel = V.variantLabel(1);
        rows(k).numPairs = height(V);
        rows(k).successRate = mean(V.wholeBodySuccess);
        rows(k).referenceSuccessRate = mean(reference.wholeBodySuccess);
        rows(k).successRateDifference = rows(k).successRate - ...
            rows(k).referenceSuccessRate;
        rows(k).referenceOnlySuccess = nnz( ...
            reference.wholeBodySuccess & ~V.wholeBodySuccess);
        rows(k).variantOnlySuccess = nnz( ...
            V.wholeBodySuccess & ~reference.wholeBodySuccess);
        rows(k).clearanceDifferenceMean = finiteMean( ...
            V.finalMinClear - reference.finalMinClear);
        rows(k).timeDifferenceMeanSec = finiteMean( ...
            V.planTimeSec - reference.planTimeSec);
    end
    paired = struct2table(rows(1:k));
end

function value = finiteMean(x)
    x = x(isfinite(x));
    if isempty(x); value = nan; else; value = mean(x); end
end

function plotResults(T, S, variants, cfg)
    labels = string({variants.label});
    colors = lines(numel(labels));

    fig = figure('Color','w','Position',[100 100 900 430]);
    rates = nan(numel(labels),1);
    for i = 1:numel(labels)
        rates(i) = mean(T.wholeBodySuccess(T.variantLabel == labels(i)));
    end
    b = bar(rates, 'FaceColor','flat');
    b.CData = colors;
    set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, ...
        'TickLabelInterpreter','none');
    ylabel('Whole-body success rate'); ylim([0 1]); grid on;
    title('CSSC ablation: success rate');
    exportgraphics(fig, fullfile(cfg.figDir, 'success_rate.jpg'), ...
        'Resolution', cfg.figureResolution);
    close(fig);

    plotBox(T, labels, 'clearanceGain', 'Clearance gain', ...
        'clearance_gain_boxplot.jpg', cfg);
    plotBox(T, labels, 'planTimeSec', 'Planning time (s)', ...
        'planning_time_boxplot.jpg', cfg);
    plotBox(T, labels, 'finalClearMargin', 'Final minClear - dMin', ...
        'clearance_margin_boxplot.jpg', cfg);

    fig = figure('Color','w','Position',[100 100 1050 480]);
    sceneIds = unique(S.sceneId, 'stable');
    tiledlayout(1, numel(sceneIds), 'TileSpacing','compact', ...
        'Padding','compact');
    for s = 1:numel(sceneIds)
        nexttile;
        data = nan(3, numel(labels));
        for d = 1:3
            for v = 1:numel(labels)
                mask = S.sceneId == sceneIds(s) & ...
                    S.difficultyId == cfg.difficultyIds(d) & ...
                    S.variantLabel == labels(v);
                if any(mask)
                    data(d,v) = S.wholeBodySuccessRate(find(mask,1));
                end
            end
        end
        bar(data); ylim([0 1]); grid on;
        title(sceneIds(s), 'Interpreter','none');
        set(gca, 'XTick', 1:3, 'XTickLabel', cfg.difficultyNames);
        if s == 1; ylabel('Success rate'); end
        if s == numel(sceneIds)
            legend(labels, 'Location','bestoutside', 'Interpreter','none');
        end
    end
    exportgraphics(fig, fullfile(cfg.figDir, ...
        'success_rate_by_difficulty.jpg'), ...
        'Resolution', cfg.figureResolution);
    close(fig);
end

function plotBox(T, labels, fieldName, yLabelText, fileName, cfg)
    fig = figure('Color','w','Position',[100 100 900 430]);
    values = T.(fieldName);
    valid = isfinite(values);
    if any(valid)
        groups = categorical(T.variantLabel(valid), labels, labels, ...
            'Ordinal', true);
        boxchart(groups, values(valid));
        ylabel(yLabelText); grid on;
    else
        axis off;
        text(0.5,0.5,'No finite data','HorizontalAlignment','center');
    end
    exportgraphics(fig, fullfile(cfg.figDir, fileName), ...
        'Resolution', cfg.figureResolution);
    close(fig);
end

function [rows, completed, elapsed] = initializeCheckpoint( ...
        cfg, totalCases, definition)
    rows = repmat(emptyRow(), totalCases, 1);
    completed = false(totalCases,1);
    elapsed = 0;
    if ~cfg.resume || ~exist(cfg.checkpointPath, 'file')
        return;
    end
    loaded = load(cfg.checkpointPath, 'checkpoint');
    checkpoint = loaded.checkpoint;
    if ~isequaln(checkpoint.definition, definition)
        error('Existing simu2 checkpoint does not match this run definition.');
    end
    rows = checkpoint.rows;
    completed = checkpoint.completed;
    elapsed = checkpoint.elapsedSec;
end

function saveCheckpoint(cfg, rows, completed, definition, elapsedSec)
    checkpoint = struct('schemaVersion', 'simu2-ablation-v1', ...
        'savedAt', datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF'), ...
        'rows', rows, 'completed', completed, ...
        'definition', definition, 'elapsedSec', elapsedSec);
    tmpPath = [tempname(cfg.outDir), '.mat'];
    save(tmpPath, 'checkpoint', '-v7.3');
    movefile(tmpPath, cfg.checkpointPath, 'f');
end

function value = cfgForDefinition(cfg)
    value = cfg;
    fields = {'resume','printEachRun'};
    for i = 1:numel(fields)
        if isfield(value, fields{i}); value = rmfield(value, fields{i}); end
    end
end
