if ~exist('simu1Overrides', 'var') || ~isstruct(simu1Overrides)
    if exist('demo2Overrides', 'var') && isstruct(demo2Overrides)
        simu1Overrides = demo2Overrides;
    else
        simu1Overrides = struct();
    end
end
clearvars -except simu1Overrides;
clc; close all;

%% Baseline comparison on the four common CSSC benchmark scenes
% Compares four baselines (RRT, RRT*, RRTSC-2D, and Sp-RRT-2D) with the
% proposed CSSC method on the same double-slit, S-channel,
% staggered-baffle, and random-mixed instances. CSSC uses RRT as its
% frontend in this experiment. All methods share each trial's environment,
% planner seed, dMin-inflated planning geometry, and planning-time budget.
% Final cubic-path metrics use evaluateCSSCHighPrecision. Native polyline
% outputs use evaluatePolylineHighPrecision2D, whose fixed-chord endpoints
% are solved analytically per segment without degree-1 Newton iterations.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeBaselineConfig(projectRoot, simu1Overrides);
sceneSpecs = makeBaselineSceneSpecs(cfg);
sceneSpecs = selectBaselineSceneSpecs(sceneSpecs, cfg);
cfg.numRunSceneGroups = numel(sceneSpecs);
cfg.numRunSceneFamilies = numel(unique([sceneSpecs.familyIndex]));
cfg.numRunDifficultyLevels = ...
    numel(unique([sceneSpecs.difficultyIndex]));
paramsEval = makeEnvelopeEvalParams(cfg);
paramsOpt = makeCSSCOptimizationParams(cfg);
if cfg.smokeTest
    sceneSpecs = sceneSpecs(1);
    paramsOpt.numIter = 3;
end

if cfg.saveResults && ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end
if cfg.saveResults && ~exist(cfg.figDir, 'dir')
    mkdir(cfg.figDir);
end
if cfg.saveResults && cfg.saveCaseResults && ~exist(cfg.caseDir, 'dir')
    mkdir(cfg.caseDir);
end

plannerNames = cfg.plannerNames;
totalRuns = numel(sceneSpecs) * cfg.numSeedsPerScene * numel(plannerNames);
provenance = prepareExperimentProvenance(cfg);
cfg.provenance = provenance;
runDefinition = makeRunDefinition( ...
    cfg, sceneSpecs, paramsEval, paramsOpt);
[results, completedMask, resumeInfo] = initializeBatchState( ...
    cfg, totalRuns, runDefinition);
if cfg.saveResults && ~resumeInfo.resumed
    saveBatchCheckpointAtomic(cfg, results, completedMask, ...
        runDefinition, provenance, 0, 0);
end

fprintf('\n[RRT vs RRT* vs RRTSC-2D vs Sp-RRT-2D vs CSSC]\n');
fprintf('  output dir : %s\n', cfg.outDir);
fprintf('  scene groups: %d (%d families x %d difficulty levels)\n', ...
    cfg.numRunSceneGroups, cfg.numRunSceneFamilies, ...
    cfg.numRunDifficultyLevels);
fprintf('  seeds/scene: %d\n', cfg.numSeedsPerScene);
fprintf('  planning centerline margin: %.4f (= dMin)\n', ...
    cfg.inflateRadius);
fprintf('  total runs : %d\n', totalRuns);
fprintf('  resumed    : %d (%d completed cases)\n\n', ...
    resumeInfo.resumed, nnz(completedMask));

caseId = 0;
tAll = tic;

for iscene = 1:numel(sceneSpecs)
    spec = sceneSpecs(iscene);
    fprintf('\n[scene %d/%d] %s | %s\n', iscene, numel(sceneSpecs), ...
        char(spec.name), char(spec.difficultyName));

    for iseed = 1:cfg.numSeedsPerScene
        envSeed = spec.envSeeds(iseed);
        [obstacles, envInfo] = ...
            generateCSSCBenchmarkScene2D(spec, cfg, envSeed);
        envInfo.obstacles = obstacles;
        % Pair methods and difficulty levels with the same planner seed.
        plannerSeed = cfg.plannerSeedBase + ...
            1000 * spec.familyIndex + iseed;

        for ialg = 1:numel(plannerNames)
            caseId = caseId + 1;
            plannerName = plannerNames{ialg};
            if completedMask(caseId)
                if cfg.printEachRun
                    fprintf('[resume skip %d/%d] %s\n', ...
                        caseId, totalRuns, plannerName);
                end
                continue;
            end

            result = emptyBaselineResult();
            result.caseId = caseId;
            result.trialIndex = iseed;
            result.sceneId = string(spec.id);
            result.sceneName = string(spec.name);
            result.sceneType = string(spec.type);
            result.difficultyIndex = spec.difficultyIndex;
            result.difficultyId = string(spec.difficultyId);
            result.difficultyName = string(spec.difficultyName);
            result.difficultyMetric = string(spec.difficultyMetric);
            result.difficultyValue = spec.difficultyValue;
            result.envSeed = envSeed;
            result.planner = string(plannerName);
            result.plannerSeed = plannerSeed;
            result.numObstacles = numel(obstacles);

            tRun = tic;
            caseData = emptyCaseData();
            try
                [result, caseData] = runOneMethod( ...
                    result, plannerName, envInfo, obstacles, ...
                    cfg, paramsEval, paramsOpt, plannerSeed);
            catch ME
                result.planTimeSec = toc(tRun);
                result.stopReason = "exception";
                result.plannerMessage = "Method execution raised an exception.";
                result.errorIdentifier = string(ME.identifier);
                result.errorMessage = string(ME.message);
                result.errorReport = string(getReport( ...
                    ME, 'extended', 'hyperlinks', 'off'));
                caseData.optimizerInfo = struct( ...
                    'method', plannerName, ...
                    'exceptionIdentifier', char(result.errorIdentifier), ...
                    'exceptionMessage', char(result.errorMessage), ...
                    'exceptionReport', char(result.errorReport));
                tEvalError = tic;
                caseData.hpMetrics = evaluateCSSCHighPrecision( ...
                    [], obstacles, paramsEval);
                result.evalTimeSec = toc(tEvalError);
                result.totalEvaluationTimeSec = result.evalTimeSec;
                caseData.stageMetrics.final = caseData.hpMetrics;
                caseData.timing = struct( ...
                    'planTimeSec', result.planTimeSec, ...
                    'evalTimeSec', result.evalTimeSec);
            end

            result.wholeBodySuccess = result.envelopeDMinSatisfied && ...
                result.ftlGeometricRealizable;
            result = finalizeSolutionStatus(result);
            result.totalTimeSec = toc(tRun);

            if cfg.saveResults && cfg.saveCaseResults
                flags = makeSuccessFlags(result);
                result.resultMatPath = saveCaseResult( ...
                    cfg, result, obstacles, caseData.paths, ...
                    caseData.optimizerInfo, caseData.hpMetrics, ...
                    caseData.timing, flags, caseData.stageMetrics, ...
                    caseData.parameters, paramsEval, paramsOpt, provenance);
            end

            results(caseId) = result;
            completedMask(caseId) = true;
            if cfg.saveResults
                saveBatchCheckpointAtomic(cfg, results, completedMask, ...
                    runDefinition, provenance, caseId, ...
                    resumeInfo.previousElapsedSec + toc(tAll));
            end

            if cfg.printEachRun
                printOneRun(result, caseId, totalRuns);
            end
        end
    end

    Tnow = struct2table(results(completedMask));
    Snow = summarizeBaselineResults(Tnow);
    disp(Snow);
end

elapsedTotalSec = resumeInfo.previousElapsedSec + toc(tAll);
if ~all(completedMask)
    error('Batch ended with %d incomplete cases.', nnz(~completedMask));
end
trialTable = struct2table(results);
summaryTable = summarizeBaselineResults(trialTable);

fprintf('\n[final RRT vs RRT* vs RRTSC-2D vs Sp-RRT-2D vs CSSC]\n');
disp(summaryTable);
fprintf('[elapsed] %.2f min\n', elapsedTotalSec / 60);

if cfg.saveResults
    writeTableAtomic(trialTable, ...
        fullfile(cfg.outDir, 'trial_results.csv'));
    writeTableAtomic(summaryTable, ...
        fullfile(cfg.outDir, 'summary_by_scene_difficulty_planner.csv'));
    writeTableAtomic(summaryTable, ...
        fullfile(cfg.outDir, 'summary_mean_table.csv'));
    batchMatPath = fullfile(cfg.outDir, ...
        'baseline_five_methods_results.mat');
    batchTmpPath = [tempname(cfg.outDir), '.mat'];
    save(batchTmpPath, ...
        'cfg', 'sceneSpecs', 'paramsEval', 'paramsOpt', 'results', ...
        'trialTable', 'summaryTable', 'elapsedTotalSec', ...
        'completedMask', 'runDefinition', 'provenance');
    replaceFileAtomic(batchTmpPath, batchMatPath);
    saveBatchCheckpointAtomic(cfg, results, completedMask, ...
        runDefinition, provenance, totalRuns, elapsedTotalSec);
    writeMetricDefinitions(cfg);
    plotBaselineFigures(trialTable, summaryTable, cfg);
    fprintf('[saved] %s\n', cfg.outDir);
end

%% Configuration

function cfg = makeBaselineConfig(projectRoot, overrides)
    % Shared geometry, scene difficulty, and planner budgets live here:
    % demos/getCSSCDemoConfig2D.m
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_' ...
        datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.caseDir = fullfile(cfg.outDir, 'cases');
    cfg.saveResults = true;
    cfg.saveCaseResults = true;  % Set false only when per-case result.mat files are unnecessary.
    cfg.resume = true;
    cfg.allowResumeProvenanceMismatch = false;
    cfg.printEachRun = false;
    cfg.plannerNames = { ...
        'RRT', 'RRT*', 'RRTSC-2D', 'Sp-RRT-2D', 'CSSC'};
    cfg.plannerColors = [
        0.20, 0.45, 0.75
        0.30, 0.65, 0.40
        0.85, 0.45, 0.18
        0.10, 0.62, 0.66
        0.55, 0.25, 0.65
    ];
    cfg.csscFrontendMethod = 'rrt';
    % All planners use the same geometric centerline clearance margin.
    % Final whole-body success is still evaluated independently on the
    % original obstacles by evaluateCSSCHighPrecision.
    cfg.inflateRadius = cfg.dMin;
    cfg.rrtscMaxTotalTime = cfg.maxPlanningTimeSec;
    cfg.sprrtMaxTimeSec = cfg.maxPlanningTimeSec;
    cfg.smokeTest = strcmpi(getenv('CSSC_BASELINE_SMOKE'), '1');
    cfg.selectedSceneId = "";
    cfg.selectedDifficultyId = "";
    cfg = applyBaselineOverrides(cfg, overrides);
    cfg.checkpointPath = fullfile(cfg.outDir, 'batch_checkpoint.mat');
    cfg.provenancePath = fullfile(cfg.outDir, 'experiment_provenance.mat');
    cfg.provenanceTextPath = fullfile( ...
        cfg.outDir, 'experiment_provenance.txt');

    if cfg.smokeTest
        cfg.numSeedsPerScene = 1;
        cfg.saveResults = false;
        cfg.saveCaseResults = false;
        cfg.resume = false;
        cfg.printEachRun = true;
        cfg.maxIter = 800;
        cfg.rrtscMaxAttempts = 3;
        cfg.maxPlanningTimeSec = 10.0;
        cfg.rrtscMaxTotalTime = cfg.maxPlanningTimeSec;
        cfg.sprrtMaxTimeSec = cfg.maxPlanningTimeSec;
    end
end

function cfg = applyBaselineOverrides(cfg, overrides)
    if nargin < 2 || isempty(overrides)
        return;
    end

    if isfield(overrides, 'sceneId') && strlength(string(overrides.sceneId)) > 0
        sceneId = string(overrides.sceneId);
        if ~any(cfg.sceneIds == sceneId)
            error('Unknown simu1 sceneId: %s', sceneId);
        end
        cfg.selectedSceneId = sceneId;
    end

    if isfield(overrides, 'difficultyId') && ...
            strlength(string(overrides.difficultyId)) > 0
        difficultyId = string(overrides.difficultyId);
        if ~any(cfg.difficultyIds == difficultyId)
            error('Unknown simu1 difficultyId: %s', difficultyId);
        end
        cfg.selectedDifficultyId = difficultyId;
    end

    if isfield(overrides, 'numTrials')
        numTrials = double(overrides.numTrials);
        if ~isscalar(numTrials) || ~isfinite(numTrials) || ...
                numTrials < 1 || numTrials ~= round(numTrials)
            error('simu1 numTrials must be a positive integer.');
        end
        cfg.numSeedsPerScene = numTrials;
    end

    if isfield(overrides, 'printEachRun')
        cfg.printEachRun = logical(overrides.printEachRun);
    end
    if isfield(overrides, 'saveCaseResults')
        cfg.saveCaseResults = logical(overrides.saveCaseResults);
    end
    if isfield(overrides, 'resume')
        cfg.resume = logical(overrides.resume);
    end
    if isfield(overrides, 'allowResumeProvenanceMismatch')
        cfg.allowResumeProvenanceMismatch = logical( ...
            overrides.allowResumeProvenanceMismatch);
    end

    isPilot = strlength(cfg.selectedSceneId) > 0 || ...
        strlength(cfg.selectedDifficultyId) > 0 || ...
        isfield(overrides, 'numTrials');
    if isfield(overrides, 'runName') && strlength(string(overrides.runName)) > 0
        cfg.runName = char(string(overrides.runName));
    elseif isPilot
        sceneTag = fallbackTag(cfg.selectedSceneId, "all-scenes");
        difficultyTag = fallbackTag( ...
            cfg.selectedDifficultyId, "all-difficulties");
        cfg.runName = sprintf('run_pilot_%s_%s_%s', ...
            char(sceneTag), char(difficultyTag), ...
            datestr(now, 'yyyymmdd_HHMMSS'));
    end

    cfg.outDir = fullfile(cfg.projectRoot, 'results', 'runs', cfg.runName);
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.caseDir = fullfile(cfg.outDir, 'cases');
end

function tag = fallbackTag(value, fallback)
    tag = string(value);
    if strlength(tag) == 0
        tag = string(fallback);
    end
    tag = regexprep(tag, '[^A-Za-z0-9_-]', '_');
end

function specs = makeBaselineSceneSpecs(cfg)
    baseSpecs = makeCSSCBenchmarkSceneSpecs2D( ...
        struct('bounds', cfg.bounds, 'maxEnvSeedTry', cfg.numSeedsPerScene));

    for family = 1:numel(baseSpecs)
        baseSpecs(family).familyIndex = family;
        baseSpecs(family).difficultyIndex = 0;
        baseSpecs(family).difficultyId = "";
        baseSpecs(family).difficultyName = "";
        baseSpecs(family).difficultyMetric = "";
        baseSpecs(family).difficultyValue = nan;
    end

    specs = repmat(baseSpecs(1), ...
        cfg.numSceneFamilies * cfg.numDifficultyLevels, 1);
    k = 0;
    for family = 1:cfg.numSceneFamilies
        for level = 1:cfg.numDifficultyLevels
            k = k + 1;
            spec = baseSpecs(family);
            spec.envSeeds = cfg.envSeedBaseByFamily(family) + ...
                (0:cfg.numSeedsPerScene-1);
            spec.difficultyIndex = level;
            spec.difficultyId = cfg.difficultyIds(level);
            spec.difficultyName = cfg.difficultyNames(level);

            switch family
                case 1
                    spec.gapHeight = cfg.doubleSlit.dGap(level);
                    spec.gapCenterRange = ...
                        cfg.doubleSlit.centerRange(level,:);
                    spec.difficultyMetric = "dGap";
                    spec.difficultyValue = spec.gapHeight;

                case 2
                    spec.dGap = cfg.sChannel.dGap(level);
                    spec.channelW = cfg.sChannel.W;
                    spec.channelH = diff(cfg.bounds(2,:));
                    spec.difficultyMetric = "dGap";
                    spec.difficultyValue = spec.dGap;

                case 3
                    spec.passageWidth = ...
                        cfg.baffles.passageWidth(level);
                    spec.difficultyMetric = "passageWidth";
                    spec.difficultyValue = spec.passageWidth;

                case 4
                    spec.nCircle = cfg.randomMixed.nCircle(level);
                    spec.nRect = cfg.randomMixed.nRect(level);
                    spec.minGap = cfg.randomMixed.minGap(level);
                    spec.difficultyMetric = "numObstacles";
                    spec.difficultyValue = spec.nCircle + spec.nRect;
            end
            specs(k) = spec;
        end
    end
end

function specs = selectBaselineSceneSpecs(specs, cfg)
    keep = true(size(specs));
    if strlength(cfg.selectedSceneId) > 0
        sceneIds = reshape(string({specs.id}), size(specs));
        keep = keep & sceneIds == cfg.selectedSceneId;
    end
    if strlength(cfg.selectedDifficultyId) > 0
        difficultyIds = reshape(string({specs.difficultyId}), size(specs));
        keep = keep & difficultyIds == ...
            cfg.selectedDifficultyId;
    end
    specs = specs(keep);
    if isempty(specs)
        error('No benchmark scene matches the requested simu1 selection.');
    end
end

function params = makeEnvelopeEvalParams(cfg)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = 240;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;

    params.printEvalTiming = false;
    params.enablePathSample = true;
    params.pathSampleN = 900;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

function params = makeCSSCOptimizationParams(cfg)
    params = struct();
    params.L = cfg.L;
    % Optimize against a slightly stricter threshold than the common
    % high-precision evaluation target.
    params.dMin = cfg.dMin + cfg.csscOptimizationClearanceBuffer;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = 160;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;

    params.wObs = 60000.0;
    params.wClear = 100.0;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0.0;
    params.wCurv = 0.0;
    params.kappaMax = 2.0;

    params.solver = struct();
    params.solver.gradMode = 'semi-analytic';
    params.numIter = 100;
    params.lr = 0.0015;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 100;

    params.activeTopK = 30;
    params.activeClearanceMargin = 0.008;

    params.stop = struct();
    params.stop.enable = true;
    params.stop.minIter = 10;
    params.stop.window = 8;
    params.stop.tolRelJ = 1e-2;
    params.stop.tolGrad = 1e-1;
    params.stop.tolStep = 1e-3;
    params.stop.patience = 10;
    params.stop.tolBestRel = 1e-4;
    params.stop.requireSafe = true;
    params.stop.clearanceMargin = 0.0;
    params.stop.maxTimeSec = cfg.maxPlanningTimeSec;

    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

%% Planner wrappers

function opts = makePlannerOpts(cfg, envInfo, seed)
    opts = struct();
    opts.bounds = envInfo.bounds;
    opts.stepSize = cfg.stepSize;
    opts.goalBias = cfg.goalBias;
    opts.goalTol = cfg.goalTol;
    opts.maxIter = cfg.maxIter;
    opts.maxTimeSec = cfg.maxPlanningTimeSec;
    opts.collisionResolution = cfg.collisionResolution;
    opts.inflateRadius = cfg.inflateRadius;
    opts.seed = seed;

    opts.rewireRadius = cfg.rewireRadius;
    opts.minRewireRadius = cfg.minRewireRadius;
    opts.rewireGamma = cfg.rewireGamma;
    opts.terminateOnFirstSolution = cfg.terminateRRTStarOnFirstSolution;
    opts.maxNoImproveIter = cfg.maxRRTStarNoImproveIter;
end

function [path, info, planTimeSec] = runPlanner(plannerName, envInfo, obstacles, opts)
    tPlan = tic;
    switch lower(plannerName)
        case 'rrt'
            [path, info] = planRRT2D(envInfo.startPt, envInfo.goalPt, obstacles, opts);

        case {'rrtstar','rrt*'}
            [path, info] = planRRTStar2D(envInfo.startPt, envInfo.goalPt, obstacles, opts);

        otherwise
            error('Unknown planner: %s', plannerName);
    end
    planTimeSec = toc(tPlan);
end

function [result, caseData] = runOneMethod( ...
        result, plannerName, envInfo, obstacles, ...
        cfg, paramsEval, paramsOpt, plannerSeed)
    caseData = emptyCaseData();
    plannerOpts = makePlannerOpts(cfg, envInfo, plannerSeed);

    switch lower(plannerName)
        case {'rrt','rrtstar','rrt*'}
            [path, planInfo, planTimeSec] = runPlanner( ...
                plannerName, envInfo, obstacles, plannerOpts);

            result.plannerSuccess = planInfo.success;
            result.planTimeSec = planTimeSec;
            result.numIter = getFieldOrDefault(planInfo, 'numIter', nan);
            result.numNodes = getFieldOrDefault(planInfo, 'numNodes', nan);
            result.firstSolutionIter = getFieldOrDefault(planInfo, 'firstSolutionIter', nan);
            result.timeToFirstSolutionSec = getFieldOrDefault( ...
                planInfo, 'timeToFirstSolutionSec', nan);
            result.bestCost = getFieldOrDefault(planInfo, 'bestCost', nan);
            result.stopReason = string(getFieldOrDefault( ...
                planInfo, 'terminationReason', ''));
            result.timedOut = isTimeTermination(result.stopReason);
            result.timeBudgetSec = cfg.maxPlanningTimeSec;
            result.plannerMessage = string(planInfo.message);
            result.frontendMethod = string(getFieldOrDefault( ...
                planInfo, 'method', plannerName));

            if planInfo.success
                tEval = tic;
                [metrics, hpMetrics, Peval] = ...
                    evaluateBaselinePath(path, obstacles, paramsEval);
                result.evalTimeSec = toc(tEval);
                result = copyPathMetrics(result, metrics);
                result.pathRepresentation = "piecewise-linear-exact-chord";
            else
                tEval = tic;
                hpMetrics = evaluatePolylineHighPrecision2D( ...
                    zeros(0,2), obstacles, paramsEval);
                result.evalTimeSec = toc(tEval);
                Peval = zeros(0,2);
            end
            result.totalEvaluationTimeSec = result.evalTimeSec;

            caseData.paths = struct( ...
                'Pinit', Peval, 'Popt', Peval, 'Pref', [], ...
                'pathRRT', path, 'pathBSplineInit', [], ...
                'pathOptimized', path, 'pathFrontend', path);
            caseData.optimizerInfo = planInfo;
            caseData.parameters = struct( ...
                'planner', plannerOpts, ...
                'evaluation', paramsEval);
            caseData.hpMetrics = hpMetrics;
            caseData.stageMetrics.final = hpMetrics;
            caseData.timing = struct( ...
                'planTimeSec', result.planTimeSec, ...
                'evalTimeSec', result.evalTimeSec);

        case {'cssc','cssc (rrt)','cssc-rrt'}
            [result, caseData] = runCSSCMethod( ...
                result, envInfo, obstacles, ...
                plannerOpts, paramsEval, paramsOpt, cfg, plannerSeed, ...
                cfg.csscFrontendMethod);

        case {'rrtsc-2d','rrtsc2d','rrtsc'}
            [result, caseData] = runRRTSCMethod( ...
                result, envInfo, obstacles, ...
                plannerOpts, paramsEval, cfg, plannerSeed);

        case {'adaptive sp-rrt-2d','adaptive-sprrt-2d', ...
                'sp-rrt-2d','sprrt-2d','sprrt2d','sprrt'}
            [result, caseData] = runSpRRTMethod( ...
                result, envInfo, obstacles, ...
                paramsEval, cfg, plannerSeed);

        otherwise
            error('Unknown method: %s', plannerName);
    end
end

function [result, caseData] = runSpRRTMethod( ...
        result, envInfo, obstacles, paramsEval, cfg, plannerSeed)
    caseData = emptyCaseData();
    opts = defaultSpRRT2DParams();
    opts.L = paramsEval.L;
    opts.nominalLinkCount = cfg.sprrtNominalLinkCount;
    opts.maxSegmentCount = cfg.sprrtMaxSegmentCount;
    opts.thetaMax = deg2rad(cfg.sprrtThetaMaxDeg);
    opts.bounds = envInfo.bounds;
    opts.seed = plannerSeed;
    opts.maxIter = cfg.maxIter;
    opts.maxTimeSec = cfg.sprrtMaxTimeSec;
    opts.entranceBias = cfg.sprrtEntranceBias;
    opts.collisionResolution = cfg.collisionResolution;
    opts.inflateRadius = cfg.inflateRadius;
    opts.pathOptimization.mode = cfg.sprrtPathOptimizationMode;
    opts.pathOptimization.thetaMax = opts.thetaMax;
    opts.emitWarnings = false;
    opts.verbose = false;

    [pathOptimized, spInfo] = planAdaptiveSpRRT2D( ...
        envInfo.startPt, envInfo.goalPt, obstacles, opts);

    result.plannerSuccess = spInfo.success;
    result.planTimeSec = spInfo.planningTimeSec;
    result.frontendTimeSec = max(0, ...
        spInfo.planningTimeSec - spInfo.pathOptimizationTimeSec);
    result.initTimeSec = spInfo.pathOptimizationTimeSec;
    result.optTimeSec = 0;
    result.numIter = spInfo.numIter;
    result.numNodes = spInfo.numNodes;
    result.firstSolutionIter = spInfo.firstSolutionIter;
    result.stopReason = string(spInfo.terminationReason);
    result.timedOut = isTimeTermination(result.stopReason);
    result.timeBudgetSec = cfg.maxPlanningTimeSec;
    result.pathRepresentation = "piecewise-linear-exact-chord";
    result.plannerMessage = string(spInfo.message);

    result.expandCallCount = spInfo.expandCallCount;
    result.expandSuccessCount = spInfo.expandSuccessCount;
    result.expandFailureCount = spInfo.expandFailureCount;
    result.abandonedCandidateCount = spInfo.abandonedCandidateCount;
    result.averageCandidatesPerExpand = spInfo.averageCandidatesPerExpand;
    result.angleRejectCount = spInfo.angleRejectCount;
    result.collisionRejectCount = spInfo.collisionRejectCount;
    result.boundsRejectCount = spInfo.boundsRejectCount;
    result.lengthRejectCount = spInfo.lengthRejectCount;
    result.depthRejectCount = spInfo.depthRejectCount;
    result.shortcutCandidateCount = spInfo.shortcutCandidateCount;
    result.shortcutAcceptedCount = spInfo.shortcutAcceptedCount;
    result.rawPathLength = spInfo.rawPathLength;
    result.optimizedPathLength = spInfo.optimizedPathLength;
    result.rawMaxTurn = spInfo.rawMaxTurn;
    result.optimizedMaxTurn = spInfo.optimizedMaxTurn;
    result.directDistance = spInfo.directDistance;
    result.nominalLinkCount = spInfo.nominalLinkCount;
    result.maxSegmentCount = spInfo.maxSegmentCount;
    result.nominalTotalLength = spInfo.nominalTotalLength;
    result.maximumAllowedLength = spInfo.maximumAllowedLength;
    result.finalRawSegmentCount = spInfo.finalRawSegmentCount;
    result.finalOptimizedWaypointCount = ...
        spInfo.finalOptimizedWaypointCount;
    result.usedExtraSegments = spInfo.usedExtraSegments;
    result.numberOfExtraSegments = spInfo.numberOfExtraSegments;
    result.maxDepthReached = spInfo.maxDepthReached;
    result.totalRobotLength = spInfo.totalRobotLength;
    result.reachabilityMargin = spInfo.reachabilityMargin;
    result.warningMessage = string(spInfo.warningMessage);

    if spInfo.success
        tEval = tic;
        hpMetrics = evaluateSpRRTPath2D( ...
            pathOptimized, obstacles, paramsEval);
        result.evalTimeSec = toc(tEval);
        result = copyPathMetrics(result, ...
            pathMetricsFromHighPrecision(hpMetrics));
        result.numWaypoints = size(pathOptimized, 1);
        result.nCtrl = size(pathOptimized, 1);
        result.frontendPathLength = spInfo.rawPathLength;
        result.bestCost = spInfo.optimizedPathLength;
    else
        tEval = tic;
        hpMetrics = evaluateSpRRTPath2D( ...
            zeros(0,2), obstacles, paramsEval);
        result.evalTimeSec = toc(tEval);
    end
    result.totalEvaluationTimeSec = result.evalTimeSec;

    caseData.paths = struct( ...
        'Pinit', pathOptimized, ...
        'Popt', pathOptimized, ...
        'Pref', [], ...
        'pathRRT', spInfo.rawPath, ...
        'pathRaw', spInfo.rawPath, ...
        'pathBSplineInit', [], ...
        'pathOptimized', pathOptimized);
    caseData.optimizerInfo = spInfo;
    caseData.parameters = struct( ...
        'planner', opts, ...
        'evaluation', paramsEval);
    caseData.hpMetrics = hpMetrics;
    caseData.stageMetrics.final = hpMetrics;
    caseData.timing = struct( ...
        'planTimeSec', result.planTimeSec, ...
        'pathOptimizationTimeSec', spInfo.pathOptimizationTimeSec, ...
        'evalTimeSec', result.evalTimeSec);
end

function [result, caseData] = runCSSCMethod( ...
        result, envInfo, obstacles, plannerOpts, ...
        paramsEval, paramsOpt, cfg, plannerSeed, frontendMethod)
    caseData = emptyCaseData();
    if nargin < 9 || isempty(frontendMethod)
        frontendMethod = 'rrt';
    end
    tMethod = tic;
    result.timeBudgetSec = cfg.maxPlanningTimeSec;
    result.pathRepresentation = "cubic-bspline-degree-3";

    initInfo = selectCSSCInitialCandidate( ...
        envInfo, obstacles, plannerOpts, paramsOpt, cfg, ...
        plannerSeed, frontendMethod, tMethod);
    result.frontendTimeSec = initInfo.frontendTimeSec;
    result.initTimeSec = initInfo.initTimeSec;
    result.planTimeSec = toc(tMethod);
    result.frontendMethod = string(frontendMethod);
    result.plannerSuccess = initInfo.success;
    result.initialCandidateCount = initInfo.numCandidates;
    result.selectedInitialAttempt = initInfo.selectedAttempt;
    result.initialMinClear = initInfo.selectedMinClear;
    result.initialSafe = initInfo.selectedSafe;

    frontendInfo = initInfo.frontendInfo;
    result.numNodes = getFieldOrDefault(frontendInfo, 'numNodes', nan);
    result.firstSolutionIter = getFieldOrDefault( ...
        frontendInfo, 'firstSolutionIter', nan);
    result.frontendPathLength = getFieldOrDefault( ...
        frontendInfo, 'pathLength', nan);
    result.plannerMessage = string(initInfo.message);

    if ~initInfo.success
        result.stopReason = string(initInfo.terminationReason);
        result.timedOut = isTimeTermination(result.stopReason);
        tEval = tic;
        hpMetrics = evaluateCSSCHighPrecision([], obstacles, paramsEval);
        result.evalTimeSec = toc(tEval);
        result.totalEvaluationTimeSec = result.evalTimeSec;
        caseData.paths = struct( ...
            'Pinit', [], 'Popt', [], 'Pref', [], ...
            'pathRRT', [], 'pathBSplineInit', [], ...
            'pathOptimized', [], 'pathFrontend', []);
        caseData.optimizerInfo = initInfo;
        caseData.parameters = struct( ...
            'planner', plannerOpts, ...
            'evaluation', paramsEval, ...
            'optimization', paramsOpt);
        caseData.hpMetrics = hpMetrics;
        caseData.stageMetrics.final = hpMetrics;
        caseData.timing = struct( ...
            'frontendTimeSec', result.frontendTimeSec, ...
            'initTimeSec', result.initTimeSec, 'optTimeSec', 0, ...
            'planTimeSec', result.planTimeSec, ...
            'finalEvalTimeSec', result.evalTimeSec, ...
            'totalEvaluationTimeSec', result.totalEvaluationTimeSec);
        return;
    end

    pathFrontend = initInfo.pathFrontend;
    shortcutInfo = initInfo.shortcutInfo;
    Pinit = initInfo.Pinit;
    splineInfo = initInfo.splineInfo;
    result.shortcutAccepted = shortcutInfo.numAccepted;
    result.nCtrl = size(Pinit, 1);

    params = paramsOpt;
    params.knot = splineInfo.knot;
    params.stop.maxTimeSec = max(0, ...
        cfg.maxPlanningTimeSec - toc(tMethod));
    Pref = Pinit;

    tOpt = tic;
    [optLog, Popt, optInfo] = evalc( ...
        'optimizeCSSC2D(Pinit, Pref, obstacles, params)'); %#ok<ASGLU>
    result.optTimeSec = toc(tOpt);
    result.numIter = optInfo.numIterActual;
    result.bestIter = optInfo.returnedIter;
    result.bestSafeIter = optInfo.bestSafeIter;
    result.bestCost = optInfo.finalJ;
    result.stopReason = string(optInfo.stopReason);
    result.timedOut = isTimeTermination(result.stopReason) || ...
        toc(tMethod) >= cfg.maxPlanningTimeSec;
    optInfo.frontendMethod = char(result.frontendMethod);
    optInfo.frontendInfo = frontendInfo;
    optInfo.initializationInfo = initInfo;
    result.planTimeSec = toc(tMethod);

    tFrontendEval = tic;
    frontendHpMetrics = evaluatePolylineHighPrecision2D( ...
        pathFrontend, obstacles, paramsEval);
    result.frontendEvalTimeSec = toc(tFrontendEval);

    paramsEvalInit = paramsEval;
    paramsEvalInit.knot = splineInfo.knot;
    tInitialEval = tic;
    initialHpMetrics = evaluateCSSCHighPrecision( ...
        Pinit, obstacles, paramsEvalInit);
    result.initialEvalTimeSec = toc(tInitialEval);

    tFinalEval = tic;
    paramsEvalOpt = paramsEval;
    paramsEvalOpt.knot = splineInfo.knot;
    [metrics, hpMetrics] = evaluateBSplinePathMetrics( ...
        Popt, obstacles, paramsEvalOpt, cfg);
    result.evalTimeSec = toc(tFinalEval);
    result.totalEvaluationTimeSec = result.frontendEvalTimeSec + ...
        result.initialEvalTimeSec + result.evalTimeSec;
    result = copyPathMetrics(result, metrics);
    result = copyCSSCStageMetrics( ...
        result, frontendHpMetrics, initialHpMetrics, hpMetrics);

    caseData.paths = struct( ...
        'Pinit', Pinit, 'Popt', Popt, 'Pref', Pref, ...
        'pathRRT', pathFrontend, ...
        'pathBSplineInit', initialHpMetrics.pathSample, ...
        'pathOptimized', hpMetrics.pathSample, ...
        'pathFrontend', pathFrontend);
    caseData.optimizerInfo = optInfo;
    caseData.parameters = struct( ...
        'planner', plannerOpts, ...
        'evaluation', paramsEvalOpt, ...
        'optimization', params);
    caseData.hpMetrics = hpMetrics;
    caseData.stageMetrics.frontend = frontendHpMetrics;
    caseData.stageMetrics.initial = initialHpMetrics;
    caseData.stageMetrics.final = hpMetrics;
    caseData.timing = struct( ...
        'frontendTimeSec', result.frontendTimeSec, ...
        'initTimeSec', result.initTimeSec, ...
        'optTimeSec', result.optTimeSec, ...
        'planTimeSec', result.planTimeSec, ...
        'frontendEvalTimeSec', result.frontendEvalTimeSec, ...
        'initialEvalTimeSec', result.initialEvalTimeSec, ...
        'finalEvalTimeSec', result.evalTimeSec, ...
        'totalEvaluationTimeSec', result.totalEvaluationTimeSec);
end

function initInfo = selectCSSCInitialCandidate( ...
        envInfo, obstacles, plannerOpts, paramsOpt, cfg, ...
        plannerSeed, frontendMethod, tMethod)
    initInfo = emptyCSSCInitInfo();
    maxCandidates = max(1, round(cfg.csscInitMaxCandidates));
    initBudgetSec = cfg.maxPlanningTimeSec * cfg.csscInitTimeFraction;
    attemptTemplate = struct( ...
        'attempt', 0, 'seed', 0, 'frontendSuccess', false, ...
        'minClear', nan, 'pathLength', nan, 'nCtrl', 0, ...
        'frontendTimeSec', 0, 'initTimeSec', 0, 'message', '');
    attemptLog = repmat(attemptTemplate, maxCandidates, 1);

    for attempt = 1:maxCandidates
        if toc(tMethod) >= initBudgetSec
            break;
        end

        candidateSeed = plannerSeed + ...
            (attempt - 1) * cfg.csscInitSeedStride;
        frontendOpts = plannerOpts;
        frontendOpts.method = frontendMethod;
        frontendOpts.seed = candidateSeed;
        frontendOpts.maxTimeSec = max(0, ...
            initBudgetSec - toc(tMethod));

        tFront = tic;
        [pathCandidate, frontendInfo] = planCSSCFrontend2D( ...
            envInfo.startPt, envInfo.goalPt, obstacles, frontendOpts);
        dtFront = toc(tFront);
        initInfo.frontendTimeSec = initInfo.frontendTimeSec + dtFront;
        initInfo.numAttempts = attempt;

        attemptLog(attempt).attempt = attempt;
        attemptLog(attempt).seed = candidateSeed;
        attemptLog(attempt).frontendSuccess = frontendInfo.success;
        attemptLog(attempt).frontendTimeSec = dtFront;
        attemptLog(attempt).message = frontendInfo.message;

        if ~frontendInfo.success
            continue;
        end

        tInit = tic;
        shortcutOpts = plannerOpts;
        shortcutOpts.seed = cfg.shortcutSeedBase + candidateSeed;
        shortcutOpts.numShortcut = cfg.numShortcut;
        [pathShort, shortcutInfo] = shortcutPath2D( ...
            pathCandidate, obstacles, shortcutOpts);
        approxLen = polylineLength(pathShort);
        nCtrl = max(paramsOpt.degree + 1, ...
            ceil(approxLen / (0.5 * paramsOpt.L)) + 1);
        [Pcandidate, splineInfo] = polylineToBSplineInit2D( ...
            pathShort, struct('degree', paramsOpt.degree, 'nCtrl', nCtrl));

        scoreParams = paramsOpt;
        scoreParams.knot = splineInfo.knot;
        scoreParams.clearanceMode = 'segment';
        scoreParams.enablePathSample = false;
        scoreParams.enablePointClearance = false;
        scoreParams.enableObstacleMetadata = false;
        state = evaluateCSSCGlobal(Pcandidate, obstacles, scoreParams);
        dtInit = toc(tInit);
        initInfo.initTimeSec = initInfo.initTimeSec + dtInit;
        initInfo.numCandidates = initInfo.numCandidates + 1;

        attemptLog(attempt).minClear = state.minClear;
        attemptLog(attempt).pathLength = approxLen;
        attemptLog(attempt).nCtrl = nCtrl;
        attemptLog(attempt).initTimeSec = dtInit;

        if ~isfinite(state.minClear)
            continue;
        end

        isBetter = ~initInfo.success || ...
            state.minClear > initInfo.selectedMinClear || ...
            (state.minClear == initInfo.selectedMinClear && ...
            approxLen < initInfo.selectedPathLength);
        if isBetter
            initInfo.success = true;
            initInfo.selectedAttempt = attempt;
            initInfo.selectedMinClear = state.minClear;
            initInfo.selectedPathLength = approxLen;
            initInfo.selectedSafe = state.minClear >= ...
                paramsOpt.dMin + cfg.csscInitAcceptMargin;
            initInfo.pathFrontend = pathCandidate;
            initInfo.frontendInfo = frontendInfo;
            initInfo.shortcutInfo = shortcutInfo;
            initInfo.Pinit = Pcandidate;
            initInfo.splineInfo = splineInfo;
        end

        if initInfo.selectedSafe
            break;
        end
    end

    initInfo.attemptLog = attemptLog(1:initInfo.numAttempts);
    if initInfo.success
        initInfo.message = sprintf( ...
            'Selected CSSC initial candidate %d/%d with minClear %.6g.', ...
            initInfo.selectedAttempt, initInfo.numAttempts, ...
            initInfo.selectedMinClear);
        initInfo.terminationReason = 'candidateSelected';
    elseif toc(tMethod) >= initBudgetSec
        initInfo.message = 'CSSC initialization reached its time budget.';
        initInfo.terminationReason = 'maxInitializationTime';
    else
        initInfo.message = 'CSSC initialization produced no valid candidate.';
        initInfo.terminationReason = 'noValidInitialCandidate';
    end
end

function info = emptyCSSCInitInfo()
    info = struct();
    info.success = false;
    info.numAttempts = 0;
    info.numCandidates = 0;
    info.selectedAttempt = nan;
    info.selectedMinClear = nan;
    info.selectedPathLength = nan;
    info.selectedSafe = false;
    info.frontendTimeSec = 0;
    info.initTimeSec = 0;
    info.pathFrontend = zeros(0,2);
    info.frontendInfo = struct();
    info.shortcutInfo = struct('numAccepted', 0);
    info.Pinit = zeros(0,2);
    info.splineInfo = struct();
    info.attemptLog = struct([]);
    info.message = '';
    info.terminationReason = 'notStarted';
end

function [result, caseData] = runRRTSCMethod( ...
        result, envInfo, obstacles, plannerOpts, paramsEval, cfg, plannerSeed)
    caseData = emptyCaseData();
    opts = defaultRRTSC2DParams();
    opts.L = paramsEval.L;
    opts.dMin = paramsEval.dMin;
    opts.safetyMargin = paramsEval.dMin;
    opts.degree = paramsEval.degree;
    opts.bounds = envInfo.bounds;
    opts.seed = plannerSeed;
    opts.maxAttempts = cfg.rrtscMaxAttempts;
    opts.maxTotalTime = cfg.rrtscMaxTotalTime;
    opts.verbose = false;
    opts.rrt = plannerOpts;
    opts.centerline.sampleResolution = cfg.rrtscCenterlineResolution;
    opts.centerline.minSamples = cfg.rrtscCenterlineMinSamples;
    opts.chord.nU = cfg.rrtscChordNU;
    opts.chord.maxRefinement = cfg.rrtscMaxRefinement;
    opts.controlPointSpacingFactor = ...
        cfg.rrtscControlPointSpacingFactor;

    [Paccepted, pathRRT, rrtscInfo] = planRRTSC2D( ...
        envInfo.startPt, envInfo.goalPt, obstacles, opts);

    result.plannerSuccess = rrtscInfo.outputAvailable;
    result.rrtscMethodAccepted = rrtscInfo.strictAccepted;
    result.fallbackReturned = rrtscInfo.fallbackReturned;
    result.returnedAttempt = rrtscInfo.returnedAttempt;
    result.methodSolutionStatus = string(rrtscInfo.solutionStatus);
    result.planTimeSec = rrtscInfo.planningTimeSec;
    result.frontendTimeSec = rrtscInfo.timing.rrtTimeSec;
    result.initTimeSec = rrtscInfo.timing.splineTimeSec;
    result.optTimeSec = 0;
    result.centerlineValidationTimeSec = ...
        rrtscInfo.timing.centerlineValidationTimeSec;
    result.chordValidationTimeSec = ...
        rrtscInfo.timing.chordValidationTimeSec;
    result.attempts = rrtscInfo.attemptCount;
    result.globalReplanningCount = rrtscInfo.globalReplanningCount;
    result.rrtFailureCount = rrtscInfo.rrtFailureCount;
    result.centerlineRejectCount = rrtscInfo.centerlineRejectCount;
    result.chordRejectCount = rrtscInfo.chordRejectCount;
    result.numericalFailureCount = rrtscInfo.numericalFailureCount;
    attemptLog = rrtscInfo.attemptLog;
    result.rrtscRRTSuccessCount = sum([attemptLog.rrtSuccess]);
    result.rrtscCenterlinePassCount = sum([attemptLog.centerlineSafe]);
    centerlinePassMask = [attemptLog.centerlineSafe];
    result.rrtscChordPassCount = sum( ...
        centerlinePassMask & [attemptLog.chordSafe]);
    result.rrtscC1Rate = safeRatio( ...
        result.rrtscRRTSuccessCount, rrtscInfo.attemptCount);
    result.rrtscC2GivenC1Rate = safeRatio( ...
        result.rrtscCenterlinePassCount, result.rrtscRRTSuccessCount);
    result.rrtscC3GivenC2Rate = safeRatio( ...
        result.rrtscChordPassCount, result.rrtscCenterlinePassCount);
    result.rrtscAnyRRTSuccess = result.rrtscRRTSuccessCount > 0;
    result.rrtscAnyCenterlinePass = ...
        result.rrtscCenterlinePassCount > 0;
    result.stopReason = string(rrtscInfo.terminationReason);
    result.timedOut = isTimeTermination(result.stopReason);
    result.timeBudgetSec = cfg.maxPlanningTimeSec;
    result.pathRepresentation = "cubic-bspline-degree-3";
    result.plannerMessage = string(rrtscInfo.message);
    result.numIter = rrtscInfo.attemptCount;

    if rrtscInfo.outputAvailable
        result.numWaypoints = size(pathRRT, 1);
        result.nCtrl = size(Paccepted, 1);
        result.frontendPathLength = polylineLength(pathRRT);
        result.numNodes = getFieldOrDefault( ...
            rrtscInfo.finalRRTInfo, 'numNodes', nan);
        result.firstSolutionIter = getFieldOrDefault( ...
            rrtscInfo.finalRRTInfo, 'firstSolutionIter', nan);
        result.bestCost = polylineLength(pathRRT);

        paramsEvalAccepted = paramsEval;
        paramsEvalAccepted.knot = rrtscInfo.finalSplineInfo.knot;
        tEval = tic;
        [metrics, hpMetrics] = evaluateBSplinePathMetrics( ...
            Paccepted, obstacles, paramsEvalAccepted, cfg);
        result.evalTimeSec = toc(tEval);
        result = copyPathMetrics(result, metrics);
    else
        tEval = tic;
        hpMetrics = evaluateCSSCHighPrecision([], obstacles, paramsEval);
        result.evalTimeSec = toc(tEval);
    end
    result.totalEvaluationTimeSec = result.evalTimeSec;

    caseData.paths = struct( ...
        'Pinit', Paccepted, 'Popt', Paccepted, 'Pref', [], ...
        'pathRRT', pathRRT, 'pathBSplineInit', hpMetrics.pathSample, ...
        'pathOptimized', hpMetrics.pathSample);
    caseData.optimizerInfo = rrtscInfo;
    caseData.parameters = struct( ...
        'planner', opts, ...
        'evaluation', paramsEval);
    caseData.hpMetrics = hpMetrics;
    caseData.stageMetrics.final = hpMetrics;
    caseData.timing = rrtscInfo.timing;
    caseData.timing.evalTimeSec = result.evalTimeSec;
end

%% Path evaluation

function [metrics, hpMetrics, Peval] = evaluateBaselinePath(path, obstacles, paramsEval)
    metrics = emptyPathMetrics();
    metrics.numWaypoints = size(path, 1);
    Peval = path;
    hpMetrics = evaluatePolylineHighPrecision2D( ...
        Peval, obstacles, paramsEval);
    metrics = pathMetricsFromHighPrecision(hpMetrics);
    metrics.numWaypoints = size(path, 1);
    metrics.nCtrl = size(path, 1);
end

function [metrics, hpMetrics] = evaluateBSplinePathMetrics(P, obstacles, paramsEval, cfg)
    metrics = emptyPathMetrics();

    hpMetrics = evaluateCSSCHighPrecision(P, obstacles, paramsEval);
    metrics = pathMetricsFromHighPrecision(hpMetrics);
    metrics.nCtrl = size(P, 1);
end

function metrics = pathMetricsFromHighPrecision(hpMetrics)
    metrics = emptyPathMetrics();
    metrics.envelopeMinClear = hpMetrics.minClear;
    metrics.envelopeClearMargin = hpMetrics.minClear - hpMetrics.dMin;
    metrics.envelopeSuccess = hpMetrics.success;
    metrics.envelopeDMinSatisfied = hpMetrics.dMinSatisfied;
    metrics.chordConstructionSuccess = hpMetrics.chordConstructionSuccess;
    metrics.ftlGeometricRealizable = hpMetrics.ftlGeometricRealizable;
    metrics.feasibilityNumericalFailure = ...
        hpMetrics.feasibilityNumericalFailure;
    metrics.maxChordLengthResidual = hpMetrics.maxChordLengthResidual;
    metrics.numWaypoints = hpMetrics.numPathSamples;
    metrics.pathLength = hpMetrics.pathLength;
    metrics.turnAbsSum = hpMetrics.turnAbsSum;
    metrics.turnSqSum = hpMetrics.turnSqSum;
    metrics.meanAbsTurn = hpMetrics.meanAbsTurn;
    metrics.pointMinClear = hpMetrics.pointMinClear;
    metrics.pointMinClearX = hpMetrics.pointMinPoint(1);
    metrics.pointMinClearY = hpMetrics.pointMinPoint(2);
    metrics.pointSuccess = hpMetrics.pointSuccess;
end

function result = copyCSSCStageMetrics( ...
        result, frontendMetrics, initialMetrics, finalMetrics)
    result.frontendPointMinClear = ...
        getFieldOrDefault(frontendMetrics, 'pointMinClear', nan);
    result.frontendEnvelopeMinClear = ...
        getFieldOrDefault(frontendMetrics, 'minClear', nan);
    result.frontendEnvelopeDMinSatisfied = ...
        getFieldOrDefault(frontendMetrics, 'dMinSatisfied', nan);
    result.frontendFTLGeometricRealizable = ...
        getFieldOrDefault(frontendMetrics, ...
        'ftlGeometricRealizable', nan);

    result.initialPointMinClear = ...
        getFieldOrDefault(initialMetrics, 'pointMinClear', nan);
    result.initialEnvelopeMinClear = ...
        getFieldOrDefault(initialMetrics, 'minClear', nan);
    result.initialEnvelopeDMinSatisfied = ...
        getFieldOrDefault(initialMetrics, 'dMinSatisfied', nan);
    result.initialFTLGeometricRealizable = ...
        getFieldOrDefault(initialMetrics, ...
        'ftlGeometricRealizable', nan);
    result.initialPathLengthHP = ...
        getFieldOrDefault(initialMetrics, 'pathLength', nan);

    finalMinClear = getFieldOrDefault(finalMetrics, 'minClear', nan);
    finalPathLength = getFieldOrDefault(finalMetrics, 'pathLength', nan);
    result.clearanceGainFromInitial = ...
        finalMinClear - result.initialEnvelopeMinClear;
    result.pathLengthChangeFromInitial = ...
        finalPathLength - result.initialPathLengthHP;
end

function [minClear, minX, minY] = polylineMinPointClearance(path, obstacles, resolution)
    samples = samplePolyline(path, resolution);

    minClear = inf;
    minX = nan;
    minY = nan;

    for i = 1:size(samples, 1)
        d = queryObstaclePointSDF2D(samples(i,:), obstacles);
        if d < minClear
            minClear = d;
            minX = samples(i,1);
            minY = samples(i,2);
        end
    end
end

function samples = samplePolyline(path, resolution)
    samples = zeros(0, 2);
    if isempty(path)
        return;
    end

    for i = 1:size(path, 1)-1
        a = path(i,:);
        b = path(i+1,:);
        len = norm(b - a);
        n = max(2, ceil(len / resolution) + 1);
        t = linspace(0, 1, n).';
        seg = (1 - t) .* a + t .* b;
        if i > 1
            seg = seg(2:end,:);
        end
        samples = [samples; seg]; %#ok<AGROW>
    end
end

function [turnAbsSum, turnSqSum, meanAbsTurn] = pathTurnMetrics(path)
    turnAbsSum = nan;
    turnSqSum = nan;
    meanAbsTurn = nan;

    if size(path, 1) < 3
        turnAbsSum = 0;
        turnSqSum = 0;
        meanAbsTurn = 0;
        return;
    end

    v1 = diff(path(1:end-1,:), 1, 1);
    v2 = diff(path(2:end,:), 1, 1);
    crossVal = v1(:,1).*v2(:,2) - v1(:,2).*v2(:,1);
    dotVal = sum(v1 .* v2, 2);
    ang = atan2(crossVal, dotVal);

    turnAbsSum = sum(abs(ang));
    turnSqSum = sum(ang.^2);
    meanAbsTurn = mean(abs(ang));
end

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

%% Result tables

function data = emptyCaseData()
    data = struct();
    data.paths = struct( ...
        'Pinit', [], 'Popt', [], 'Pref', [], ...
        'pathRRT', [], 'pathBSplineInit', [], ...
        'pathOptimized', [], 'pathFrontend', []);
    data.optimizerInfo = struct();
    data.hpMetrics = struct();
    data.timing = struct();
    data.stageMetrics = struct( ...
        'frontend', [], 'initial', [], 'final', []);
    data.parameters = struct();
end

function result = emptyBaselineResult()
    result = struct();
    result.caseId = 0;
    result.trialIndex = 0;
    result.sceneId = "";
    result.sceneName = "";
    result.sceneType = "";
    result.difficultyIndex = 0;
    result.difficultyId = "";
    result.difficultyName = "";
    result.difficultyMetric = "";
    result.difficultyValue = nan;
    result.envSeed = 0;
    result.planner = "";
    result.plannerSeed = 0;
    result.numObstacles = nan;
    result.frontendMethod = "";

    result.plannerSuccess = false;
    result.pathFound = false;
    result.centerlineCollisionFree = false;
    result.wholeBodyCollisionFree = false;
    result.dMinSatisfied = false;
    result.strictAccepted = false;
    result.fallbackReturned = false;
    result.numericallyCertified = false;
    result.solutionStatus = "";
    result.methodSolutionStatus = "";
    result.rrtscMethodAccepted = false;
    result.returnedAttempt = nan;
    result.pointSuccess = false;
    result.envelopeSuccess = false;
    result.envelopeDMinSatisfied = false;
    result.wholeBodySuccess = false;
    result.chordConstructionSuccess = false;
    result.ftlGeometricRealizable = false;
    result.feasibilityNumericalFailure = false;

    result.pointMinClear = nan;
    result.pointMinClearX = nan;
    result.pointMinClearY = nan;
    result.envelopeMinClear = nan;
    result.envelopeClearMargin = nan;
    result.maxChordLengthResidual = nan;
    result.frontendPointMinClear = nan;
    result.frontendEnvelopeMinClear = nan;
    result.frontendEnvelopeDMinSatisfied = nan;
    result.frontendFTLGeometricRealizable = nan;
    result.initialPointMinClear = nan;
    result.initialEnvelopeMinClear = nan;
    result.initialEnvelopeDMinSatisfied = nan;
    result.initialFTLGeometricRealizable = nan;
    result.initialPathLengthHP = nan;
    result.clearanceGainFromInitial = nan;
    result.pathLengthChangeFromInitial = nan;

    result.pathLength = nan;
    result.turnAbsSum = nan;
    result.turnSqSum = nan;
    result.meanAbsTurn = nan;
    result.numWaypoints = nan;
    result.nCtrl = nan;

    result.planTimeSec = nan;
    result.frontendTimeSec = nan;
    result.initTimeSec = nan;
    result.optTimeSec = nan;
    result.centerlineValidationTimeSec = nan;
    result.chordValidationTimeSec = nan;
    result.evalTimeSec = nan;
    result.frontendEvalTimeSec = nan;
    result.initialEvalTimeSec = nan;
    result.totalEvaluationTimeSec = nan;
    result.totalTimeSec = nan;
    result.timeBudgetSec = nan;
    result.timeToFirstSolutionSec = nan;
    result.timedOut = false;
    result.numIter = nan;
    result.numNodes = nan;
    result.firstSolutionIter = nan;
    result.bestCost = nan;
    result.bestIter = nan;
    result.bestSafeIter = nan;
    result.frontendPathLength = nan;
    result.shortcutAccepted = nan;
    result.initialCandidateCount = nan;
    result.selectedInitialAttempt = nan;
    result.initialMinClear = nan;
    result.initialSafe = false;
    result.attempts = nan;
    result.globalReplanningCount = nan;
    result.rrtFailureCount = nan;
    result.centerlineRejectCount = nan;
    result.chordRejectCount = nan;
    result.numericalFailureCount = nan;
    result.rrtscRRTSuccessCount = nan;
    result.rrtscCenterlinePassCount = nan;
    result.rrtscChordPassCount = nan;
    result.rrtscC1Rate = nan;
    result.rrtscC2GivenC1Rate = nan;
    result.rrtscC3GivenC2Rate = nan;
    result.rrtscAnyRRTSuccess = false;
    result.rrtscAnyCenterlinePass = false;

    result.expandCallCount = nan;
    result.expandSuccessCount = nan;
    result.expandFailureCount = nan;
    result.abandonedCandidateCount = nan;
    result.averageCandidatesPerExpand = nan;
    result.angleRejectCount = nan;
    result.collisionRejectCount = nan;
    result.boundsRejectCount = nan;
    result.lengthRejectCount = nan;
    result.depthRejectCount = nan;
    result.shortcutCandidateCount = nan;
    result.shortcutAcceptedCount = nan;
    result.rawPathLength = nan;
    result.optimizedPathLength = nan;
    result.rawMaxTurn = nan;
    result.optimizedMaxTurn = nan;
    result.directDistance = nan;
    result.nominalLinkCount = nan;
    result.maxSegmentCount = nan;
    result.nominalTotalLength = nan;
    result.maximumAllowedLength = nan;
    result.finalRawSegmentCount = nan;
    result.finalOptimizedWaypointCount = nan;
    result.usedExtraSegments = false;
    result.numberOfExtraSegments = nan;
    result.maxDepthReached = nan;
    result.totalRobotLength = nan;
    result.reachabilityMargin = nan;
    result.warningMessage = "";

    result.stopReason = "";
    result.pathRepresentation = "";
    result.resultMatPath = "";

    result.plannerMessage = "";
    result.errorIdentifier = "";
    result.errorMessage = "";
    result.errorReport = "";
end

function metrics = emptyPathMetrics()
    metrics = struct();
    metrics.pointSuccess = false;
    metrics.envelopeSuccess = false;
    metrics.envelopeDMinSatisfied = false;
    metrics.chordConstructionSuccess = false;
    metrics.ftlGeometricRealizable = false;
    metrics.feasibilityNumericalFailure = false;
    metrics.maxChordLengthResidual = nan;
    metrics.pointMinClear = nan;
    metrics.pointMinClearX = nan;
    metrics.pointMinClearY = nan;
    metrics.envelopeMinClear = nan;
    metrics.envelopeClearMargin = nan;
    metrics.pathLength = nan;
    metrics.turnAbsSum = nan;
    metrics.turnSqSum = nan;
    metrics.meanAbsTurn = nan;
    metrics.numWaypoints = nan;
    metrics.nCtrl = nan;
end

function result = copyPathMetrics(result, metrics)
    fields = fieldnames(metrics);
    for i = 1:numel(fields)
        result.(fields{i}) = metrics.(fields{i});
    end
    result.wholeBodySuccess = result.envelopeDMinSatisfied && ...
        result.ftlGeometricRealizable;
end

function summaryTable = summarizeBaselineResults(T)
    sceneIds = unique(T.sceneId, 'stable');
    difficultyIndices = unique(T.difficultyIndex, 'stable');
    planners = unique(T.planner, 'stable');

    rows = repmat(emptySummaryRow(), ...
        numel(sceneIds) * numel(difficultyIndices) * numel(planners), 1);
    krow = 0;

    for i = 1:numel(sceneIds)
        for d = 1:numel(difficultyIndices)
            for j = 1:numel(planners)
                mask = strcmp(T.sceneId, sceneIds(i)) & ...
                    T.difficultyIndex == difficultyIndices(d) & ...
                    strcmp(T.planner, planners(j));
                rowsT = T(mask, :);
                nTrial = height(rowsT);

                krow = krow + 1;
                rows(krow).sceneId = string(sceneIds(i));
                rows(krow).difficultyIndex = difficultyIndices(d);
                if nTrial > 0
                    rows(krow).difficultyId = rowsT.difficultyId(1);
                    rows(krow).difficultyName = rowsT.difficultyName(1);
                    rows(krow).difficultyMetric = rowsT.difficultyMetric(1);
                    rows(krow).difficultyValue = rowsT.difficultyValue(1);
                end
                rows(krow).planner = string(planners(j));
                rows(krow).nTrial = nTrial;
                rows(krow).nValidEnvelope = ...
                    sum(isfinite(rowsT.envelopeMinClear));
                rows(krow).nValidPathQuality = ...
                    sum(isfinite(rowsT.pathLength));
                rows(krow).nValidFrontendHP = ...
                    sum(isfinite(rowsT.frontendEnvelopeMinClear));
                rows(krow).nValidInitialHP = ...
                    sum(isfinite(rowsT.initialEnvelopeMinClear));
                rows(krow).plannerSuccessRate = ...
                    safeRate(rowsT.plannerSuccess, nTrial);
                rows(krow).fallbackReturnedRate = ...
                    safeRate(rowsT.fallbackReturned, nTrial);
                rows(krow).pointSuccessRate = ...
                    safeRate(rowsT.pointSuccess, nTrial);
                rows(krow).envelopeSuccessRate = ...
                    safeRate(rowsT.envelopeSuccess, nTrial);
                rows(krow).envelopeDMinRate = ...
                    safeRate(rowsT.envelopeDMinSatisfied, nTrial);
                rows(krow).ftlGeometricRealizableRate = ...
                    safeRate(rowsT.ftlGeometricRealizable, nTrial);
                rows(krow).chordConstructionSuccessRate = ...
                    safeRate(rowsT.chordConstructionSuccess, nTrial);
                rows(krow).feasibilityNumericalFailureRate = ...
                    safeRate(rowsT.feasibilityNumericalFailure, nTrial);
                rows(krow).wholeBodySuccessRate = ...
                    safeRate(rowsT.wholeBodySuccess, nTrial);
                rows(krow).timeoutRate = safeRate(rowsT.timedOut, nTrial);
                rows(krow).pointClearMean = safeMean(rowsT.pointMinClear);
                rows(krow).envelopeClearMean = ...
                    safeMean(rowsT.envelopeMinClear);
                rows(krow).pointClearMin = safeMin(rowsT.pointMinClear);
                rows(krow).envelopeClearMin = ...
                    safeMin(rowsT.envelopeMinClear);
                rows(krow).envelopeClearMarginMean = ...
                    safeMean(rowsT.envelopeClearMargin);
                rows(krow).envelopeClearMarginMin = ...
                    safeMin(rowsT.envelopeClearMargin);
                rows(krow).maxChordLengthResidualMean = ...
                    safeMean(rowsT.maxChordLengthResidual);
                rows(krow).frontendDMinRate = safeFiniteRate( ...
                    rowsT.frontendEnvelopeDMinSatisfied);
                rows(krow).initialDMinRate = safeFiniteRate( ...
                    rowsT.initialEnvelopeDMinSatisfied);
                rows(krow).frontendEnvelopeClearMean = ...
                    safeMean(rowsT.frontendEnvelopeMinClear);
                rows(krow).initialEnvelopeClearMean = ...
                    safeMean(rowsT.initialEnvelopeMinClear);
                rows(krow).clearanceGainFromInitialMean = ...
                    safeMean(rowsT.clearanceGainFromInitial);
                rows(krow).pathLengthMean = safeMean(rowsT.pathLength);
                rows(krow).pathLengthChangeFromInitialMean = ...
                    safeMean(rowsT.pathLengthChangeFromInitial);
                rows(krow).turnAbsMean = safeMean(rowsT.turnAbsSum);
                rows(krow).turnSqMean = safeMean(rowsT.turnSqSum);
                rows(krow).planTimeMeanSec = safeMean(rowsT.planTimeSec);
                rows(krow).planTimeMedianSec = safeMedian(rowsT.planTimeSec);
                rows(krow).timeToFirstSolutionMeanSec = ...
                    safeMean(rowsT.timeToFirstSolutionSec);
                rows(krow).frontendTimeMeanSec = ...
                    safeMean(rowsT.frontendTimeSec);
                rows(krow).initTimeMeanSec = safeMean(rowsT.initTimeSec);
                rows(krow).optTimeMeanSec = safeMean(rowsT.optTimeSec);
                rows(krow).centerlineValidationTimeMeanSec = ...
                    safeMean(rowsT.centerlineValidationTimeSec);
                rows(krow).chordValidationTimeMeanSec = ...
                    safeMean(rowsT.chordValidationTimeSec);
                rows(krow).evalTimeMeanSec = safeMean(rowsT.evalTimeSec);
                rows(krow).frontendEvalTimeMeanSec = ...
                    safeMean(rowsT.frontendEvalTimeSec);
                rows(krow).initialEvalTimeMeanSec = ...
                    safeMean(rowsT.initialEvalTimeSec);
                rows(krow).totalEvaluationTimeMeanSec = ...
                    safeMean(rowsT.totalEvaluationTimeSec);
                rows(krow).totalTimeMeanSec = safeMean(rowsT.totalTimeSec);
                rows(krow).numNodesMean = safeMean(rowsT.numNodes);
                rows(krow).firstSolutionIterMean = ...
                    safeMean(rowsT.firstSolutionIter);
                rows(krow).bestIterMean = safeMean(rowsT.bestIter);
                rows(krow).attemptsMean = safeMean(rowsT.attempts);
                rows(krow).globalReplanningMean = ...
                    safeMean(rowsT.globalReplanningCount);
                rows(krow).rrtFailureMean = ...
                    safeMean(rowsT.rrtFailureCount);
                rows(krow).centerlineRejectMean = ...
                    safeMean(rowsT.centerlineRejectCount);
                rows(krow).chordRejectMean = ...
                    safeMean(rowsT.chordRejectCount);
                rows(krow).numericalFailureMean = ...
                    safeMean(rowsT.numericalFailureCount);
                rows(krow).expandCallsMean = ...
                    safeMean(rowsT.expandCallCount);
                rows(krow).abandonedCandidatesMean = ...
                    safeMean(rowsT.abandonedCandidateCount);
                rows(krow).averageCandidatesPerExpandMean = ...
                    safeMean(rowsT.averageCandidatesPerExpand);
                rows(krow).angleRejectMean = ...
                    safeMean(rowsT.angleRejectCount);
                rows(krow).collisionRejectMean = ...
                    safeMean(rowsT.collisionRejectCount);
                rows(krow).boundsRejectMean = ...
                    safeMean(rowsT.boundsRejectCount);
                rows(krow).lengthRejectMean = ...
                    safeMean(rowsT.lengthRejectCount);
                rows(krow).depthRejectMean = ...
                    safeMean(rowsT.depthRejectCount);
                rows(krow).shortcutCandidateMean = ...
                    safeMean(rowsT.shortcutCandidateCount);
                rows(krow).shortcutAcceptedMean = ...
                    safeMean(rowsT.shortcutAcceptedCount);
                rows(krow).rawPathLengthMean = ...
                    safeMean(rowsT.rawPathLength);
                rows(krow).optimizedPathLengthMean = ...
                    safeMean(rowsT.optimizedPathLength);
                rows(krow).rawMaxTurnMean = ...
                    safeMean(rowsT.rawMaxTurn);
                rows(krow).optimizedMaxTurnMean = ...
                    safeMean(rowsT.optimizedMaxTurn);
                rows(krow).reachabilityMarginMean = ...
                    safeMean(rowsT.reachabilityMargin);
                rows(krow).directDistanceMean = ...
                    safeMean(rowsT.directDistance);
                rows(krow).nominalTotalLengthMean = ...
                    safeMean(rowsT.nominalTotalLength);
                rows(krow).maximumAllowedLengthMean = ...
                    safeMean(rowsT.maximumAllowedLength);
                rows(krow).finalRawSegmentCountMean = ...
                    safeMean(rowsT.finalRawSegmentCount);
                rows(krow).finalOptimizedWaypointCountMean = ...
                    safeMean(rowsT.finalOptimizedWaypointCount);
                rows(krow).usedExtraSegmentsRate = ...
                    safeRatio(sum(rowsT.usedExtraSegments & ...
                    rowsT.plannerSuccess), sum(rowsT.plannerSuccess));
                rows(krow).numberOfExtraSegmentsMean = ...
                    safeMean(rowsT.numberOfExtraSegments);
                rows(krow).maxDepthReachedMean = ...
                    safeMean(rowsT.maxDepthReached);
                rows(krow).totalRobotLengthMean = ...
                    safeMean(rowsT.totalRobotLength);

                if strcmpi(planners(j), "RRTSC-2D")
                    rows(krow).rrtscMethodAcceptedRate = ...
                        safeRate(rowsT.rrtscMethodAccepted, nTrial);
                    rows(krow).rrtscFallbackReturnedRate = ...
                        safeRate(rowsT.fallbackReturned, nTrial);
                    nAttempts = safeSum(rowsT.attempts);
                    nC1 = safeSum(rowsT.rrtscRRTSuccessCount);
                    nC2 = safeSum(rowsT.rrtscCenterlinePassCount);
                    nC3 = safeSum(rowsT.rrtscChordPassCount);
                    rows(krow).rrtscC1Rate = safeRatio(nC1, nAttempts);
                    rows(krow).rrtscC2GivenC1Rate = ...
                        safeRatio(nC2, nC1);
                    rows(krow).rrtscC3GivenC2Rate = ...
                        safeRatio(nC3, nC2);
                    rows(krow).rrtscTrialRRTSuccessRate = ...
                        safeRate(rowsT.rrtscAnyRRTSuccess, nTrial);
                    rows(krow).rrtscTrialCenterlinePassRate = ...
                        safeRate(rowsT.rrtscAnyCenterlinePass, nTrial);
                end
            end
        end
    end

    summaryTable = struct2table(rows(1:krow));
end

function row = emptySummaryRow()
    row = struct();
    row.sceneId = "";
    row.difficultyIndex = 0;
    row.difficultyId = "";
    row.difficultyName = "";
    row.difficultyMetric = "";
    row.difficultyValue = nan;
    row.planner = "";
    row.nTrial = 0;
    row.nValidEnvelope = 0;
    row.nValidPathQuality = 0;
    row.nValidFrontendHP = 0;
    row.nValidInitialHP = 0;
    row.plannerSuccessRate = nan;
    row.fallbackReturnedRate = nan;
    row.pointSuccessRate = nan;
    row.envelopeSuccessRate = nan;
    row.envelopeDMinRate = nan;
    row.ftlGeometricRealizableRate = nan;
    row.chordConstructionSuccessRate = nan;
    row.feasibilityNumericalFailureRate = nan;
    row.wholeBodySuccessRate = nan;
    row.timeoutRate = nan;
    row.pointClearMean = nan;
    row.envelopeClearMean = nan;
    row.pointClearMin = nan;
    row.envelopeClearMin = nan;
    row.envelopeClearMarginMean = nan;
    row.envelopeClearMarginMin = nan;
    row.maxChordLengthResidualMean = nan;
    row.frontendDMinRate = nan;
    row.initialDMinRate = nan;
    row.frontendEnvelopeClearMean = nan;
    row.initialEnvelopeClearMean = nan;
    row.clearanceGainFromInitialMean = nan;
    row.pathLengthMean = nan;
    row.pathLengthChangeFromInitialMean = nan;
    row.turnAbsMean = nan;
    row.turnSqMean = nan;
    row.planTimeMeanSec = nan;
    row.planTimeMedianSec = nan;
    row.timeToFirstSolutionMeanSec = nan;
    row.frontendTimeMeanSec = nan;
    row.initTimeMeanSec = nan;
    row.optTimeMeanSec = nan;
    row.centerlineValidationTimeMeanSec = nan;
    row.chordValidationTimeMeanSec = nan;
    row.evalTimeMeanSec = nan;
    row.frontendEvalTimeMeanSec = nan;
    row.initialEvalTimeMeanSec = nan;
    row.totalEvaluationTimeMeanSec = nan;
    row.totalTimeMeanSec = nan;
    row.numNodesMean = nan;
    row.firstSolutionIterMean = nan;
    row.bestIterMean = nan;
    row.attemptsMean = nan;
    row.globalReplanningMean = nan;
    row.rrtFailureMean = nan;
    row.centerlineRejectMean = nan;
    row.chordRejectMean = nan;
    row.numericalFailureMean = nan;
    row.expandCallsMean = nan;
    row.abandonedCandidatesMean = nan;
    row.averageCandidatesPerExpandMean = nan;
    row.angleRejectMean = nan;
    row.collisionRejectMean = nan;
    row.boundsRejectMean = nan;
    row.lengthRejectMean = nan;
    row.depthRejectMean = nan;
    row.shortcutCandidateMean = nan;
    row.shortcutAcceptedMean = nan;
    row.rawPathLengthMean = nan;
    row.optimizedPathLengthMean = nan;
    row.rawMaxTurnMean = nan;
    row.optimizedMaxTurnMean = nan;
    row.reachabilityMarginMean = nan;
    row.directDistanceMean = nan;
    row.nominalTotalLengthMean = nan;
    row.maximumAllowedLengthMean = nan;
    row.finalRawSegmentCountMean = nan;
    row.finalOptimizedWaypointCountMean = nan;
    row.usedExtraSegmentsRate = nan;
    row.numberOfExtraSegmentsMean = nan;
    row.maxDepthReachedMean = nan;
    row.totalRobotLengthMean = nan;
    row.rrtscC1Rate = nan;
    row.rrtscC2GivenC1Rate = nan;
    row.rrtscC3GivenC2Rate = nan;
    row.rrtscTrialRRTSuccessRate = nan;
    row.rrtscTrialCenterlinePassRate = nan;
    row.rrtscMethodAcceptedRate = nan;
    row.rrtscFallbackReturnedRate = nan;
end

function r = safeRate(x, nDen)
    if nDen <= 0
        r = nan;
    else
        r = sum(x) / nDen;
    end
end

function r = safeFiniteRate(x)
    valid = isfinite(x);
    if ~any(valid)
        r = nan;
    else
        r = sum(x(valid)) / nnz(valid);
    end
end

function m = safeMean(x)
    x = x(isfinite(x));
    if isempty(x)
        m = nan;
    else
        m = mean(x);
    end
end

function m = safeMin(x)
    x = x(isfinite(x));
    if isempty(x)
        m = nan;
    else
        m = min(x);
    end
end

function m = safeMedian(x)
    x = x(isfinite(x));
    if isempty(x)
        m = nan;
    else
        m = median(x);
    end
end

function s = safeSum(x)
    x = x(isfinite(x));
    if isempty(x)
        s = 0;
    else
        s = sum(x);
    end
end

function r = safeRatio(numerator, denominator)
    if ~isfinite(denominator) || denominator <= 0
        r = nan;
    else
        r = numerator / denominator;
    end
end

function val = getFieldOrDefault(s, fieldName, defaultVal)
    if isstruct(s) && isfield(s, fieldName)
        val = s.(fieldName);
    else
        val = defaultVal;
    end
end

function result = finalizeSolutionStatus(result)
    result.pathFound = result.plannerSuccess;
    result.centerlineCollisionFree = result.pointSuccess;
    result.wholeBodyCollisionFree = result.envelopeSuccess;
    result.dMinSatisfied = result.envelopeDMinSatisfied;
    result.numericallyCertified = result.ftlGeometricRealizable;
    result.strictAccepted = result.wholeBodySuccess;

    if ~result.pathFound
        result.solutionStatus = "no-path";
    elseif ~result.centerlineCollisionFree
        result.solutionStatus = "centerline-collision";
    elseif ~result.numericallyCertified
        result.solutionStatus = "numerically-uncertified";
    elseif result.strictAccepted
        result.solutionStatus = "strict-safe";
    elseif result.wholeBodyCollisionFree
        result.solutionStatus = "collision-free-below-dmin";
    else
        result.solutionStatus = "centerline-only";
    end
end

function tf = isTimeTermination(reason)
    reason = lower(string(reason));
    tf = contains(reason, 'max') && contains(reason, 'time');
end

%% Output helpers

function provenance = prepareExperimentProvenance(cfg)
    current = collectExperimentProvenance(cfg);
    if ~cfg.saveResults
        provenance = current;
        return;
    end

    provenanceExists = exist(cfg.provenancePath, 'file') == 2;
    checkpointExists = exist(cfg.checkpointPath, 'file') == 2;
    if provenanceExists && ~cfg.resume
        error(['Provenance already exists for this runName. Use a new ' ...
            'runName or enable resume.']);
    end
    if checkpointExists && ~provenanceExists
        error(['Checkpoint exists without experiment provenance. Refusing ' ...
            'to resume an uncertified run directory.']);
    end

    if cfg.resume && provenanceExists
        loaded = load(cfg.provenancePath, 'provenance');
        if ~isfield(loaded, 'provenance')
            error('Invalid provenance file: %s', cfg.provenancePath);
        end
        provenance = loaded.provenance;
        mismatch = provenanceMismatch(provenance, current);
        if mismatch
            message = ['The saved run provenance differs from the current ' ...
                'code or MATLAB environment.'];
            if cfg.allowResumeProvenanceMismatch
                warning('%s Resume was explicitly allowed.', message);
            else
                error('%s Set allowResumeProvenanceMismatch=true only if intentional.', ...
                    message);
            end
        end
        return;
    end

    existingCases = dir(fullfile(cfg.caseDir, 'case_*', 'result.mat'));
    if ~isempty(existingCases)
        error(['Case files already exist without experiment provenance. ' ...
            'Use a new runName to avoid mixing experiments.']);
    end

    provenance = current;
    saveProvenanceAtomic(cfg, provenance);
end

function provenance = collectExperimentProvenance(cfg)
    provenance = struct();
    provenance.schemaVersion = 'simu1-provenance-v1';
    provenance.createdAt = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
    provenance.script = fullfile( ...
        cfg.projectRoot, 'demos', 'simu1_baseline_compare.m');
    provenance.projectRoot = cfg.projectRoot;
    provenance.runName = cfg.runName;
    provenance.matlabVersion = version;
    provenance.matlabRelease = version('-release');
    provenance.computer = computer;
    provenance.hostName = getenv('COMPUTERNAME');
    provenance.processor = getenv('PROCESSOR_IDENTIFIER');
    provenance.logicalProcessorCount = str2double( ...
        getenv('NUMBER_OF_PROCESSORS'));
    provenance.userName = getenv('USERNAME');

    try
        memoryInfo = memory;
        provenance.memory = memoryInfo;
    catch
        provenance.memory = struct();
    end

    provenance.gitCommit = runGitCommand( ...
        cfg.projectRoot, 'rev-parse HEAD');
    provenance.gitBranch = runGitCommand( ...
        cfg.projectRoot, 'rev-parse --abbrev-ref HEAD');
    provenance.gitStatus = runGitCommand( ...
        cfg.projectRoot, 'status --porcelain=v1 --untracked-files=all');
    provenance.gitDirty = ~isempty(strtrim(provenance.gitStatus));
end

function output = runGitCommand(projectRoot, arguments)
    command = sprintf('git -C "%s" %s', projectRoot, arguments);
    [status, output] = system(command);
    if status ~= 0
        output = '';
    else
        output = strtrim(output);
    end
end

function mismatch = provenanceMismatch(saved, current)
    fields = {'gitCommit', 'gitStatus', 'matlabRelease', 'computer'};
    mismatch = false;
    for i = 1:numel(fields)
        name = fields{i};
        savedValue = string(getFieldOrDefault(saved, name, ""));
        currentValue = string(getFieldOrDefault(current, name, ""));
        if ~isequal(savedValue, currentValue)
            mismatch = true;
            return;
        end
    end
end

function saveProvenanceAtomic(cfg, provenance)
    tmpMatPath = [tempname(cfg.outDir), '.mat'];
    cleanupObj = onCleanup( ...
        @() deleteTemporaryFile(tmpMatPath)); %#ok<NASGU>
    save(tmpMatPath, 'provenance');
    replaceFileAtomic(tmpMatPath, cfg.provenancePath);

    tmpTextPath = [tempname(cfg.outDir), '.txt'];
    fid = fopen(tmpTextPath, 'w');
    if fid < 0
        error('Could not create provenance text file.');
    end
    fprintf(fid, 'CSSC baseline experiment provenance\n\n');
    fprintf(fid, 'createdAt: %s\n', provenance.createdAt);
    fprintf(fid, 'script: %s\n', provenance.script);
    fprintf(fid, 'runName: %s\n', provenance.runName);
    fprintf(fid, 'MATLAB: %s (%s)\n', ...
        provenance.matlabVersion, provenance.matlabRelease);
    fprintf(fid, 'computer: %s\n', provenance.computer);
    fprintf(fid, 'host: %s\n', provenance.hostName);
    fprintf(fid, 'processor: %s\n', provenance.processor);
    fprintf(fid, 'logical processors: %g\n', ...
        provenance.logicalProcessorCount);
    fprintf(fid, 'git commit: %s\n', provenance.gitCommit);
    fprintf(fid, 'git branch: %s\n', provenance.gitBranch);
    fprintf(fid, 'git dirty: %d\n', provenance.gitDirty);
    fprintf(fid, 'git status:\n%s\n', provenance.gitStatus);
    fclose(fid);
    replaceFileAtomic(tmpTextPath, cfg.provenanceTextPath);
end

function definition = makeRunDefinition( ...
        cfg, sceneSpecs, paramsEval, paramsOpt)
    definition = struct();
    definition.schemaVersion = 'simu1-run-definition-v1';
    definition.cfg = cfg;
    operationalFields = { ...
        'provenance', 'resume', 'allowResumeProvenanceMismatch', ...
        'printEachRun', 'saveResults', 'saveCaseResults'};
    for i = 1:numel(operationalFields)
        if isfield(definition.cfg, operationalFields{i})
            definition.cfg = rmfield( ...
                definition.cfg, operationalFields{i});
        end
    end
    definition.sceneSpecs = sceneSpecs;
    definition.paramsEval = paramsEval;
    definition.paramsOpt = paramsOpt;
end

function [results, completedMask, resumeInfo] = initializeBatchState( ...
        cfg, totalRuns, runDefinition)
    results = repmat(emptyBaselineResult(), totalRuns, 1);
    completedMask = false(totalRuns, 1);
    resumeInfo = struct( ...
        'resumed', false, ...
        'lastCompletedCaseId', 0, ...
        'previousElapsedSec', 0);

    if ~cfg.saveResults
        return;
    end

    checkpointExists = exist(cfg.checkpointPath, 'file') == 2;
    if checkpointExists && ~cfg.resume
        error(['Checkpoint already exists for this runName. Use a new ' ...
            'runName or enable resume.']);
    end
    if ~checkpointExists
        existingCases = dir(fullfile( ...
            cfg.caseDir, 'case_*', 'result.mat'));
        if ~isempty(existingCases)
            error(['Case files exist but batch_checkpoint.mat is missing. ' ...
                'Use a new runName to avoid overwriting prior data.']);
        end
        return;
    end

    loaded = load(cfg.checkpointPath, 'checkpoint');
    if ~isfield(loaded, 'checkpoint')
        error('Invalid checkpoint file: %s', cfg.checkpointPath);
    end
    checkpoint = loaded.checkpoint;
    required = {'results', 'completedMask', 'runDefinition'};
    for i = 1:numel(required)
        if ~isfield(checkpoint, required{i})
            error('Checkpoint is missing field: %s', required{i});
        end
    end
    if ~isequaln(checkpoint.runDefinition, runDefinition)
        error(['Checkpoint definition does not match the current scene, ' ...
            'method, parameter, or source-code configuration.']);
    end
    if numel(checkpoint.results) ~= totalRuns || ...
            numel(checkpoint.completedMask) ~= totalRuns
        error('Checkpoint size does not match the requested batch.');
    end

    results = checkpoint.results;
    completedMask = logical(checkpoint.completedMask(:));
    resumeInfo.resumed = true;
    resumeInfo.lastCompletedCaseId = getFieldOrDefault( ...
        checkpoint, 'lastCompletedCaseId', 0);
    resumeInfo.previousElapsedSec = getFieldOrDefault( ...
        checkpoint, 'batchElapsedSec', 0);
end

function saveBatchCheckpointAtomic( ...
        cfg, results, completedMask, runDefinition, provenance, ...
        caseId, batchElapsedSec)
    checkpoint = struct();
    checkpoint.schemaVersion = 'simu1-checkpoint-v1';
    checkpoint.savedAt = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
    checkpoint.lastCompletedCaseId = caseId;
    checkpoint.batchElapsedSec = batchElapsedSec;
    checkpoint.numCompleted = nnz(completedMask);
    checkpoint.isComplete = all(completedMask);
    checkpoint.results = results;
    checkpoint.completedMask = completedMask;
    checkpoint.runDefinition = runDefinition;
    checkpoint.provenance = provenance;

    tmpPath = [tempname(cfg.outDir), '.mat'];
    cleanupObj = onCleanup(@() deleteTemporaryFile(tmpPath)); %#ok<NASGU>
    save(tmpPath, 'checkpoint');
    replaceFileAtomic(tmpPath, cfg.checkpointPath);
end

function writeTableAtomic(T, filePath)
    [folder, ~, extension] = fileparts(filePath);
    tmpPath = [tempname(folder), extension];
    cleanupObj = onCleanup(@() deleteTemporaryFile(tmpPath)); %#ok<NASGU>
    writetable(T, tmpPath);
    replaceFileAtomic(tmpPath, filePath);
end

function replaceFileAtomic(sourcePath, destinationPath)
    [ok, message] = movefile(sourcePath, destinationPath, 'f');
    if ~ok
        error('Could not replace output file %s: %s', ...
            destinationPath, message);
    end
end

function deleteTemporaryFile(filePath)
    if exist(filePath, 'file')
        delete(filePath);
    end
end

function flags = makeSuccessFlags(result)
    flags = struct();
    flags.plannerSuccess = result.plannerSuccess;
    flags.pathFound = result.pathFound;
    flags.highPrecisionSuccess = result.envelopeSuccess;
    flags.dMinSatisfied = result.envelopeDMinSatisfied;
    flags.ftlGeometricRealizable = result.ftlGeometricRealizable;
    flags.wholeBodySuccess = result.wholeBodySuccess;
    flags.fallbackReturned = result.fallbackReturned;
    flags.solutionStatus = result.solutionStatus;
    flags.pointSuccess = result.pointSuccess;
end

function filePath = saveCaseResult( ...
        cfg, row, obstacles, paths, optimizerInfo, hpMetrics, timing, ...
        flags, stageMetrics, caseParameters, ...
        paramsEval, paramsOpt, provenance)
    caseName = sprintf('case_%04d_%s_%s_%s', row.caseId, ...
        char(row.planner), char(row.sceneId), char(row.difficultyId));
    caseName = regexprep(caseName, '[^\w\-]', '_');
    caseDir = fullfile(cfg.caseDir, caseName);
    if ~exist(caseDir, 'dir')
        mkdir(caseDir);
    end

    seed = struct('envSeed', row.envSeed, ...
        'plannerSeed', row.plannerSeed, 'trialIndex', row.trialIndex);
    parameters = struct( ...
        'globalEvaluation', paramsEval, ...
        'globalOptimization', paramsOpt, ...
        'effective', caseParameters);
    result = makeCSSCExperimentResult(cfg, seed, obstacles, paths, ...
        optimizerInfo, hpMetrics, timing, flags, stageMetrics, ...
        parameters, provenance);
    filePathChar = fullfile(caseDir, 'result.mat');
    row.resultMatPath = string(filePathChar);
    result.summaryRow = row;

    tmpPath = [tempname(caseDir), '.mat'];
    cleanupObj = onCleanup(@() deleteTemporaryFile(tmpPath)); %#ok<NASGU>
    save(tmpPath, 'result');
    replaceFileAtomic(tmpPath, filePathChar);
    filePath = string(filePathChar);
end

function plotBaselineFigures(T, summaryTable, cfg)
    if ~exist(cfg.figDir, 'dir')
        mkdir(cfg.figDir);
    end

    metricSpecs = {
        'envelopeClearMargin', 'Safety-clearance margin', 'minClear - dMin', 'envelope_clearance_boxplot.jpg'
        'pathLength',       'Path length',         'length',    'path_length_boxplot.jpg'
        'turnAbsSum',       'Path turning',        'turn sum',  'turning_boxplot.jpg'
        'planTimeSec',      'Planning runtime',    'time (s)',  'runtime_boxplot.jpg'
    };

    for i = 1:size(metricSpecs, 1)
        plotMetricBoxplot(T, cfg, ...
            metricSpecs{i, 1}, metricSpecs{i, 2}, ...
            metricSpecs{i, 3}, metricSpecs{i, 4});
    end

    plotMeanOverview(summaryTable, cfg);
    plotRRTSCStageRates(summaryTable, cfg);
end

function plotMetricBoxplot(T, cfg, metricName, figTitle, yLabelText, fileName)
    sceneIds = unique(T.sceneId, 'stable');
    difficultyIndices = unique(T.difficultyIndex, 'stable');
    planners = string(cfg.plannerNames);
    fig = figure('Color','w', 'Name', figTitle, ...
        'Position',[60 40 1500 920]);
    tiledlayout(numel(difficultyIndices), numel(sceneIds), ...
        'Padding','compact', 'TileSpacing','compact');

    for d = 1:numel(difficultyIndices)
        for i = 1:numel(sceneIds)
            nexttile((d - 1) * numel(sceneIds) + i);
            rows = T(strcmp(T.sceneId, sceneIds(i)) & ...
                T.difficultyIndex == difficultyIndices(d), :);
            y = rows.(metricName);
            valid = isfinite(y);
            groupIndex = nan(height(rows), 1);
            nSuccess = zeros(1, numel(planners));
            nTotal = zeros(1, numel(planners));

            for j = 1:numel(planners)
                methodMask = strcmp(rows.planner, planners(j));
                groupIndex(methodMask) = j;
                nTotal(j) = sum(methodMask);
                nSuccess(j) = sum(methodMask & rows.wholeBodySuccess);
            end

            if any(valid)
                present = unique(groupIndex(valid), 'stable');
                boxplot(y(valid), groupIndex(valid), ...
                    'Positions', present, 'Symbol','k.');
            else
                text(0.5, 0.5, 'no valid data', ...
                    'HorizontalAlignment','center', 'FontWeight','bold');
            end

            xlim([0.5, numel(planners) + 0.5]);
            set(gca, 'XTick', 1:numel(planners), ...
                'XTickLabel', cellstr(planners), ...
                'TickLabelInterpreter','none');
            if strcmp(metricName, 'envelopeClearMargin')
                yline(0, '--', 'Color', [0.75, 0.15, 0.15]);
            elseif strcmp(metricName, 'planTimeSec') && ...
                    any(valid) && all(y(valid) > 0)
                set(gca, 'YScale', 'log');
            end
            grid on;

            for j = 1:numel(planners)
                xNormalized = (j - 0.5) / numel(planners);
                text(xNormalized, 0.02, ...
                    sprintf('%d/%d', nSuccess(j), nTotal(j)), ...
                    'Units', 'normalized', ...
                    'HorizontalAlignment','center', ...
                    'VerticalAlignment','bottom', ...
                    'FontSize', 7, 'Color', [0.25, 0.25, 0.25], ...
                    'BackgroundColor', 'white', 'Margin', 0.5);
            end

            difficultyName = cfg.difficultyNames(difficultyIndices(d));
            title(sprintf('%s | %s', ...
                char(sceneIds(i)), char(difficultyName)), ...
                'Interpreter','none');
            if i == 1
                ylabel(yLabelText);
            end
        end
    end

    sgtitle(figTitle, 'Interpreter','none');
    exportgraphics(fig, fullfile(cfg.figDir, fileName), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    savefig(fig, fullfile(cfg.figDir, strrep(fileName, '.jpg', '.fig')));
    close(fig);
end

function plotMeanOverview(S, cfg)
    sceneIds = unique(S.sceneId, 'stable');
    difficultyIndices = unique(S.difficultyIndex, 'stable');
    planners = string(cfg.plannerNames);
    fig = figure('Color','w', 'Name','Mean baseline summary', ...
        'Position',[60 60 1500 720]);
    tiledlayout(2, numel(sceneIds), ...
        'Padding','compact', 'TileSpacing','compact');

    for i = 1:numel(sceneIds)
        successData = summaryMatrix(S, sceneIds(i), ...
            difficultyIndices, planners, 'wholeBodySuccessRate');
        timeData = summaryMatrix(S, sceneIds(i), ...
            difficultyIndices, planners, 'planTimeMeanSec');

        nexttile(i);
        colororder(cfg.plannerColors);
        bar(1:numel(difficultyIndices), successData);
        title(char(sceneIds(i)), 'Interpreter','none');
        ylabel('whole-body success rate');
        ylim([0, 1]);
        set(gca, 'XTick', 1:numel(difficultyIndices), ...
            'XTickLabel', cellstr( ...
            cfg.difficultyNames(difficultyIndices)), ...
            'TickLabelInterpreter','none');
        grid on;

        nexttile(numel(sceneIds) + i);
        colororder(cfg.plannerColors);
        bar(1:numel(difficultyIndices), timeData);
        ylabel('mean planning time (s)');
        set(gca, 'YScale','log', ...
            'XTick', 1:numel(difficultyIndices), ...
            'XTickLabel', cellstr( ...
            cfg.difficultyNames(difficultyIndices)), ...
            'TickLabelInterpreter','none');
        grid on;
        if i == numel(sceneIds)
            legend(cellstr(planners), 'Location','southoutside', ...
                'Orientation','horizontal');
        end
    end

    sgtitle(sprintf('Mean results over %d trials', cfg.numSeedsPerScene));
    exportgraphics(fig, fullfile(cfg.figDir, 'summary_mean_overview.jpg'), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    savefig(fig, fullfile(cfg.figDir, 'summary_mean_overview.fig'));
    close(fig);
end

function plotRRTSCStageRates(S, cfg)
    sceneIds = unique(S.sceneId, 'stable');
    difficultyIndices = unique(S.difficultyIndex, 'stable');
    fig = figure('Color','w', 'Name','RRTSC-2D stage pass rates', ...
        'Position',[80 80 1400 420]);
    tiledlayout(1, numel(sceneIds), ...
        'Padding','compact', 'TileSpacing','compact');

    for i = 1:numel(sceneIds)
        stageRates = nan(numel(difficultyIndices), 3);
        for d = 1:numel(difficultyIndices)
            row = S(strcmp(S.sceneId, sceneIds(i)) & ...
                S.difficultyIndex == difficultyIndices(d) & ...
                strcmpi(S.planner, "RRTSC-2D"), :);
            if ~isempty(row)
                stageRates(d,:) = [
                    row.rrtscC1Rate(1), ...
                    row.rrtscC2GivenC1Rate(1), ...
                    row.rrtscC3GivenC2Rate(1)
                ];
            end
        end

        nexttile;
        bar(1:numel(difficultyIndices), stageRates);
        title(char(sceneIds(i)), 'Interpreter','none');
        ylabel('conditional pass rate');
        ylim([0, 1]);
        set(gca, 'XTick', 1:numel(difficultyIndices), ...
            'XTickLabel', cellstr( ...
            cfg.difficultyNames(difficultyIndices)), ...
            'TickLabelInterpreter','none');
        grid on;
        if i == numel(sceneIds)
            legend({'Collision 1', 'Collision 2 | C1', ...
                'Collision 3 | C2'}, ...
                'Location','southoutside', 'Orientation','horizontal');
        end
    end

    sgtitle('RRTSC-2D attempt-level stage pass rates');
    exportgraphics(fig, fullfile(cfg.figDir, ...
        'rrtsc_stage_pass_rates.jpg'), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    savefig(fig, fullfile(cfg.figDir, 'rrtsc_stage_pass_rates.fig'));
    close(fig);
end

function data = summaryMatrix(S, sceneId, difficultyIndices, planners, fieldName)
    data = nan(numel(difficultyIndices), numel(planners));
    for d = 1:numel(difficultyIndices)
        for j = 1:numel(planners)
            row = S(strcmp(S.sceneId, sceneId) & ...
                S.difficultyIndex == difficultyIndices(d) & ...
                strcmp(S.planner, planners(j)), :);
            if ~isempty(row)
                values = row.(fieldName);
                data(d,j) = values(1);
            end
        end
    end
end

function printOneRun(result, caseId, totalRuns)
    if result.plannerSuccess
        fprintf('%4d/%4d | %-9s | %-19s | %-6s | pointClr=%+.4f envClr=%+.4f | len=%.3f turn=%.3f | t=%.3fs', ...
            caseId, totalRuns, char(result.planner), char(result.sceneId), ...
            char(result.difficultyId), ...
            result.pointMinClear, result.envelopeMinClear, ...
            result.pathLength, result.turnAbsSum, result.planTimeSec);
        if strcmpi(result.planner, "RRTSC-2D")
            fprintf(' | attempts=%d replans=%d centerReject=%d chordReject=%d', ...
                result.attempts, result.globalReplanningCount, ...
                result.centerlineRejectCount, result.chordRejectCount);
        elseif strcmpi(result.planner, "Sp-RRT-2D")
            fprintf([' | nodes=%d segments=%d extra=%d ' ...
                'abandoned=%d angleReject=%d collisionReject=%d'], ...
                result.numNodes, result.finalRawSegmentCount, ...
                result.numberOfExtraSegments, ...
                result.abandonedCandidateCount, ...
                result.angleRejectCount, result.collisionRejectCount);
        end
        fprintf('\n');
    else
        msg = char(result.plannerMessage);
        if strlength(result.errorMessage) > 0
            msg = char(result.errorMessage);
        end
        fprintf('%4d/%4d | %-9s | %-19s | %-6s | failed | %s\n', ...
            caseId, totalRuns, char(result.planner), char(result.sceneId), ...
            char(result.difficultyId), msg);
    end
end

function writeMetricDefinitions(cfg)
    filePath = fullfile(cfg.outDir, 'metric_definitions.txt');
    fid = fopen(filePath, 'w');
    if fid < 0
        warning('Could not write metric definitions: %s', filePath);
        return;
    end

    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, ['RRT vs RRT* vs RRTSC-2D vs Sp-RRT-2D vs ' ...
        'CSSC metric definitions\n\n']);
    fprintf(fid, 'Benchmark geometry matches fig_Qualitative_result.m and fig_scenes_display.m. This run contains %d scene groups (%d families x %d difficulty levels).\n', ...
        cfg.numRunSceneGroups, cfg.numRunSceneFamilies, ...
        cfg.numRunDifficultyLevels);
    fprintf(fid, 'All methods and difficulty levels receive paired environment/planner seeds within one family and trial.\n');
    fprintf(fid, 'Planning-stage centerline collision checks for RRT, RRT*, Sp-RRT-2D, the RRTSC RRT stage, and the CSSC RRT frontend all require point/segment SDF > dMin through inflateRadius=dMin. This is a geometric safety margin, not a robot-radius term.\n');
    fprintf(fid, 'Final metrics query the original obstacles and apply dMin once; no obstacle inflation is added during evaluateCSSCHighPrecision.\n');
    fprintf(fid, 'plannerSuccess/pathFound: the method returned a complete start-to-goal path. For RRTSC-2D this includes an explicitly labelled fallback candidate when no strict candidate is accepted; use rrtscMethodAccepted or wholeBodySuccess for strict success.\n');
    fprintf(fid, 'CSSC optimization uses an internal clearance target dMin + %.6f = %.6f, while the common final evaluator remains at dMin = %.6f.\n', ...
        cfg.csscOptimizationClearanceBuffer, ...
        cfg.dMin + cfg.csscOptimizationClearanceBuffer, cfg.dMin);
    fprintf(fid, 'CSSC initialization: within %.0f%% of the common planning budget, generate at most %d deterministic RRT-shortcut-cubic-B-spline candidates and select the candidate with maximum segment-mode fixed-chord clearance; stop candidate generation once the internal CSSC clearance target is satisfied.\n', ...
        100 * cfg.csscInitTimeFraction, cfg.csscInitMaxCandidates);
    fprintf(fid, 'initialCandidateCount/selectedInitialAttempt/initialMinClear/initialSafe: CSSC initialization diagnostics relative to the internal optimization target.\n');
    fprintf(fid, 'stageHighPrecisionMetrics.frontend/initial/final: common high-precision evaluation of the selected raw RRT polyline, cubic B-spline initialization, and returned CSSC path. These evaluations are outside the planning-time budget.\n');
    fprintf(fid, 'frontendEnvelopeMinClear/initialEnvelopeMinClear: scalar trial-table copies of the corresponding stage metrics.\n');
    fprintf(fid, 'clearanceGainFromInitial/pathLengthChangeFromInitial: final minus initial values under the same high-precision evaluator.\n');
    fprintf(fid, 'CSSC best-safe return: bestP is updated for every strict J decrease, tolBestRel is used only by patience, and any historical iterate satisfying the internal clearance target is preferred at return through bestSafeP.\n');
    fprintf(fid, 'pointMinClear: high-resolution sampled centerline point SDF minimum.\n');
    fprintf(fid, 'pointSuccess: pointMinClear > 0.\n');
    fprintf(fid, 'path representation: RRT, RRT*, and Sp-RRT-2D retain their native polylines. Their fixed-chord endpoints are obtained from exact segment-circle intersections using the first forward root in path order; no degree-1 Newton solve or smoothing is used. RRTSC-2D and CSSC retain their native cubic B-spline outputs.\n');
    fprintf(fid, 'envelopeMinClear: high-precision fixed-chord clearance from the representation-specific evaluator on each method''s native reported path.\n');
    fprintf(fid, 'envelopeClearMargin: envelopeMinClear - dMin; nonnegative values satisfy the requested safety threshold.\n');
    fprintf(fid, 'envelopeSuccess: envelopeMinClear > 0.\n');
    fprintf(fid, 'envelopeDMinSatisfied: envelopeMinClear >= %.6f.\n', cfg.dMin);
    fprintf(fid, 'ftlGeometricRealizable: every sampled fixed chord returned by the representation-specific solver has finite endpoints, full segment-clearance coverage, and a length residual within tolerance. Polyline roots are analytic; cubic B-spline roots use the smooth-curve solver. Ideal joints are assumed and joint-angle/actuator limits are not imposed.\n');
    fprintf(fid, 'feasibilityNumericalFailure/chordConstructionSuccess/maxChordLengthResidual: numerical certification diagnostics saved per trial and aggregated in the summary table.\n');
    fprintf(fid, 'wholeBodySuccess: envelopeDMinSatisfied and ftlGeometricRealizable are both true.\n');
    fprintf(fid, 'solutionStatus: no-path, centerline-collision, numerically-uncertified, centerline-only, collision-free-below-dmin, or strict-safe according to the common high-precision evaluator.\n');
    fprintf(fid, 'fallbackReturned: RRTSC-2D exhausted its stopping limit without strict acceptance and returned the best complete rejected candidate for diagnostic/degraded use; it is never counted as strict success unless the common final evaluator independently certifies wholeBodySuccess.\n');
    fprintf(fid, 'pathLength: path length from the common high-precision evaluator on the native reported representation.\n');
    fprintf(fid, 'turnAbsSum: sum of absolute heading changes along the reported path.\n');
    fprintf(fid, 'turnSqSum: sum of squared heading changes along the reported path.\n');
    fprintf(fid, 'planTimeSec: end-to-end method runtime excluding final high-precision evaluation. The common budget is %.3f s; early success may return sooner, while timeout runs report best available output where supported.\n', cfg.maxPlanningTimeSec);
    fprintf(fid, 'timedOut/timeoutRate: whether the common planning budget terminated the method and its rate over all trials. Runtime remains reportable because methods are not forced to consume the full budget.\n');
    fprintf(fid, 'frontendTimeSec: frontend tree-search time; CSSC uses RRT before shortcut, B-spline initialization, and CSSC optimization.\n');
    fprintf(fid, 'initTimeSec: B-spline initialization time where applicable; for Sp-RRT-2D this compatibility field stores deterministic pruning time.\n');
    fprintf(fid, 'optTimeSec: CSSC optimization time when applicable.\n');
    fprintf(fid, 'attempts/globalReplanningCount: RRTSC-2D total attempts and restarts after the first attempt.\n');
    fprintf(fid, 'centerlineRejectCount/chordRejectCount: RRTSC-2D global candidate rejections at Collision 2/3.\n');
    fprintf(fid, 'rrtscC1Rate: raw RRT path success over all RRTSC attempts.\n');
    fprintf(fid, 'rrtscC2GivenC1Rate: smoothed-centerline pass count divided by Collision-1 successes.\n');
    fprintf(fid, 'rrtscC3GivenC2Rate: fixed-chord pass count divided by Collision-2 successes.\n');
    fprintf(fid, 'rrtscMethodAcceptedRate/rrtscFallbackReturnedRate: strict RRTSC acceptance and degraded-output return rates, respectively.\n');
    fprintf(fid, 'Sp-RRT-2D uses nominalLinkCount=6 only as robot metadata and maxSegmentCount=12 as the path-segment depth budget; fields report extra-segment use, maximum depth, rejection counts, raw/optimized length, and turns.\n');
    fprintf(fid, 'evalTimeSec: final-path high-precision evaluation time.\n');
    fprintf(fid, 'frontendEvalTimeSec/initialEvalTimeSec: auxiliary CSSC stage-evaluation times, excluded from planTimeSec.\n');
    fprintf(fid, 'totalEvaluationTimeSec: sum of all high-precision evaluation stages available for the method.\n');
    fprintf(fid, 'totalTimeSec: planner plus all metric evaluation and non-file-I/O overhead.\n');
    fprintf(fid, 'batch_checkpoint.mat: atomically replaced after every completed case and used for caseId-based resume with run-definition validation.\n');
    fprintf(fid, 'experiment_provenance.mat/txt: Git commit/status, MATLAB release, platform, processor, host, and run metadata.\n');
    fprintf(fid, 'Unexpected method exceptions are saved as completed failed cases with errorIdentifier, errorMessage, and errorReport.\n');
    fprintf(fid, 'Clearance/path-quality means and boxplots condition on finite returned paths; nValidEnvelope reports that denominator explicitly. Failed trials remain in success and timeout rates.\n');
    fprintf(fid, 'Boxplot labels n/N report whole-body-successful trials over all trials and are placed below each method distribution.\n');
    fprintf(fid, 'summary_mean_table.csv contains %d selected scene groups x %d methods; trial dimensions are aggregated by means/rates.\n', ...
        cfg.numRunSceneGroups, numel(cfg.plannerNames));
    fprintf(fid, 'Figures are saved under: %s\n', cfg.figDir);
end
