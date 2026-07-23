clear; clc; close all;

%% Baseline comparison: RRT vs RRT* vs CSSC
% This preliminary baseline experiment compares raw point-path planners and
% the full RRT-initialized CSSC backend. Results are saved as tables and
% boxplot figures under one results/runs folder. Final success, minClear,
% and dMin rates are computed by evaluateCSSCHighPrecision.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeBaselineConfig(projectRoot);
sceneSpecs = makeSceneSpecs(cfg);
paramsEval = makeEnvelopeEvalParams();
paramsOpt = makeCSSCOptimizationParams();

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
results = repmat(emptyBaselineResult(), totalRuns, 1);

fprintf('\n[RRT vs RRT* vs CSSC baseline comparison]\n');
fprintf('  output dir : %s\n', cfg.outDir);
fprintf('  scenes     : %d\n', numel(sceneSpecs));
fprintf('  seeds/scene: %d\n', cfg.numSeedsPerScene);
fprintf('  total runs : %d\n\n', totalRuns);

caseId = 0;
tAll = tic;

for iscene = 1:numel(sceneSpecs)
    spec = sceneSpecs(iscene);
    fprintf('\n[scene %d/%d] %s\n', iscene, numel(sceneSpecs), char(spec.name));

    for iseed = 1:cfg.numSeedsPerScene
        envSeed = cfg.envSeedBase + 1000 * iscene + iseed;
        [obstacles, envInfo] = makeEnvironmentForSpec(spec, cfg, envSeed);
        envInfo.obstacles = obstacles;

        for ialg = 1:numel(plannerNames)
            caseId = caseId + 1;
            plannerName = plannerNames{ialg};
            plannerSeed = cfg.plannerSeedBase + 100000 * ialg + 1000 * iscene + iseed;

            result = emptyBaselineResult();
            result.caseId = caseId;
            result.sceneId = string(spec.id);
            result.sceneName = string(spec.name);
            result.sceneType = string(spec.type);
            result.envSeed = envSeed;
            result.planner = string(plannerName);
            result.plannerSeed = plannerSeed;
            result.numObstacles = numel(obstacles);

            tRun = tic;
            try
                result = runOneMethod(result, plannerName, envInfo, obstacles, ...
                    cfg, paramsEval, paramsOpt, plannerSeed);
            catch ME
                result.errorMessage = string(ME.message);
            end

            result.totalTimeSec = toc(tRun);
            results(caseId) = result;

            if cfg.printEachRun
                printOneRun(result, caseId, totalRuns);
            end
        end
    end

    Tnow = struct2table(results(1:caseId));
    Snow = summarizeBaselineResults(Tnow);
    disp(Snow);
end

elapsedTotalSec = toc(tAll);
trialTable = struct2table(results);
summaryTable = summarizeBaselineResults(trialTable);

fprintf('\n[final RRT vs RRT* vs CSSC summary]\n');
disp(summaryTable);
fprintf('[elapsed] %.2f min\n', elapsedTotalSec / 60);

if cfg.saveResults
    writetable(trialTable, fullfile(cfg.outDir, 'trial_results.csv'));
    writetable(summaryTable, fullfile(cfg.outDir, 'summary_by_scene_planner.csv'));
    save(fullfile(cfg.outDir, 'baseline_rrt_rrtstar_cssc_results.mat'), ...
        'cfg', 'sceneSpecs', 'paramsEval', 'paramsOpt', 'results', ...
        'trialTable', 'summaryTable', 'elapsedTotalSec');
    writeMetricDefinitions(cfg);
    plotBaselineBoxplots(trialTable, cfg);
    fprintf('[saved] %s\n', cfg.outDir);
end

%% Configuration

function cfg = makeBaselineConfig(projectRoot)
    cfg = struct();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_baseline_rrt_rrtstar_cssc_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.caseDir = fullfile(cfg.outDir, 'cases');
    cfg.saveResults = true;
    cfg.saveCaseResults = true;
    cfg.printEachRun = true;
    cfg.plannerNames = {'RRT', 'RRT*', 'CSSC'};

    cfg.numSeedsPerScene = 10;  % Preliminary demo. Increase for paper-scale statistics.
    cfg.envSeedBase = 1200;
    cfg.plannerSeedBase = 500000;

    cfg.bounds = [0, 1; -0.4, 0.4];
    cfg.startPt = [0.05, 0.0];
    cfg.goalPt = [0.95, 0.0];

    cfg.stepSize = 0.030;
    cfg.goalBias = 0.16;
    cfg.goalTol = 0.035;
    cfg.maxIter = 4000;
    cfg.collisionResolution = 0.003;
    cfg.inflateRadius = 0.0;

    cfg.rewireRadius = 0.12;
    cfg.minRewireRadius = 0.060;
    cfg.rewireGamma = 0.45;
    cfg.terminateRRTStarOnFirstSolution = false;
    cfg.maxRRTStarNoImproveIter = 1200;

    cfg.pointClearanceResolution = 0.0015;
    cfg.dMin = 0.02;
    cfg.shortcutSeedBase = 710000;
    cfg.numShortcut = 180;
    cfg.figureResolution = 300;
end

function specs = makeSceneSpecs(cfg)
    specs = repmat(emptySceneSpec(), 3, 1);

    specs(1) = emptySceneSpec();
    specs(1).id = "double_slit";
    specs(1).name = "Offset Double Slit";
    specs(1).kind = "structured";
    specs(1).type = "offsetDoubleSlit";
    specs(1).gapHeight = 0.11;

    specs(2) = emptySceneSpec();
    specs(2).id = "s_channel";
    specs(2).name = "Four-Rect S Channel";
    specs(2).kind = "structured";
    specs(2).type = "fourRectSChannel";
    specs(2).dGap = 0.10;
    specs(2).channelW = 0.64;
    specs(2).channelH = cfg.bounds(2,2) - cfg.bounds(2,1);

    specs(3) = emptySceneSpec();
    specs(3).id = "random_mixed_medium";
    specs(3).name = "Random Mixed Medium";
    specs(3).kind = "random";
    specs(3).type = "randomMixed";
    specs(3).nCircle = 7;
    specs(3).nRect = 6;
    specs(3).minGap = 0.020;
end

function spec = emptySceneSpec()
    spec = struct();
    spec.id = "";
    spec.name = "";
    spec.kind = "";
    spec.type = "";
    spec.gapHeight = nan;
    spec.dGap = nan;
    spec.channelW = nan;
    spec.channelH = nan;
    spec.nCircle = nan;
    spec.nRect = nan;
    spec.minGap = nan;
end

function params = makeEnvelopeEvalParams()
    params = struct();
    params.L = 0.15;
    params.dMin = 0.02;
    params.dPref = 0.03;
    params.degree = 3;

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

function params = makeCSSCOptimizationParams()
    params = struct();
    params.L = 0.15;
    params.dMin = 0.02;
    params.dPref = 0.03;
    params.degree = 3;

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = 120;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;

    params.wObs = 600.0;
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
    params.lr = 0.001;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 100;

    params.activeTopK = 100;
    params.activeClearanceMargin = 0.0125;

    params.stop = struct();
    params.stop.enable = true;
    params.stop.minIter = 10;
    params.stop.window = 8;
    params.stop.tolRelJ = 1e-2;
    params.stop.tolGrad = 1e-1;
    params.stop.tolStep = 1e-3;
    params.stop.patience = 10;
    params.stop.tolBestRel = 1e-4;
    params.stop.requireSafe = false;
    params.stop.clearanceMargin = 0.0;

    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

%% Environment and planner wrappers

function [obstacles, envInfo] = makeEnvironmentForSpec(spec, cfg, seed)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    opts.seed = seed;

    switch lower(char(spec.kind))
        case 'structured'
            switch lower(char(spec.type))
                case 'offsetdoubleslit'
                    opts.gapHeight = spec.gapHeight;
                    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(spec.type, opts);
                    envInfo.seed = seed;

                case 'fourrectschannel'
                    opts.dGap = spec.dGap;
                    opts.W = spec.channelW;
                    opts.H = spec.channelH;
                    opts.yInOutRange = [-0.20, 0.20];
                    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(spec.type, opts);

                otherwise
                    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(spec.type, opts);
                    envInfo.seed = seed;
            end

        case 'random'
            opts.nCircle = spec.nCircle;
            opts.nRect = spec.nRect;
            opts.minGap = spec.minGap;
            opts.maxTry = 4000;
            [obstacles, envInfo] = generateCSSCEnvironment2D(spec.type, opts);

        otherwise
            error('Unknown scene kind: %s', char(spec.kind));
    end

    envInfo.id = char(spec.id);
    envInfo.name = char(spec.name);
end

function opts = makePlannerOpts(cfg, envInfo, seed)
    opts = struct();
    opts.bounds = envInfo.bounds;
    opts.stepSize = cfg.stepSize;
    opts.goalBias = cfg.goalBias;
    opts.goalTol = cfg.goalTol;
    opts.maxIter = cfg.maxIter;
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

function result = runOneMethod(result, plannerName, envInfo, obstacles, cfg, paramsEval, paramsOpt, plannerSeed)
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
            result.bestCost = getFieldOrDefault(planInfo, 'bestCost', nan);
            result.plannerMessage = string(planInfo.message);

            if planInfo.success
                tEval = tic;
                [metrics, hpMetrics, Peval, splineInfo] = ...
                    evaluateBaselinePath(path, obstacles, paramsEval, cfg);
                result.evalTimeSec = toc(tEval);
                result = copyPathMetrics(result, metrics);

                if cfg.saveResults && cfg.saveCaseResults
                    paths = struct('Pinit', Peval, 'Popt', Peval, 'Pref', Peval, ...
                        'pathRRT', path, 'pathBSplineInit', hpMetrics.pathSample, ...
                        'pathOptimized', hpMetrics.pathSample);
                    timing = struct('planTimeSec', result.planTimeSec, ...
                        'evalTimeSec', result.evalTimeSec);
                    flags = makeSuccessFlags(result);
                    result.resultMatPath = saveCaseResult(cfg, result, obstacles, ...
                        paths, planInfo, hpMetrics, timing, flags);
                end
            end

        case 'cssc'
            result = runCSSCMethod(result, envInfo, obstacles, ...
                plannerOpts, paramsEval, paramsOpt, cfg, plannerSeed);

        otherwise
            error('Unknown method: %s', plannerName);
    end
end

function result = runCSSCMethod(result, envInfo, obstacles, plannerOpts, paramsEval, paramsOpt, cfg, plannerSeed)
    tFront = tic;
    [pathRRT, rrtInfo] = planRRT2D(envInfo.startPt, envInfo.goalPt, obstacles, plannerOpts);
    result.frontendTimeSec = toc(tFront);

    result.plannerSuccess = rrtInfo.success;
    result.numNodes = getFieldOrDefault(rrtInfo, 'numNodes', nan);
    result.firstSolutionIter = getFieldOrDefault(rrtInfo, 'firstSolutionIter', nan);
    result.frontendPathLength = getFieldOrDefault(rrtInfo, 'pathLength', nan);
    result.plannerMessage = string(rrtInfo.message);

    if ~rrtInfo.success
        return;
    end

    shortcutOpts = plannerOpts;
    shortcutOpts.seed = cfg.shortcutSeedBase + plannerSeed;
    shortcutOpts.numShortcut = cfg.numShortcut;

    tInit = tic;
    [pathShort, shortcutInfo] = shortcutPath2D(pathRRT, obstacles, shortcutOpts);
    approxLen = polylineLength(pathShort);
    nCtrl = max(paramsOpt.degree + 1, ceil(approxLen / (0.5 * paramsOpt.L)) + 1);
    [Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, ...
        struct('degree', paramsOpt.degree, 'nCtrl', nCtrl));
    result.initTimeSec = toc(tInit);
    result.shortcutAccepted = shortcutInfo.numAccepted;
    result.nCtrl = size(Pinit, 1);

    params = paramsOpt;
    params.knot = splineInfo.knot;
    Pref = Pinit;

    tOpt = tic;
    optLog = evalc('[Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, params);'); %#ok<NASGU>
    result.optTimeSec = toc(tOpt);
    result.numIter = optInfo.numIterActual;
    result.bestIter = optInfo.bestIter;
    result.bestCost = optInfo.finalJ;
    result.stopReason = string(optInfo.stopReason);

    tEval = tic;
    paramsEvalOpt = paramsEval;
    paramsEvalOpt.knot = splineInfo.knot;
    [metrics, hpMetrics] = evaluateBSplinePathMetrics(Popt, obstacles, paramsEvalOpt, cfg);
    result.evalTimeSec = toc(tEval);
    result = copyPathMetrics(result, metrics);

    result.planTimeSec = result.frontendTimeSec + result.initTimeSec + result.optTimeSec;

    if cfg.saveResults && cfg.saveCaseResults
        paths = struct('Pinit', Pinit, 'Popt', Popt, 'Pref', Pref, ...
            'pathRRT', pathRRT, 'pathBSplineInit', [], ...
            'pathOptimized', hpMetrics.pathSample);
        timing = struct('frontendTimeSec', result.frontendTimeSec, ...
            'initTimeSec', result.initTimeSec, ...
            'optTimeSec', result.optTimeSec, ...
            'evalTimeSec', result.evalTimeSec);
        flags = makeSuccessFlags(result);
        result.resultMatPath = saveCaseResult(cfg, result, obstacles, ...
            paths, optInfo, hpMetrics, timing, flags);
    end
end

%% Path evaluation

function [metrics, hpMetrics, Pinit, splineInfo] = evaluateBaselinePath(path, obstacles, paramsEval, cfg)
    metrics = emptyPathMetrics();
    metrics.numWaypoints = size(path, 1);

    splineOpts = struct();
    splineOpts.degree = paramsEval.degree;
    splineOpts.nCtrl = max(paramsEval.degree + 1, ...
        ceil(metrics.pathLength / (0.5 * paramsEval.L)) + 1);

    [Pinit, splineInfo] = polylineToBSplineInit2D(path, splineOpts);
    paramsEnvelope = paramsEval;
    paramsEnvelope.knot = splineInfo.knot;
    hpMetrics = evaluateCSSCHighPrecision(Pinit, obstacles, paramsEnvelope);

    metrics.nCtrl = size(Pinit, 1);
    metrics.pathLength = hpMetrics.pathLength;
    metrics.turnAbsSum = hpMetrics.turnAbsSum;
    metrics.turnSqSum = hpMetrics.turnSqSum;
    metrics.meanAbsTurn = hpMetrics.meanAbsTurn;
    metrics.pointMinClear = hpMetrics.pointMinClear;
    metrics.pointMinClearX = hpMetrics.pointMinPoint(1);
    metrics.pointMinClearY = hpMetrics.pointMinPoint(2);
    metrics.pointSuccess = hpMetrics.pointSuccess;
    metrics.envelopeMinClear = hpMetrics.minClear;
    metrics.envelopeSuccess = hpMetrics.success;
    metrics.envelopeDMinSatisfied = hpMetrics.dMinSatisfied;
end

function [metrics, hpMetrics] = evaluateBSplinePathMetrics(P, obstacles, paramsEval, cfg)
    metrics = emptyPathMetrics();

    hpMetrics = evaluateCSSCHighPrecision(P, obstacles, paramsEval);
    metrics.envelopeMinClear = hpMetrics.minClear;
    metrics.envelopeSuccess = hpMetrics.success;
    metrics.envelopeDMinSatisfied = hpMetrics.dMinSatisfied;
    metrics.numWaypoints = hpMetrics.numPathSamples;
    metrics.pathLength = hpMetrics.pathLength;
    metrics.turnAbsSum = hpMetrics.turnAbsSum;
    metrics.turnSqSum = hpMetrics.turnSqSum;
    metrics.meanAbsTurn = hpMetrics.meanAbsTurn;
    metrics.pointMinClear = hpMetrics.pointMinClear;
    metrics.pointMinClearX = hpMetrics.pointMinPoint(1);
    metrics.pointMinClearY = hpMetrics.pointMinPoint(2);
    metrics.pointSuccess = hpMetrics.pointSuccess;
    metrics.nCtrl = size(P, 1);
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

function result = emptyBaselineResult()
    result = struct();
    result.caseId = 0;
    result.sceneId = "";
    result.sceneName = "";
    result.sceneType = "";
    result.envSeed = 0;
    result.planner = "";
    result.plannerSeed = 0;
    result.numObstacles = nan;

    result.plannerSuccess = false;
    result.pointSuccess = false;
    result.envelopeSuccess = false;
    result.envelopeDMinSatisfied = false;

    result.pointMinClear = nan;
    result.pointMinClearX = nan;
    result.pointMinClearY = nan;
    result.envelopeMinClear = nan;

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
    result.evalTimeSec = nan;
    result.totalTimeSec = nan;
    result.numIter = nan;
    result.numNodes = nan;
    result.firstSolutionIter = nan;
    result.bestCost = nan;
    result.bestIter = nan;
    result.frontendPathLength = nan;
    result.shortcutAccepted = nan;
    result.stopReason = "";
    result.resultMatPath = "";

    result.plannerMessage = "";
    result.errorMessage = "";
end

function metrics = emptyPathMetrics()
    metrics = struct();
    metrics.pointSuccess = false;
    metrics.envelopeSuccess = false;
    metrics.envelopeDMinSatisfied = false;
    metrics.pointMinClear = nan;
    metrics.pointMinClearX = nan;
    metrics.pointMinClearY = nan;
    metrics.envelopeMinClear = nan;
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
end

function summaryTable = summarizeBaselineResults(T)
    sceneIds = unique(T.sceneId, 'stable');
    planners = unique(T.planner, 'stable');

    rows = repmat(emptySummaryRow(), numel(sceneIds) * numel(planners), 1);
    krow = 0;

    for i = 1:numel(sceneIds)
        for j = 1:numel(planners)
            mask = strcmp(T.sceneId, sceneIds(i)) & strcmp(T.planner, planners(j));
            rowsT = T(mask, :);
            nTrial = height(rowsT);

            krow = krow + 1;
            rows(krow).sceneId = string(sceneIds(i));
            rows(krow).planner = string(planners(j));
            rows(krow).nTrial = nTrial;
            rows(krow).plannerSuccessRate = safeRate(rowsT.plannerSuccess, nTrial);
            rows(krow).pointSuccessRate = safeRate(rowsT.pointSuccess, nTrial);
            rows(krow).envelopeSuccessRate = safeRate(rowsT.envelopeSuccess, nTrial);
            rows(krow).envelopeDMinRate = safeRate(rowsT.envelopeDMinSatisfied, nTrial);
            rows(krow).pointClearMean = safeMean(rowsT.pointMinClear);
            rows(krow).envelopeClearMean = safeMean(rowsT.envelopeMinClear);
            rows(krow).pointClearMin = safeMin(rowsT.pointMinClear);
            rows(krow).envelopeClearMin = safeMin(rowsT.envelopeMinClear);
            rows(krow).pathLengthMean = safeMean(rowsT.pathLength);
            rows(krow).turnAbsMean = safeMean(rowsT.turnAbsSum);
            rows(krow).turnSqMean = safeMean(rowsT.turnSqSum);
            rows(krow).planTimeMeanSec = safeMean(rowsT.planTimeSec);
            rows(krow).frontendTimeMeanSec = safeMean(rowsT.frontendTimeSec);
            rows(krow).initTimeMeanSec = safeMean(rowsT.initTimeSec);
            rows(krow).optTimeMeanSec = safeMean(rowsT.optTimeSec);
            rows(krow).evalTimeMeanSec = safeMean(rowsT.evalTimeSec);
            rows(krow).totalTimeMeanSec = safeMean(rowsT.totalTimeSec);
            rows(krow).numNodesMean = safeMean(rowsT.numNodes);
            rows(krow).firstSolutionIterMean = safeMean(rowsT.firstSolutionIter);
            rows(krow).bestIterMean = safeMean(rowsT.bestIter);
        end
    end

    summaryTable = struct2table(rows(1:krow));
end

function row = emptySummaryRow()
    row = struct();
    row.sceneId = "";
    row.planner = "";
    row.nTrial = 0;
    row.plannerSuccessRate = nan;
    row.pointSuccessRate = nan;
    row.envelopeSuccessRate = nan;
    row.envelopeDMinRate = nan;
    row.pointClearMean = nan;
    row.envelopeClearMean = nan;
    row.pointClearMin = nan;
    row.envelopeClearMin = nan;
    row.pathLengthMean = nan;
    row.turnAbsMean = nan;
    row.turnSqMean = nan;
    row.planTimeMeanSec = nan;
    row.frontendTimeMeanSec = nan;
    row.initTimeMeanSec = nan;
    row.optTimeMeanSec = nan;
    row.evalTimeMeanSec = nan;
    row.totalTimeMeanSec = nan;
    row.numNodesMean = nan;
    row.firstSolutionIterMean = nan;
    row.bestIterMean = nan;
end

function r = safeRate(x, nDen)
    if nDen <= 0
        r = nan;
    else
        r = sum(x) / nDen;
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

function val = getFieldOrDefault(s, fieldName, defaultVal)
    if isstruct(s) && isfield(s, fieldName)
        val = s.(fieldName);
    else
        val = defaultVal;
    end
end

%% Output helpers

function flags = makeSuccessFlags(result)
    flags = struct();
    flags.plannerSuccess = result.plannerSuccess;
    flags.highPrecisionSuccess = result.envelopeSuccess;
    flags.dMinSatisfied = result.envelopeDMinSatisfied;
    flags.pointSuccess = result.pointSuccess;
end

function filePath = saveCaseResult(cfg, row, obstacles, paths, optimizerInfo, hpMetrics, timing, flags)
    caseName = sprintf('case_%04d_%s_%s', row.caseId, char(row.planner), char(row.sceneId));
    caseName = regexprep(caseName, '[^\w\-]', '_');
    caseDir = fullfile(cfg.caseDir, caseName);
    if ~exist(caseDir, 'dir')
        mkdir(caseDir);
    end

    seed = struct('envSeed', row.envSeed, 'plannerSeed', row.plannerSeed);
    result = makeCSSCExperimentResult(cfg, seed, obstacles, paths, ...
        optimizerInfo, hpMetrics, timing, flags);
    result.summaryRow = row;

    filePathChar = fullfile(caseDir, 'result.mat');
    save(filePathChar, 'result');
    filePath = string(filePathChar);
end

function plotBaselineBoxplots(T, cfg)
    if ~exist(cfg.figDir, 'dir')
        mkdir(cfg.figDir);
    end

    metricSpecs = {
        'envelopeMinClear', 'Envelope clearance', 'clearance', 'envelope_clearance_boxplot.jpg'
        'pathLength',       'Path length',         'length',    'path_length_boxplot.jpg'
        'turnAbsSum',       'Path turning',        'turn sum',  'turning_boxplot.jpg'
        'totalTimeSec',     'Runtime',             'time (s)',  'runtime_boxplot.jpg'
    };

    for i = 1:size(metricSpecs, 1)
        plotMetricBoxplot(T, cfg, ...
            metricSpecs{i, 1}, metricSpecs{i, 2}, ...
            metricSpecs{i, 3}, metricSpecs{i, 4});
    end

    plotSuccessRateBars(T, cfg);
end

function plotMetricBoxplot(T, cfg, metricName, figTitle, yLabelText, fileName)
    sceneIds = unique(T.sceneId, 'stable');
    fig = figure('Color','w', 'Name', figTitle, 'Position',[80 80 1200 360]);
    tiledlayout(1, numel(sceneIds), 'Padding','compact', 'TileSpacing','compact');

    for i = 1:numel(sceneIds)
        nexttile;
        rows = T(strcmp(T.sceneId, sceneIds(i)), :);
        y = rows.(metricName);
        valid = isfinite(y);

        if any(valid)
            groups = cellstr(rows.planner(valid));
            groupOrder = cfg.plannerNames(ismember(cfg.plannerNames, unique(groups, 'stable')));
            boxplot(y(valid), groups, 'GroupOrder', groupOrder, 'Symbol','k.');
            grid on;
        else
            text(0.5, 0.5, 'no valid data', ...
                'HorizontalAlignment','center', 'FontWeight','bold');
            axis off;
        end

        title(char(sceneIds(i)), 'Interpreter','none');
        ylabel(yLabelText);
    end

    sgtitle(figTitle, 'Interpreter','none');
    exportgraphics(fig, fullfile(cfg.figDir, fileName), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    savefig(fig, fullfile(cfg.figDir, strrep(fileName, '.jpg', '.fig')));
    close(fig);
end

function plotSuccessRateBars(T, cfg)
    sceneIds = unique(T.sceneId, 'stable');
    planners = string(cfg.plannerNames);
    plannerSuccess = nan(numel(sceneIds), numel(planners));
    envelopeSuccess = nan(numel(sceneIds), numel(planners));

    for i = 1:numel(sceneIds)
        for j = 1:numel(planners)
            rows = T(strcmp(T.sceneId, sceneIds(i)) & strcmp(T.planner, planners(j)), :);
            n = height(rows);
            plannerSuccess(i,j) = safeRate(rows.plannerSuccess, n);
            envelopeSuccess(i,j) = safeRate(rows.envelopeSuccess, n);
        end
    end

    fig = figure('Color','w', 'Name','Success rates', 'Position',[80 80 1100 420]);
    tiledlayout(1, 2, 'Padding','compact', 'TileSpacing','compact');

    nexttile;
    bar(plannerSuccess);
    title('Planner success');
    ylabel('rate');
    ylim([0, 1]);
    set(gca, 'XTickLabel', cellstr(sceneIds), 'TickLabelInterpreter','none');
    legend(cellstr(planners), 'Location','southoutside', 'Orientation','horizontal');
    grid on;

    nexttile;
    bar(envelopeSuccess);
    title('Envelope success');
    ylabel('rate');
    ylim([0, 1]);
    set(gca, 'XTickLabel', cellstr(sceneIds), 'TickLabelInterpreter','none');
    legend(cellstr(planners), 'Location','southoutside', 'Orientation','horizontal');
    grid on;

    exportgraphics(fig, fullfile(cfg.figDir, 'success_rate_bars.jpg'), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    savefig(fig, fullfile(cfg.figDir, 'success_rate_bars.fig'));
    close(fig);
end

function printOneRun(result, caseId, totalRuns)
    if result.plannerSuccess
        fprintf('%4d/%4d | %-7s | %-19s | pointClr=%+.4f envClr=%+.4f | len=%.3f turn=%.3f | t=%.3fs\n', ...
            caseId, totalRuns, char(result.planner), char(result.sceneId), ...
            result.pointMinClear, result.envelopeMinClear, ...
            result.pathLength, result.turnAbsSum, result.planTimeSec);
    else
        msg = char(result.plannerMessage);
        if strlength(result.errorMessage) > 0
            msg = char(result.errorMessage);
        end
        fprintf('%4d/%4d | %-7s | %-19s | failed | %s\n', ...
            caseId, totalRuns, char(result.planner), char(result.sceneId), msg);
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
    fprintf(fid, 'RRT vs RRT* vs CSSC baseline metric definitions\n\n');
    fprintf(fid, 'plannerSuccess: for RRT/RRT*, planner returned a point collision-free path; for CSSC, frontend RRT and backend optimization returned a path.\n');
    fprintf(fid, 'pointMinClear: high-resolution sampled centerline point SDF minimum.\n');
    fprintf(fid, 'pointSuccess: pointMinClear > 0.\n');
    fprintf(fid, 'envelopeMinClear: high-precision CSSC fixed-chord envelope clearance from evaluateCSSCHighPrecision. RRT/RRT* are evaluated after B-spline conversion; CSSC is evaluated after optimization.\n');
    fprintf(fid, 'envelopeSuccess: envelopeMinClear > 0.\n');
    fprintf(fid, 'envelopeDMinSatisfied: envelopeMinClear >= %.6f.\n', cfg.dMin);
    fprintf(fid, 'pathLength: RRT/RRT* raw planner polyline length; CSSC optimized sampled B-spline length.\n');
    fprintf(fid, 'turnAbsSum: sum of absolute heading changes along the reported path.\n');
    fprintf(fid, 'turnSqSum: sum of squared heading changes along the reported path.\n');
    fprintf(fid, 'planTimeSec: method runtime excluding final metric evaluation; CSSC includes frontend, initialization, and optimization.\n');
    fprintf(fid, 'frontendTimeSec: CSSC frontend RRT time when applicable.\n');
    fprintf(fid, 'initTimeSec: CSSC shortcut and B-spline initialization time when applicable.\n');
    fprintf(fid, 'optTimeSec: CSSC optimization time when applicable.\n');
    fprintf(fid, 'evalTimeSec: time spent evaluating point and envelope clearance metrics.\n');
    fprintf(fid, 'totalTimeSec: planner plus metric evaluation and overhead.\n');
    fprintf(fid, 'Figures are saved under: %s\n', cfg.figDir);
end
