clear; clc; close all;

%% Batch experiment: structured scene 3 S-channel
% Runs four-rectangle S-channel cases over several dGap values and random
% environment seeds. Results are saved under results/runs/run_yyyymmdd_HHMMSS.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeExperimentConfig(projectRoot);
paramsBase = makeBatchOptimizationParams();

if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

fprintf('\n[scene 3 S-channel batch experiment]\n');
fprintf('  output dir : %s\n', cfg.outDir);
fprintf('  dGap list  : %s\n', mat2str(cfg.dGapList, 4));
fprintf('  seeds      : %d seeds, %d to %d\n', ...
    numel(cfg.seedList), cfg.seedList(1), cfg.seedList(end));
fprintf('  total runs : %d\n\n', numel(cfg.dGapList) * numel(cfg.seedList));

totalRuns = numel(cfg.dGapList) * numel(cfg.seedList);
trialResults = repmat(emptyTrialResult(), totalRuns, 1);

caseId = 0;
tExperiment = tic;

for ig = 1:numel(cfg.dGapList)
    dGap = cfg.dGapList(ig);
    fprintf('\n[dGap %.3f] %d seeds\n', dGap, numel(cfg.seedList));

    for iseed = 1:numel(cfg.seedList)
        caseId = caseId + 1;
        envSeed = cfg.seedList(iseed);
        tCase = tic;

        result = emptyTrialResult();
        result.caseId = caseId;
        result.dGap = dGap;
        result.seedIndex = iseed;
        result.envSeed = envSeed;
        result.rrtSeed = cfg.rrtSeedBase + 1000 * ig + iseed;
        result.shortcutSeed = cfg.shortcutSeedBase + 1000 * ig + iseed;

        try
            result = runOneTrial(result, cfg, paramsBase);
        catch ME
            result.errorMessage = string(ME.message);
        end

        result.elapsedSec = toc(tCase);
        trialResults(caseId) = result;

        if cfg.printEachTrial
            printTrialProgress(result, caseId, totalRuns);
        end

        if cfg.savePartial && (mod(caseId, cfg.saveEveryN) == 0 || caseId == totalRuns)
            trialTablePartial = struct2table(trialResults(1:caseId));
            writetable(trialTablePartial, fullfile(cfg.outDir, 'trial_results_partial.csv'));
            save(fullfile(cfg.outDir, 'partial_results.mat'), ...
                'cfg', 'paramsBase', 'trialResults', 'caseId');
        end
    end

    trialTableNow = struct2table(trialResults(1:caseId));
    summaryNow = summarizeTrials(trialTableNow, cfg);
    disp(summaryNow);
end

elapsedTotalSec = toc(tExperiment);
trialTable = struct2table(trialResults);
summaryTable = summarizeTrials(trialTable, cfg);

writetable(trialTable, fullfile(cfg.outDir, 'trial_results.csv'));
writetable(summaryTable, fullfile(cfg.outDir, 'summary_by_dgap.csv'));
save(fullfile(cfg.outDir, 'experiment_results.mat'), ...
    'cfg', 'paramsBase', 'trialResults', 'trialTable', ...
    'summaryTable', 'elapsedTotalSec');
writeExperimentSummaryText(cfg, paramsBase, summaryTable, elapsedTotalSec);

fprintf('\n[final summary by dGap]\n');
disp(summaryTable);
fprintf('[saved] %s\n', cfg.outDir);
fprintf('[elapsed] %.2f min\n', elapsedTotalSec / 60);

%% Configuration

function cfg = makeExperimentConfig(projectRoot)
    cfg = struct();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);

    cfg.dGapList = 0.06:0.02:0.14;
    cfg.seedList = 1101:1200;

    cfg.bounds = [0, 1; -0.4, 0.4];
    cfg.startPt = [0.05, 0.0];
    cfg.goalPt = [0.95, 0.0];
    cfg.channelW = 0.64;
    cfg.channelH = cfg.bounds(2,2) - cfg.bounds(2,1);
    cfg.yInOutRange = [-0.20, 0.20];

    cfg.rrtSeedBase = 300000;
    cfg.shortcutSeedBase = 700000;
    cfg.suppressOptimizerPrint = true;
    cfg.printEachTrial = true;
    cfg.savePartial = true;
    cfg.saveEveryN = 10;
end

function params = makeBatchOptimizationParams()
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
    params.timingPrintInterval = 20;
    params.timingPrintWindow = 5;
end

%% One trial

function result = runOneTrial(result, cfg, paramsBase)
    envOpts = struct();
    envOpts.bounds = cfg.bounds;
    envOpts.startPt = cfg.startPt;
    envOpts.goalPt = cfg.goalPt;
    envOpts.dGap = result.dGap;
    envOpts.W = cfg.channelW;
    envOpts.H = cfg.channelH;
    envOpts.seed = result.envSeed;
    envOpts.yInOutRange = cfg.yInOutRange;

    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('fourRectSChannel', envOpts);
    envInfo.obstacles = obstacles;
    result.envOk = true;
    result.numObstacles = numel(obstacles);
    result.L1 = envInfo.params.L1;
    result.L2 = envInfo.params.L2;
    result.L3 = envInfo.params.L3;
    result.yIn = envInfo.params.yIn;
    result.yOut = envInfo.params.yOut;
    result.swappedByL2 = envInfo.params.swappedByL2;

    rrtOpts = makeBatchRRTOpts(envInfo, result.rrtSeed);
    [pathRRT, rrtInfo] = runRRTWithRestarts(envInfo, rrtOpts);
    result.rrtSuccess = rrtInfo.success;
    result.rrtNumIter = rrtInfo.numIter;
    result.rrtNumNodes = rrtInfo.numNodes;
    result.rrtRestartUsed = getFieldOrDefault(rrtInfo, 'numRestartUsed', nan);
    result.rrtPathLength = getFieldOrDefault(rrtInfo, 'pathLength', nan);

    if ~rrtInfo.success
        result.errorMessage = string(rrtInfo.message);
        return;
    end

    shortcutOpts = rrtOpts;
    shortcutOpts.seed = result.shortcutSeed;
    shortcutOpts.numShortcut = 200;
    [pathShort, shortcutInfo] = shortcutPath2D(pathRRT, obstacles, shortcutOpts);
    result.shortcutInitialNodes = shortcutInfo.initialNodes;
    result.shortcutFinalNodes = shortcutInfo.finalNodes;
    result.shortcutAccepted = shortcutInfo.numAccepted;

    params = paramsBase;
    approxLen = polylineLength(pathShort);
    nCtrl = max(params.degree + 1, ceil(approxLen / (0.5 * params.L)) + 1);
    splineOpts = struct('degree', params.degree, 'nCtrl', nCtrl);
    [Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, splineOpts);
    Pref = Pinit;
    params.knot = splineInfo.knot;
    result.nCtrl = size(Pinit, 1);

    stateInit = evaluateCSSCGlobal(Pinit, obstacles, params);
    result.initEvaluated = true;
    result.initMinClear = stateInit.minClear;
    result.initSuccess = isfinite(stateInit.minClear) && stateInit.minClear > 0;
    result.initDMinSatisfied = isfinite(stateInit.minClear) && stateInit.minClear >= params.dMin;

    try
        [Popt, optInfo, optimizerLog] = runOptimizer(Pinit, Pref, obstacles, params, cfg.suppressOptimizerPrint);
    catch ME
        result.errorMessage = string(ME.message);
        return;
    end

    result.optimizerLogChars = numel(char(optimizerLog));
    result.bestJ = optInfo.bestJ;
    result.finalJ = optInfo.finalJ;
    result.bestIter = optInfo.bestIter;
    result.numIterActual = optInfo.numIterActual;
    result.stopReason = string(optInfo.stopReason);
    result.returnedBest = getFieldOrDefault(optInfo, 'returnedBest', true);

    try
        stateOpt = evaluateCSSCGlobal(Popt, obstacles, params);
    catch ME
        result.errorMessage = string(ME.message);
        return;
    end

    result.optSuccess = true;
    result.finalMinClear = stateOpt.minClear;
    result.clearanceGain = result.finalMinClear - result.initMinClear;
    result.finalSuccess = isfinite(stateOpt.minClear) && stateOpt.minClear > 0;
    result.finalDMinSatisfied = isfinite(stateOpt.minClear) && stateOpt.minClear >= params.dMin;
end

function rrtOpts = makeBatchRRTOpts(envInfo, seed)
    rrtOpts = struct();
    rrtOpts.bounds = envInfo.bounds;
    rrtOpts.stepSize = 0.025;
    rrtOpts.goalBias = 0.16;
    rrtOpts.goalTol = 0.035;
    rrtOpts.maxIter = 8000;
    rrtOpts.collisionResolution = 0.003;
    rrtOpts.inflateRadius = 0.0;
    rrtOpts.seed = seed;
    rrtOpts.numRestart = 12;
end

function [Popt, optInfo, optimizerLog] = runOptimizer(Pinit, Pref, obstacles, params, suppressPrint)
    optimizerLog = "";
    if suppressPrint
        Popt = [];
        optInfo = [];
        optimizerLog = evalc('[Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, params);');
    else
        [Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, params);
    end
end

function [path, info] = runRRTWithRestarts(envInfo, rrtOpts)
    path = zeros(0, 2);
    info = struct('success', false, 'message', 'RRT failed.', ...
        'numIter', 0, 'numNodes', 0, 'numRestartUsed', 0, 'pathLength', nan);

    for trial = 1:rrtOpts.numRestart
        optsTrial = rrtOpts;
        optsTrial.seed = rrtOpts.seed + trial - 1;
        [pathTrial, infoTrial] = planRRT2D( ...
            envInfo.startPt, envInfo.goalPt, envInfo.obstacles, optsTrial);

        infoTrial.numRestartUsed = trial;
        info = infoTrial;
        if infoTrial.success
            path = pathTrial;
            return;
        end
    end
end

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

%% Result tables

function result = emptyTrialResult()
    result = struct();
    result.caseId = 0;
    result.dGap = nan;
    result.seedIndex = 0;
    result.envSeed = 0;
    result.rrtSeed = 0;
    result.shortcutSeed = 0;

    result.envOk = false;
    result.rrtSuccess = false;
    result.initEvaluated = false;
    result.optSuccess = false;
    result.initSuccess = false;
    result.finalSuccess = false;
    result.initDMinSatisfied = false;
    result.finalDMinSatisfied = false;
    result.returnedBest = false;

    result.initMinClear = nan;
    result.finalMinClear = nan;
    result.clearanceGain = nan;
    result.bestJ = nan;
    result.finalJ = nan;
    result.bestIter = nan;
    result.numIterActual = nan;
    result.stopReason = "";

    result.numObstacles = nan;
    result.nCtrl = nan;
    result.L1 = nan;
    result.L2 = nan;
    result.L3 = nan;
    result.yIn = nan;
    result.yOut = nan;
    result.swappedByL2 = false;

    result.rrtNumIter = nan;
    result.rrtNumNodes = nan;
    result.rrtRestartUsed = nan;
    result.rrtPathLength = nan;
    result.shortcutInitialNodes = nan;
    result.shortcutFinalNodes = nan;
    result.shortcutAccepted = nan;

    result.optimizerLogChars = nan;
    result.elapsedSec = nan;
    result.errorMessage = "";
end

function summaryTable = summarizeTrials(trialTable, cfg)
    summary = repmat(emptySummaryRow(), numel(cfg.dGapList), 1);

    for i = 1:numel(cfg.dGapList)
        dGap = cfg.dGapList(i);
        mask = abs(trialTable.dGap - dGap) < 1e-12;
        rows = trialTable(mask, :);
        nTrial = height(rows);

        summary(i).dGap = dGap;
        summary(i).nTrial = nTrial;
        summary(i).envOk = sum(rows.envOk);
        summary(i).rrtSuccess = sum(rows.rrtSuccess);
        summary(i).optSuccess = sum(rows.optSuccess);
        summary(i).initSuccessRate = safeRate(rows.initSuccess, nTrial);
        summary(i).finalSuccessRate = safeRate(rows.finalSuccess, nTrial);
        summary(i).successRateGain = summary(i).finalSuccessRate - summary(i).initSuccessRate;
        summary(i).initDMinRate = safeRate(rows.initDMinSatisfied, nTrial);
        summary(i).finalDMinRate = safeRate(rows.finalDMinSatisfied, nTrial);
        summary(i).initClearMean = safeMean(rows.initMinClear);
        summary(i).finalClearMean = safeMean(rows.finalMinClear);
        summary(i).clearGainMean = safeMean(rows.clearanceGain);
        summary(i).initClearMedian = safeMedian(rows.initMinClear);
        summary(i).finalClearMedian = safeMedian(rows.finalMinClear);
        summary(i).initClearMin = safeMin(rows.initMinClear);
        summary(i).finalClearMin = safeMin(rows.finalMinClear);
        summary(i).bestIterMean = safeMean(rows.bestIter);
        summary(i).runIterMean = safeMean(rows.numIterActual);
        summary(i).elapsedMeanSec = safeMean(rows.elapsedSec);
    end

    summaryTable = struct2table(summary);
end

function row = emptySummaryRow()
    row = struct();
    row.dGap = nan;
    row.nTrial = 0;
    row.envOk = 0;
    row.rrtSuccess = 0;
    row.optSuccess = 0;
    row.initSuccessRate = nan;
    row.finalSuccessRate = nan;
    row.successRateGain = nan;
    row.initDMinRate = nan;
    row.finalDMinRate = nan;
    row.initClearMean = nan;
    row.finalClearMean = nan;
    row.clearGainMean = nan;
    row.initClearMedian = nan;
    row.finalClearMedian = nan;
    row.initClearMin = nan;
    row.finalClearMin = nan;
    row.bestIterMean = nan;
    row.runIterMean = nan;
    row.elapsedMeanSec = nan;
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

function m = safeMedian(x)
    x = x(isfinite(x));
    if isempty(x)
        m = nan;
    else
        m = median(x);
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

function printTrialProgress(result, caseId, totalRuns)
    if result.optSuccess
        fprintf('%4d/%4d | dGap=%.3f seed=%d | init=%.5f final=%.5f gain=%+.5f | bestIter=%3d runIter=%3d | %s\n', ...
            caseId, totalRuns, result.dGap, result.envSeed, ...
            result.initMinClear, result.finalMinClear, result.clearanceGain, ...
            result.bestIter, result.numIterActual, char(result.stopReason));
    elseif result.rrtSuccess
        fprintf('%4d/%4d | dGap=%.3f seed=%d | opt failed | %s\n', ...
            caseId, totalRuns, result.dGap, result.envSeed, char(result.errorMessage));
    else
        fprintf('%4d/%4d | dGap=%.3f seed=%d | RRT failed | %s\n', ...
            caseId, totalRuns, result.dGap, result.envSeed, char(result.errorMessage));
    end
end

function writeExperimentSummaryText(cfg, paramsBase, summaryTable, elapsedTotalSec)
    filePath = fullfile(cfg.outDir, 'summary.txt');
    fid = fopen(filePath, 'w');
    if fid < 0
        warning('Could not write summary file: %s', filePath);
        return;
    end

    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'Scene 3 S-channel batch experiment\n');
    fprintf(fid, 'Run name: %s\n', cfg.runName);
    fprintf(fid, 'dGap list: %s\n', mat2str(cfg.dGapList, 4));
    fprintf(fid, 'seed list: %d to %d (%d seeds)\n', ...
        cfg.seedList(1), cfg.seedList(end), numel(cfg.seedList));
    fprintf(fid, 'success criterion: minClear > 0\n');
    fprintf(fid, 'dMin target: %.6f\n', paramsBase.dMin);
    fprintf(fid, 'elapsed: %.2f min\n\n', elapsedTotalSec / 60);

    fprintf(fid, '%s\n', evalc('disp(summaryTable)'));
end
