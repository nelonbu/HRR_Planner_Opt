clear; clc; close all;

%% Bounding-circle obstacle filtering demo
% Compare segment clearance with and without broad-phase obstacle filtering.
% The filtered version orders obstacles by bounding-circle lower bounds and
% stops exact checks once the remaining obstacles cannot improve the current
% best clearance.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeConfig(projectRoot);
if cfg.saveResults
    if ~exist(cfg.outDir, 'dir'); mkdir(cfg.outDir); end
    if ~exist(cfg.figDir, 'dir'); mkdir(cfg.figDir); end
end

fprintf('\n[obstacle filter comparison]\n');
fprintf('  output dir     : %s\n', cfg.outDir);
fprintf('  num scenes     : %d\n', cfg.numScenes);
fprintf('  repeats / mode : %d\n', cfg.numTimingRepeats);
fprintf('  nU             : %d\n\n', cfg.nU);

results = repmat(emptyResult(), cfg.numScenes, 1);
sceneData = cell(cfg.numScenes, 1);
tAll = tic;

for icase = 1:cfg.numScenes
    sceneType = cfg.sceneTypes{mod(icase - 1, numel(cfg.sceneTypes)) + 1};
    seed = cfg.seedBase + icase;

    try
        [obstacles, envInfo, P, paramsBase] = makeSceneCase(cfg, sceneType, seed);

        paramsOff = paramsBase;
        paramsOff.obstacleFilter.enable = false;

        paramsOn = paramsBase;
        paramsOn.obstacleFilter.enable = true;

        [stateOff, timeOffMs] = evalRepeated(P, obstacles, paramsOff, cfg.numTimingRepeats);
        [stateOn, timeOnMs] = evalRepeated(P, obstacles, paramsOn, cfg.numTimingRepeats);

        results(icase) = summarizeCase(icase, sceneType, seed, numel(obstacles), ...
            stateOff, stateOn, timeOffMs, timeOnMs);

        sceneData{icase} = struct( ...
            'sceneType', sceneType, ...
            'seed', seed, ...
            'obstacles', obstacles, ...
            'envInfo', envInfo, ...
            'P', P, ...
            'paramsBase', paramsBase, ...
            'stateOff', stateOff, ...
            'stateOn', stateOn, ...
            'timeOffMs', timeOffMs, ...
            'timeOnMs', timeOnMs);

    catch ME
        warning('Case %d failed (%s, seed=%d): %s', ...
            icase, sceneType, seed, ME.message);
        results(icase) = emptyResult();
        results(icase).caseId = icase;
        results(icase).sceneType = string(sceneType);
        results(icase).seed = seed;
        results(icase).errorMessage = string(ME.message);
    end

    if cfg.printEvery > 0 && (mod(icase, cfg.printEvery) == 0 || icase == cfg.numScenes)
        fprintf('  finished %d / %d cases\n', icase, cfg.numScenes);
    end
end

elapsedSec = toc(tAll);
caseTable = struct2table(results);
statsAll = makeStats(caseTable, "all");
statsByScene = makeStatsByScene(caseTable);

fprintf('\n[overall stats]\n');
disp(statsAll);
fprintf('\n[stats by scene type]\n');
disp(statsByScene);
fprintf('[elapsed] %.2f s\n', elapsedSec);

if cfg.saveResults
    writetable(caseTable, fullfile(cfg.outDir, 'obstacle_filter_case_results.csv'));
    writetable(statsAll, fullfile(cfg.outDir, 'obstacle_filter_stats_all.csv'));
    writetable(statsByScene, fullfile(cfg.outDir, 'obstacle_filter_stats_by_scene.csv'));
    save(fullfile(cfg.outDir, 'obstacle_filter_results.mat'), ...
        'cfg', 'caseTable', 'statsAll', 'statsByScene', 'sceneData', 'elapsedSec');
    plotSummary(caseTable, cfg);
    writeNotes(cfg);
    fprintf('[saved] %s\n', cfg.outDir);
end

%% Configuration

function cfg = makeConfig(projectRoot)
    cfg = struct();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_obstacle_filter_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.saveResults = true;

    cfg.numScenes = 100;
    cfg.numTimingRepeats = 3;
    cfg.printEvery = 10;
    cfg.seedBase = 31000;

    cfg.sceneTypes = { ...
        'randomCirclesDense', ...
        'randomRectsDense', ...
        'randomMixedDense', ...
        'offsetDoubleSlit', ...
        'fourRectSChannel'};

    cfg.bounds = [0, 1; -0.4, 0.4];
    cfg.startPt = [0.05, 0.0];
    cfg.goalPt = [0.95, 0.0];

    cfg.L = 0.15;
    cfg.dMin = 0.02;
    cfg.dPref = 0.03;
    cfg.degree = 3;
    cfg.nU = 180;
    cfg.pathSampleN = 400;
    cfg.figureResolution = 300;
end

function [obstacles, envInfo, P, params] = makeSceneCase(cfg, sceneType, seed)
    opts = baseEnvOpts(cfg);

    switch lower(sceneType)
        case 'randomcirclesdense'
            opts.seed = seed;
            opts.nObs = 22;
            opts.radiusRange = [0.012, 0.030];
            opts.minGap = 0.006;
            [obstacles, envInfo] = generateCSSCEnvironment2D('randomCircles', opts);

        case 'randomrectsdense'
            opts.seed = seed;
            opts.nObs = 22;
            opts.halfSizeXRange = [0.012, 0.045];
            opts.halfSizeYRange = [0.014, 0.060];
            opts.yawRange = [-pi/3, pi/3];
            opts.minGap = 0.006;
            [obstacles, envInfo] = generateCSSCEnvironment2D('randomRects', opts);

        case 'randommixeddense'
            opts.seed = seed;
            opts.nCircle = 12;
            opts.nRect = 12;
            opts.radiusRange = [0.012, 0.028];
            opts.halfSizeXRange = [0.012, 0.040];
            opts.halfSizeYRange = [0.014, 0.055];
            opts.yawRange = [-pi/3, pi/3];
            opts.minGap = 0.006;
            [obstacles, envInfo] = generateCSSCEnvironment2D('randomMixed', opts);

        case 'offsetdoubleslit'
            opts.xWalls = [0.32, 0.68];
            opts.gapCentersY = [-0.12, 0.14];
            opts.gapHeight = 0.10;
            opts.wallThickness = 0.08;
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
                'offsetDoubleSlit', opts);

        case 'fourrectschannel'
            opts.seed = seed;
            opts.dGap = 0.08 + 0.05*randUnit(seed, 1);
            opts.channelW = 0.62;
            opts.channelH = cfg.bounds(2,2) - cfg.bounds(2,1);
            opts.yInOutRange = [-0.20, 0.20];
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
                'fourRectSChannel', opts);

        otherwise
            error('Unknown sceneType: %s', sceneType);
    end

    P = makePath(cfg, envInfo, sceneType, seed);
    params = makeParams(cfg, P);
end

function opts = baseEnvOpts(cfg)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    opts.maxTry = 6000;
    opts.keepoutStart = 0.08;
    opts.keepoutGoal = 0.08;
end

function P = makePath(cfg, envInfo, sceneType, seed)
    startPt = envInfo.startPt;
    goalPt = envInfo.goalPt;

    switch lower(sceneType)
        case 'offsetdoubleslit'
            waypoints = [
                startPt
                0.20, 0.00
                envInfo.params.xWalls(1), envInfo.params.gapCentersY(1)
                0.50, 0.00
                envInfo.params.xWalls(2), envInfo.params.gapCentersY(2)
                0.82, 0.02
                goalPt
            ];

        case 'fourrectschannel'
            yIn = envInfo.params.yIn;
            yOut = envInfo.params.yOut;
            waypoints = [
                startPt
                0.20, yIn
                0.38, yIn
                0.52, 0.5*(yIn + yOut)
                0.66, yOut
                0.82, yOut
                goalPt
            ];

        otherwise
            x = linspace(startPt(1), goalPt(1), 9).';
            s = (x - x(1)) / max(eps, x(end) - x(1));
            amp = 0.08 + 0.04*randUnit(seed, 1);
            phase = 2*pi*randUnit(seed, 2);
            y = amp*sin(2*pi*s + phase) + 0.035*sin(4*pi*s + 0.5*phase);
            waypoints = [x, y];
            waypoints(1,:) = startPt;
            waypoints(end,:) = goalPt;
    end

    P = resamplePolylineLocal(waypoints, 9);
    P(1,:) = startPt;
    P(end,:) = goalPt;
end

function params = makeParams(cfg, P)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;
    params.knot = makeClampedUniformKnot(size(P, 1), params.degree);
    params.clearanceMode = 'segment';

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = cfg.nU;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;
    params.envOpts.lambdaTol = 1e-9;
    params.envOpts.maxNewtonIter = 6;
    params.envOpts.vResidualTol = 1e-5;
    params.envOpts.vAcceptTol = 5e-4;
    params.envOpts.fallbackNGrid = 30;
    params.envOpts.enableFallback = true;

    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
    params.printEvalTiming = false;
    params.pathSampleN = cfg.pathSampleN;

    params.obstacleFilter = struct();
    params.obstacleFilter.enable = false;
    params.obstacleFilter.useForSegment = true;
    params.obstacleFilter.useForPoint = true;
    params.obstacleFilter.minCandidates = 1;
    params.obstacleFilter.stopTol = 1e-12;
end

%% Evaluation and summaries

function [state, samplesMs] = evalRepeated(P, obstacles, params, repeats)
    evaluateCSSCGlobal(P, obstacles, params);
    samplesMs = nan(repeats, 1);
    for r = 1:repeats
        state = evaluateCSSCGlobal(P, obstacles, params);
        samplesMs(r) = 1000 * state.timing.total;
    end
end

function result = summarizeCase(caseId, sceneType, seed, nObs, stateOff, stateOn, timeOffMs, timeOnMs)
    result = emptyResult();
    result.caseId = caseId;
    result.sceneType = string(sceneType);
    result.seed = seed;
    result.numObstacles = nObs;

    result.minClearOff = stateOff.minClear;
    result.minClearOn = stateOn.minClear;
    result.absClearDiff = abs(stateOn.minClear - stateOff.minClear);
    result.signedClearDiff = stateOn.minClear - stateOff.minClear;

    result.timeOffMs = finiteMean(timeOffMs);
    result.timeOnMs = finiteMean(timeOnMs);
    result.timeDiffMs = result.timeOnMs - result.timeOffMs;
    result.timeSavingMs = result.timeOffMs - result.timeOnMs;
    result.speedup = result.timeOffMs / result.timeOnMs;

    fsOff = stateOff.timing.obstacleFilter;
    fsOn = stateOn.timing.obstacleFilter;
    result.checkFractionOff = fsOff.checkFraction;
    result.checkFractionOn = fsOn.checkFraction;
    result.segmentChecksOff = fsOff.segmentObstacleChecks;
    result.segmentChecksOn = fsOn.segmentObstacleChecks;
    result.segmentChecksPossible = fsOn.segmentObstaclePossible;
end

function stats = makeStats(caseTable, groupName)
    valid = caseTable.errorMessage == "";
    T = caseTable(valid,:);
    row = emptyStatsRow();
    row.group = string(groupName);
    row.numCases = height(T);
    row.absClearDiffMin = finiteMin(T.absClearDiff);
    row.absClearDiffMax = finiteMax(T.absClearDiff);
    row.absClearDiffMean = finiteMean(T.absClearDiff);
    row.signedClearDiffMean = finiteMean(T.signedClearDiff);
    row.timeSavingMsMin = finiteMin(T.timeSavingMs);
    row.timeSavingMsMax = finiteMax(T.timeSavingMs);
    row.timeSavingMsMean = finiteMean(T.timeSavingMs);
    row.timeDiffMsMean = finiteMean(T.timeDiffMs);
    row.speedupMean = finiteMean(T.speedup);
    row.checkFractionOffMean = finiteMean(T.checkFractionOff);
    row.checkFractionOnMean = finiteMean(T.checkFractionOn);
    stats = struct2table(row);
end

function stats = makeStatsByScene(caseTable)
    valid = caseTable.errorMessage == "";
    sceneTypes = unique(caseTable.sceneType(valid), 'stable');
    rows = repmat(emptyStatsRow(), numel(sceneTypes), 1);
    for i = 1:numel(sceneTypes)
        Tscene = caseTable(valid & caseTable.sceneType == sceneTypes(i), :);
        rows(i) = table2struct(makeStats(Tscene, sceneTypes(i)));
    end
    stats = struct2table(rows);
end

function result = emptyResult()
    result = struct();
    result.caseId = nan;
    result.sceneType = "";
    result.seed = nan;
    result.numObstacles = nan;
    result.minClearOff = nan;
    result.minClearOn = nan;
    result.absClearDiff = nan;
    result.signedClearDiff = nan;
    result.timeOffMs = nan;
    result.timeOnMs = nan;
    result.timeDiffMs = nan;
    result.timeSavingMs = nan;
    result.speedup = nan;
    result.checkFractionOff = nan;
    result.checkFractionOn = nan;
    result.segmentChecksOff = nan;
    result.segmentChecksOn = nan;
    result.segmentChecksPossible = nan;
    result.errorMessage = "";
end

function row = emptyStatsRow()
    row = struct();
    row.group = "";
    row.numCases = 0;
    row.absClearDiffMin = nan;
    row.absClearDiffMax = nan;
    row.absClearDiffMean = nan;
    row.signedClearDiffMean = nan;
    row.timeSavingMsMin = nan;
    row.timeSavingMsMax = nan;
    row.timeSavingMsMean = nan;
    row.timeDiffMsMean = nan;
    row.speedupMean = nan;
    row.checkFractionOffMean = nan;
    row.checkFractionOnMean = nan;
end

%% Plotting and utilities

function plotSummary(caseTable, cfg)
    valid = caseTable.errorMessage == "";
    T = caseTable(valid,:);

    fig = figure('Color','w', 'Position', [100, 100, 860, 360]);
    tiledlayout(fig, 1, 3, 'TileSpacing','compact', 'Padding','compact');

    nexttile;
    boxplot(T.absClearDiff, categorical(T.sceneType), 'Symbol','k.');
    ylabel('|minClear filtered - unfiltered|');
    title('Clearance difference');
    ylim([-0.01 0.01])
    grid on;

    nexttile;
    boxplot(T.timeSavingMs, categorical(T.sceneType), 'Symbol','k.');
    ylabel('time saving (ms)');
    title('Runtime saving');
    grid on;

    nexttile;
    boxplot(T.checkFractionOn, categorical(T.sceneType), 'Symbol','k.');
    ylabel('checked / possible');
    title('Obstacle check fraction');
    grid on;

    exportgraphics(fig, fullfile(cfg.figDir, 'obstacle_filter_boxplots.jpg'), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    close(fig);
end

function writeNotes(cfg)
    fid = fopen(fullfile(cfg.outDir, 'obstacle_filter_notes.txt'), 'w');
    if fid < 0
        return;
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'Bounding-circle obstacle filter demo\n\n');
    fprintf(fid, 'The filtered mode uses exact branch-and-bound over obstacle bounding circles.\n');
    fprintf(fid, 'A zero absClearDiff means the filter preserved the unfiltered segment minClear.\n');
    fprintf(fid, 'timeSavingMs = unfiltered time - filtered time. Positive means filtering is faster.\n');
    fprintf(fid, 'checkFractionOn is the fraction of exact obstacle checks after filtering.\n');
end

function val = finiteMean(x)
    x = x(isfinite(x));
    if isempty(x); val = nan; else; val = mean(x); end
end

function val = finiteMin(x)
    x = x(isfinite(x));
    if isempty(x); val = nan; else; val = min(x); end
end

function val = finiteMax(x)
    x = x(isfinite(x));
    if isempty(x); val = nan; else; val = max(x); end
end

function p = resamplePolylineLocal(path, n)
    segLen = vecnorm(diff(path, 1, 1), 2, 2);
    s = [0; cumsum(segLen)];
    if s(end) <= eps
        p = repmat(path(1,:), n, 1);
        return;
    end
    p = interp1(s, path, linspace(0, s(end), n).', 'linear');
end

function r = randUnit(seed, offset)
    r = 0.5 + 0.5 * sin(12.9898 * (seed + 37 * offset) + 78.233);
    r = r - floor(r);
end
