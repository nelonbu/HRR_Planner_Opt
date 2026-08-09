clear; clc; close all;

%% Batch clearance backend comparison: segment vs envelope vs hybrid
% This demo evaluates many random/structured 2D scenes. For each scene, the
% exact segment mode is used as the reference, and the script reports how
% envelope and hybrid differ in minClear and runtime.
%
% Outputs:
%   results/runs/run_clearance_modes_batch_*/clearance_mode_case_results.csv
%   results/runs/run_clearance_modes_batch_*/clearance_mode_stats_by_mode.csv
%   results/runs/run_clearance_modes_batch_*/clearance_mode_stats_by_scene.csv
%   results/runs/run_clearance_modes_batch_*/clearance_mode_batch_results.mat

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

fprintf('\n[batch clearance mode comparison]\n');
fprintf('  output dir      : %s\n', cfg.outDir);
fprintf('  num scenes      : %d\n', cfg.numScenes);
fprintf('  repeats / mode  : %d\n', cfg.numTimingRepeats);
fprintf('  nU              : %d\n', cfg.nU);
fprintf('  scene categories: %s\n\n', strjoin(cfg.sceneTypes, ', '));

modes = {'segment', 'envelope', 'hybrid'};
caseResults = repmat(emptyCaseResult(), cfg.numScenes, 1);
modeResults = repmat(emptyModeResult(), cfg.numScenes * numel(modes), 1);
sceneData = cell(cfg.numScenes, 1);

row = 0;
tAll = tic;

for icase = 1:cfg.numScenes
    sceneType = cfg.sceneTypes{mod(icase - 1, numel(cfg.sceneTypes)) + 1};
    seed = cfg.seedBase + icase;

    try
        [obstacles, envInfo, P, paramsBase] = makeSceneCase(cfg, sceneType, seed);
        states = cell(1, numel(modes));
        timesMs = nan(cfg.numTimingRepeats, numel(modes));
        modeSummary = repmat(emptyModeResult(), 1, numel(modes));

        for imode = 1:numel(modes)
            paramsMode = paramsBase;
            paramsMode.clearanceMode = modes{imode};

            % Warm-up: avoid first-call overhead contaminating timing.
            states{imode} = evaluateCSSCGlobal(P, obstacles, paramsMode);

            for r = 1:cfg.numTimingRepeats
                state = evaluateCSSCGlobal(P, obstacles, paramsMode);
                timesMs(r, imode) = 1000 * state.timing.total;
            end
            states{imode} = state;
            modeSummary(imode) = summarizeMode(icase, sceneType, seed, ...
                modes{imode}, state, timesMs(:, imode));

            row = row + 1;
            modeResults(row) = modeSummary(imode);
        end

        segment = modeSummary(strcmp(modes, 'segment'));
        envelope = modeSummary(strcmp(modes, 'envelope'));
        hybrid = modeSummary(strcmp(modes, 'hybrid'));

        caseResults(icase) = summarizeCase(icase, sceneType, seed, ...
            numel(obstacles), segment, envelope, hybrid);

        sceneData{icase} = struct( ...
            'sceneType', sceneType, ...
            'seed', seed, ...
            'obstacles', obstacles, ...
            'envInfo', envInfo, ...
            'P', P, ...
            'paramsBase', paramsBase, ...
            'states', {states}, ...
            'timesMs', timesMs);

    catch ME
        warning('Case %d failed (%s, seed=%d): %s', ...
            icase, sceneType, seed, ME.message);
        caseResults(icase) = emptyCaseResult();
        caseResults(icase).caseId = icase;
        caseResults(icase).sceneType = string(sceneType);
        caseResults(icase).seed = seed;
        caseResults(icase).errorMessage = string(ME.message);
    end

    if cfg.printEvery > 0 && (mod(icase, cfg.printEvery) == 0 || icase == cfg.numScenes)
        fprintf('  finished %d / %d cases\n', icase, cfg.numScenes);
    end
end

modeResults = modeResults(1:row);
elapsedSec = toc(tAll);

caseTable = struct2table(caseResults);
modeTable = struct2table(modeResults);
statsByMode = makeStatsByMode(caseTable);
statsByScene = makeStatsByScene(caseTable);

fprintf('\n[stats by mode]\n');
disp(statsByMode);
fprintf('\n[stats by scene type]\n');
disp(statsByScene);
fprintf('[elapsed] %.2f s\n', elapsedSec);

if cfg.saveResults
    writetable(caseTable, fullfile(cfg.outDir, 'clearance_mode_case_results.csv'));
    writetable(modeTable, fullfile(cfg.outDir, 'clearance_mode_raw_mode_results.csv'));
    writetable(statsByMode, fullfile(cfg.outDir, 'clearance_mode_stats_by_mode.csv'));
    writetable(statsByScene, fullfile(cfg.outDir, 'clearance_mode_stats_by_scene.csv'));
    save(fullfile(cfg.outDir, 'clearance_mode_batch_results.mat'), ...
        'cfg', 'caseTable', 'modeTable', 'statsByMode', 'statsByScene', ...
        'sceneData', 'elapsedSec');
    plotBatchSummary(caseTable, cfg);
    writeNotes(cfg);
    fprintf('[saved] %s\n', cfg.outDir);
end

%% Configuration and scene generation

function cfg = makeConfig(projectRoot)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_clearance_modes_batch_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.figDir = fullfile(cfg.outDir, 'figures');
    cfg.saveResults = true;

    cfg.numScenes = 100;
    cfg.numTimingRepeats = 3;
    cfg.printEvery = 10;
    cfg.seedBase = 26000;

    cfg.sceneTypes = { ...
        'offsetDoubleSlit', ...
        'fourRectSChannel', ...
        'staggeredBaffles3', ...
        'randomCircles', ...
        'randomRects', ...
        'randomMixed'};

    cfg.nU = 180;
    cfg.pathSampleN = 500;
    cfg.activeTopK = 10;
    cfg.activeClearanceMargin = 0.012;

    cfg.hybridTriggerFactor = 1.2;
    cfg.hybridRefineActiveTopK = true;
    cfg.hybridForceExactStride = 0;

end

function [obstacles, envInfo, P, params] = makeSceneCase(cfg, sceneType, seed)
    opts = baseEnvOpts(cfg);

    switch lower(sceneType)
        case 'offsetdoubleslit'
            opts.xWalls = [0.30 + 0.04*randUnit(seed, 1), ...
                           0.66 + 0.05*randUnit(seed, 2)];
            opts.gapCentersY = [-0.16 + 0.08*randUnit(seed, 3), ...
                                 0.10 + 0.10*randUnit(seed, 4)];
            opts.gapHeight = 0.09 + 0.045*randUnit(seed, 5);
            opts.wallThickness = 0.07 + 0.025*randUnit(seed, 6);
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
                'offsetDoubleSlit', opts);

        case 'fourrectschannel'
            opts.seed = seed;
            opts.dGap = 0.07 + 0.07*randUnit(seed, 1);
            opts.channelW = 0.58 + 0.16*randUnit(seed, 2);
            opts.channelH = cfg.bounds(2,2) - cfg.bounds(2,1);
            opts.yInOutRange = [-0.20, 0.20];
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
                'fourRectSChannel', opts);

        case 'staggeredbaffles3'
            yTop = 0.10 + 0.08*randUnit(seed, 1);
            yBot = -0.10 - 0.08*randUnit(seed, 2);
            opts.centers = [
                0.25 + 0.03*randUnit(seed, 3), yTop
                0.50 + 0.03*randUnit(seed, 4), yBot
                0.75 + 0.03*randUnit(seed, 5), yTop - 0.03*randUnit(seed, 6)
            ];
            opts.halfSize = [0.022 + 0.012*randUnit(seed, 7), ...
                             0.15 + 0.06*randUnit(seed, 8)];
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
                'staggeredBaffles3', opts);

        case 'randomcircles'
            opts.seed = seed;
            opts.nObs = 8 + mod(seed, 5);
            opts.radiusRange = [0.025, 0.055];
            opts.minGap = 0.018;
            [obstacles, envInfo] = generateCSSCEnvironment2D( ...
                'randomCircles', opts);

        case 'randomrects'
            opts.seed = seed;
            opts.nObs = 7 + mod(seed, 5);
            opts.halfSizeXRange = [0.020, 0.060];
            opts.halfSizeYRange = [0.025, 0.080];
            opts.yawRange = [-pi/3, pi/3];
            opts.minGap = 0.016;
            [obstacles, envInfo] = generateCSSCEnvironment2D( ...
                'randomRects', opts);

        case 'randommixed'
            opts.seed = seed;
            opts.nCircle = 5 + mod(seed, 4);
            opts.nRect = 5 + mod(seed + 2, 4);
            opts.radiusRange = [0.022, 0.052];
            opts.halfSizeXRange = [0.022, 0.058];
            opts.halfSizeYRange = [0.025, 0.075];
            opts.yawRange = [-pi/3, pi/3];
            opts.minGap = 0.016;
            [obstacles, envInfo] = generateCSSCEnvironment2D( ...
                'randomMixed', opts);

        otherwise
            error('Unknown sceneType: %s', sceneType);
    end

    P = makePathForScene(cfg, envInfo, sceneType, seed);
    params = makeEvalParams(cfg, P);
end

function opts = baseEnvOpts(cfg)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    opts.maxTry = 4000;
    opts.keepoutStart = 0.08;
    opts.keepoutGoal = 0.08;
end

function P = makePathForScene(cfg, envInfo, sceneType, seed)
    startPt = envInfo.startPt;
    goalPt = envInfo.goalPt;

    switch lower(sceneType)
        case 'offsetdoubleslit'
            gapY = envInfo.params.gapCentersY;
            waypoints = [
                startPt
                0.20, 0.03*randSigned(seed, 1)
                envInfo.params.xWalls(1), gapY(1)
                0.50, 0.02*randSigned(seed, 2)
                envInfo.params.xWalls(2), gapY(2)
                0.82, 0.03*randSigned(seed, 3)
                goalPt
            ];

        case 'fourrectschannel'
            yIn = envInfo.params.yIn;
            yOut = envInfo.params.yOut;
            midY = 0.5 * (yIn + yOut);
            waypoints = [
                startPt
                0.18, yIn
                0.36, yIn
                0.50, midY
                0.64, yOut
                0.82, yOut
                goalPt
            ];

        case 'staggeredbaffles3'
            waypoints = [
                startPt
                0.18, -0.08
                0.35, -0.18
                0.50,  0.18
                0.66,  0.15
                0.82, -0.04
                goalPt
            ];

        otherwise
            amp1 = 0.08 + 0.04*randUnit(seed, 1);
            amp2 = 0.03 + 0.03*randUnit(seed, 2);
            phase = 2*pi*randUnit(seed, 3);
            x = linspace(startPt(1), goalPt(1), 8).';
            s = (x - x(1)) / max(eps, x(end) - x(1));
            y = amp1*sin(2*pi*s + phase) + amp2*sin(4*pi*s + 0.7*phase);
            waypoints = [x, y];
            waypoints(1,:) = startPt;
            waypoints(end,:) = goalPt;
    end

    P = resamplePolylineLocal(waypoints, 9);
    P(1,:) = startPt;
    P(end,:) = goalPt;
end

function params = makeEvalParams(cfg, P)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;
    params.knot = makeClampedUniformKnot(size(P, 1), params.degree);

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
    params.activeTopK = cfg.activeTopK;
    params.activeClearanceMargin = cfg.activeClearanceMargin;

    params.hybrid = struct();
    params.hybrid.triggerFactor = cfg.hybridTriggerFactor;
    params.hybrid.refineActiveTopK = cfg.hybridRefineActiveTopK;
    params.hybrid.forceExactStride = cfg.hybridForceExactStride;
end

%% Summaries

function result = summarizeMode(caseId, sceneType, seed, modeName, state, samplesMs)
    samples = samplesMs(isfinite(samplesMs));
    if isempty(samples)
        samples = nan;
    end

    result = emptyModeResult();
    result.caseId = caseId;
    result.sceneType = string(sceneType);
    result.seed = seed;
    result.mode = string(modeName);
    result.minClear = state.minClear;
    result.minType = string(state.minType);
    result.meanTimeMs = mean(samples);
    result.minTimeMs = min(samples);
    result.maxTimeMs = max(samples);
    result.nValidLine = state.timing.numValidLine;
    result.nValidSegment = state.timing.numValidSegment;
    result.nExactSegment = state.timing.numExactSegment;
    result.exactFraction = state.timing.exactFraction;
end

function result = summarizeCase(caseId, sceneType, seed, nObs, segment, envelope, hybrid)
    result = emptyCaseResult();
    result.caseId = caseId;
    result.sceneType = string(sceneType);
    result.seed = seed;
    result.numObstacles = nObs;

    result.segmentMinClear = segment.minClear;
    result.segmentTimeMs = segment.meanTimeMs;

    result.envelopeMinClear = envelope.minClear;
    result.envelopeTimeMs = envelope.meanTimeMs;
    result.envelopeAbsClearDiff = abs(envelope.minClear - segment.minClear);
    result.envelopeSignedClearDiff = envelope.minClear - segment.minClear;
    result.envelopeTimeDiffMs = envelope.meanTimeMs - segment.meanTimeMs;
    result.envelopeTimeSavingMs = segment.meanTimeMs - envelope.meanTimeMs;
    result.envelopeSpeedup = segment.meanTimeMs / envelope.meanTimeMs;
    result.envelopeExactFraction = envelope.exactFraction;

    result.hybridMinClear = hybrid.minClear;
    result.hybridTimeMs = hybrid.meanTimeMs;
    result.hybridAbsClearDiff = abs(hybrid.minClear - segment.minClear);
    result.hybridSignedClearDiff = hybrid.minClear - segment.minClear;
    result.hybridTimeDiffMs = hybrid.meanTimeMs - segment.meanTimeMs;
    result.hybridTimeSavingMs = segment.meanTimeMs - hybrid.meanTimeMs;
    result.hybridSpeedup = segment.meanTimeMs / hybrid.meanTimeMs;
    result.hybridExactFraction = hybrid.exactFraction;
end

function stats = makeStatsByMode(caseTable)
    valid = caseTable.errorMessage == "";
    modeNames = ["envelope"; "hybrid"];
    rows = repmat(emptyStatsRow(), numel(modeNames), 1);

    for i = 1:numel(modeNames)
        mode = modeNames(i);
        rows(i) = statsForMode(caseTable(valid,:), mode, "all");
    end

    stats = struct2table(rows);
end

function stats = makeStatsByScene(caseTable)
    valid = caseTable.errorMessage == "";
    sceneTypes = unique(caseTable.sceneType(valid), 'stable');
    modeNames = ["envelope"; "hybrid"];
    rows = repmat(emptyStatsRow(), numel(sceneTypes) * numel(modeNames), 1);

    k = 0;
    for iscene = 1:numel(sceneTypes)
        scene = sceneTypes(iscene);
        Tscene = caseTable(valid & caseTable.sceneType == scene, :);
        for imode = 1:numel(modeNames)
            k = k + 1;
            rows(k) = statsForMode(Tscene, modeNames(imode), scene);
        end
    end

    stats = struct2table(rows(1:k));
end

function row = statsForMode(T, mode, groupName)
    row = emptyStatsRow();
    row.group = string(groupName);
    row.mode = string(mode);
    row.numCases = height(T);

    switch char(mode)
        case 'envelope'
            clearDiff = T.envelopeAbsClearDiff;
            signedClearDiff = T.envelopeSignedClearDiff;
            timeDiff = T.envelopeTimeDiffMs;
            timeSaving = T.envelopeTimeSavingMs;
            speedup = T.envelopeSpeedup;
            exactFraction = T.envelopeExactFraction;
        case 'hybrid'
            clearDiff = T.hybridAbsClearDiff;
            signedClearDiff = T.hybridSignedClearDiff;
            timeDiff = T.hybridTimeDiffMs;
            timeSaving = T.hybridTimeSavingMs;
            speedup = T.hybridSpeedup;
            exactFraction = T.hybridExactFraction;
        otherwise
            error('Unknown mode: %s', mode);
    end

    row.absClearDiffMin = finiteMin(clearDiff);
    row.absClearDiffMax = finiteMax(clearDiff);
    row.absClearDiffMean = finiteMean(clearDiff);
    row.signedClearDiffMean = finiteMean(signedClearDiff);
    row.timeDiffMsMin = finiteMin(timeDiff);
    row.timeDiffMsMax = finiteMax(timeDiff);
    row.timeDiffMsMean = finiteMean(timeDiff);
    row.timeSavingMsMin = finiteMin(timeSaving);
    row.timeSavingMsMax = finiteMax(timeSaving);
    row.timeSavingMsMean = finiteMean(timeSaving);
    row.speedupMean = finiteMean(speedup);
    row.exactFractionMean = finiteMean(exactFraction);
end

function result = emptyModeResult()
    result = struct();
    result.caseId = nan;
    result.sceneType = "";
    result.seed = nan;
    result.mode = "";
    result.minClear = nan;
    result.minType = "";
    result.meanTimeMs = nan;
    result.minTimeMs = nan;
    result.maxTimeMs = nan;
    result.nValidLine = nan;
    result.nValidSegment = nan;
    result.nExactSegment = nan;
    result.exactFraction = nan;
end

function result = emptyCaseResult()
    result = struct();
    result.caseId = nan;
    result.sceneType = "";
    result.seed = nan;
    result.numObstacles = nan;
    result.segmentMinClear = nan;
    result.segmentTimeMs = nan;
    result.envelopeMinClear = nan;
    result.envelopeTimeMs = nan;
    result.envelopeAbsClearDiff = nan;
    result.envelopeSignedClearDiff = nan;
    result.envelopeTimeDiffMs = nan;
    result.envelopeTimeSavingMs = nan;
    result.envelopeSpeedup = nan;
    result.envelopeExactFraction = nan;
    result.hybridMinClear = nan;
    result.hybridTimeMs = nan;
    result.hybridAbsClearDiff = nan;
    result.hybridSignedClearDiff = nan;
    result.hybridTimeDiffMs = nan;
    result.hybridTimeSavingMs = nan;
    result.hybridSpeedup = nan;
    result.hybridExactFraction = nan;
    result.errorMessage = "";
end

function row = emptyStatsRow()
    row = struct();
    row.group = "";
    row.mode = "";
    row.numCases = 0;
    row.absClearDiffMin = nan;
    row.absClearDiffMax = nan;
    row.absClearDiffMean = nan;
    row.signedClearDiffMean = nan;
    row.timeDiffMsMin = nan;
    row.timeDiffMsMax = nan;
    row.timeDiffMsMean = nan;
    row.timeSavingMsMin = nan;
    row.timeSavingMsMax = nan;
    row.timeSavingMsMean = nan;
    row.speedupMean = nan;
    row.exactFractionMean = nan;
end

%% Plotting and notes

function plotBatchSummary(caseTable, cfg)
    valid = caseTable.errorMessage == "";
    T = caseTable(valid,:);

    fig = figure('Color','w', 'Position', [100, 100, 760, 360]);
    tiledlayout(fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');

    nexttile;
    y = [T.envelopeAbsClearDiff; T.hybridAbsClearDiff];
    g = [repmat("envelope", height(T), 1); repmat("hybrid", height(T), 1)];
    boxplot(y, categorical(g), 'Symbol','k.');
    ylabel('|minClear_{mode} - minClear_{segment}|');
    title('Clearance difference');
    grid on;

    nexttile;
    y = [T.envelopeTimeSavingMs; T.hybridTimeSavingMs];
    g = [repmat("envelope", height(T), 1); repmat("hybrid", height(T), 1)];
    boxplot(y, categorical(g), 'Symbol','k.');
    ylabel('segment time - mode time (ms)');
    title('Runtime saving');
    grid on;

    exportgraphics(fig, fullfile(cfg.figDir, 'clearance_mode_batch_boxplots.jpg'), ...
        'Resolution', cfg.figureResolution, 'BackgroundColor','white');
    close(fig);
end

function writeNotes(cfg)
    fid = fopen(fullfile(cfg.outDir, 'clearance_mode_batch_notes.txt'), 'w');
    if fid < 0
        return;
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'Batch clearance mode comparison notes\n\n');
    fprintf(fid, 'Each case uses the same B-spline path and obstacles for segment, envelope, and hybrid.\n');
    fprintf(fid, 'segment is the reference: exact segment-obstacle clearance for every valid chord.\n');
    fprintf(fid, 'envelope uses G/M/N point-SDF probes only and can miss interior chord collisions.\n');
    fprintf(fid, 'hybrid uses G/M/N probes, then exact segment refinement near dMin and for active top-K candidates.\n');
    fprintf(fid, 'timeDiffMs = modeMeanTimeMs - segmentMeanTimeMs. Negative values mean faster than segment.\n');
    fprintf(fid, 'timeSavingMs = segmentMeanTimeMs - modeMeanTimeMs. Positive values mean faster than segment.\n');
end

%% Numeric utilities

function val = finiteMean(x)
    x = x(isfinite(x));
    if isempty(x)
        val = nan;
    else
        val = mean(x);
    end
end

function val = finiteMin(x)
    x = x(isfinite(x));
    if isempty(x)
        val = nan;
    else
        val = min(x);
    end
end

function val = finiteMax(x)
    x = x(isfinite(x));
    if isempty(x)
        val = nan;
    else
        val = max(x);
    end
end

function p = resamplePolylineLocal(path, n)
    if size(path, 1) == n
        p = path;
        return;
    end

    segLen = vecnorm(diff(path, 1, 1), 2, 2);
    s = [0; cumsum(segLen)];
    if s(end) <= eps
        p = repmat(path(1,:), n, 1);
        return;
    end

    sq = linspace(0, s(end), n).';
    p = interp1(s, path, sq, 'linear');
end

function r = randUnit(seed, offset)
    r = 0.5 + 0.5 * sin(12.9898 * (seed + 37 * offset) + 78.233);
    r = r - floor(r);
end

function r = randSigned(seed, offset)
    r = 2 * randUnit(seed, offset) - 1;
end
