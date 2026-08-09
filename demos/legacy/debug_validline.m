clear; clc; close all;

%% Debug validChord versus validLine on the three anomalous S-channel cases
% Rebuilds the original S-channel Easy trials 13, 14, and 34 using the same
% environment/planner seed mapping as demo2_baseline_compare. Each panel
% compares the CSSC B-spline initialization, a fresh RRTSC-2D result, and
% the repaired CSSC result. Final clearances use evaluateCSSCHighPrecision.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeDebugConfig(projectRoot);
spec = makeSChannelEasySpec(cfg);
paramsOpt = makeOptimizationParams(cfg);
paramsEval = makeEvaluationParams(cfg);

if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

cases = repmat(emptyCase(), numel(cfg.envSeeds), 1);
fprintf('\n[validChord/validLine debug]\n');
fprintf('  output: %s\n', cfg.outDir);

for i = 1:numel(cfg.envSeeds)
    envSeed = cfg.envSeeds(i);
    trialIndex = cfg.trialIndices(i);
    plannerSeed = cfg.plannerSeedBase + 2000 + trialIndex;

    [obstacles, envInfo] = ...
        generateCSSCBenchmarkScene2D(spec, cfg, envSeed);
    envInfo.obstacles = obstacles;

    rrtOpts = makeRRTOpts(cfg, plannerSeed);
    [pathRRT, rrtInfo] = planRRT2D( ...
        envInfo.startPt, envInfo.goalPt, obstacles, rrtOpts);
    if ~rrtInfo.success
        error('CSSC RRT failed for trial %d, seed %d.', ...
            trialIndex, envSeed);
    end

    shortcutOpts = rrtOpts;
    shortcutOpts.seed = cfg.shortcutSeedBase + plannerSeed;
    shortcutOpts.numShortcut = cfg.numShortcut;
    [pathShort, shortcutInfo] = ...
        shortcutPath2D(pathRRT, obstacles, shortcutOpts);

    nCtrl = max(cfg.degree + 1, ...
        ceil(polylineLengthLocal(pathShort) / (0.5 * cfg.L)) + 1);
    [Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, ...
        struct('degree', cfg.degree, 'nCtrl', nCtrl));

    params = paramsOpt;
    params.knot = splineInfo.knot;
    paramsEvalCase = paramsEval;
    paramsEvalCase.knot = splineInfo.knot;

    initMetrics = evaluateCSSCHighPrecision( ...
        Pinit, obstacles, paramsEvalCase);
    optLog = evalc( ...
        '[Popt, optInfo] = optimizeCSSC2D(Pinit, Pinit, obstacles, params);'); %#ok<NASGU>
    optMetrics = evaluateCSSCHighPrecision( ...
        Popt, obstacles, paramsEvalCase);

    rrtscOpts = makeRRTSCOpts(cfg, plannerSeed, envInfo);
    [Prrtsc, pathRRTSC, rrtscInfo] = planRRTSC2D( ...
        envInfo.startPt, envInfo.goalPt, obstacles, rrtscOpts);
    if rrtscInfo.success
        rrtscEval = paramsEval;
        rrtscEval.knot = rrtscInfo.finalSplineInfo.knot;
        rrtscMetrics = evaluateCSSCHighPrecision( ...
            Prrtsc, obstacles, rrtscEval);
    else
        rrtscMetrics = evaluateCSSCHighPrecision( ...
            [], obstacles, paramsEval);
    end

    cases(i).trialIndex = trialIndex;
    cases(i).envSeed = envSeed;
    cases(i).plannerSeed = plannerSeed;
    cases(i).obstacles = obstacles;
    cases(i).envInfo = envInfo;
    cases(i).pathRRT = pathRRT;
    cases(i).pathShort = pathShort;
    cases(i).Pinit = Pinit;
    cases(i).Popt = Popt;
    cases(i).Prrtsc = Prrtsc;
    cases(i).pathRRTSC = pathRRTSC;
    cases(i).splineInfo = splineInfo;
    cases(i).rrtInfo = rrtInfo;
    cases(i).shortcutInfo = shortcutInfo;
    cases(i).optInfo = optInfo;
    cases(i).rrtscInfo = rrtscInfo;
    cases(i).initMetrics = initMetrics;
    cases(i).optMetrics = optMetrics;
    cases(i).rrtscMetrics = rrtscMetrics;

    fprintf(['  trial %2d | envSeed %d | init %.5f (%d/%d chords) | ' ...
        'CSSC %.5f (%d/%d) | RRTSC %.5f | dP %.4g\n'], ...
        trialIndex, envSeed, ...
        initMetrics.minClear, initMetrics.numEvaluatedChords, ...
        initMetrics.numValidChords, ...
        optMetrics.minClear, optMetrics.numEvaluatedChords, ...
        optMetrics.numValidChords, ...
        rrtscMetrics.minClear, norm(Popt - Pinit, 'fro'));
end

figPath = fullfile(cfg.outDir, 'debug_validline.jpg');
matPath = fullfile(cfg.outDir, 'debug_validline_data.mat');
drawComparisonFigure(cases, cfg, figPath);
save(matPath, 'cfg', 'spec', 'paramsOpt', 'paramsEval', 'cases');

fprintf('  figure: %s\n', figPath);
fprintf('  data  : %s\n', matPath);

%% Configuration

function cfg = makeDebugConfig(projectRoot)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.outDir = fullfile(projectRoot, 'results', 'figures', ...
        'debug_validline');

    cfg.trialIndices = [13, 14, 34];
    cfg.envSeeds = [1801, 1802, 1822];
    cfg.sChannelGap = cfg.sChannel.dGap(1);
    cfg.sChannelW = cfg.sChannel.W;
    cfg.maxEnvSeedTry = 50;

    cfg.figureVisible = 'on';
    cfg.figureWidthCm = 18;
    cfg.figureHeightCm = 6.8;
    cfg.figureResolution = 350;
    cfg.fontName = 'Times New Roman';
end

function spec = makeSChannelEasySpec(cfg)
    specs = makeCSSCBenchmarkSceneSpecs2D(cfg);
    spec = specs(2);
    spec.dGap = cfg.sChannelGap;
    spec.channelW = cfg.sChannelW;
    spec.channelH = diff(cfg.bounds(2,:));
end

function opts = makeRRTOpts(cfg, seed)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.stepSize = cfg.stepSize;
    opts.goalBias = cfg.goalBias;
    opts.goalTol = cfg.goalTol;
    opts.maxIter = cfg.maxIter;
    opts.collisionResolution = cfg.collisionResolution;
    opts.inflateRadius = cfg.inflateRadius;
    opts.seed = seed;
end

function opts = makeRRTSCOpts(cfg, seed, envInfo)
    opts = defaultRRTSC2DParams();
    opts.L = cfg.L;
    opts.dMin = cfg.dMin;
    opts.safetyMargin = cfg.dMin;
    opts.degree = cfg.degree;
    opts.bounds = envInfo.bounds;
    opts.seed = seed;
    opts.maxAttempts = cfg.rrtscMaxAttempts;
    opts.maxTotalTime = cfg.rrtscMaxTotalTime;
    opts.controlPointSpacingFactor = ...
        cfg.rrtscControlPointSpacingFactor;
    opts.verbose = false;
    opts.rrt = makeRRTOpts(cfg, seed);
    opts.centerline.sampleResolution = cfg.rrtscCenterlineResolution;
    opts.centerline.minSamples = cfg.rrtscCenterlineMinSamples;
    opts.chord.nU = cfg.rrtscChordNU;
    opts.chord.maxRefinement = cfg.rrtscMaxRefinement;
end

function params = makeOptimizationParams(cfg)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;
    params.clearanceMode = 'segment';

    params.envOpts = struct( ...
        'uRange', [0, 1], ...
        'vSearchRange', [0, 1], ...
        'nU', 120, ...
        'epsV', 1e-6, ...
        'tolDen', 1e-6);

    params.wObs = 600.0;
    params.wClear = 100.0;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0.0;
    params.wCurv = 0.0;
    params.kappaMax = 2.0;

    params.solver = struct('gradMode', 'semi-analytic');
    params.numIter = 100;
    params.lr = 0.001;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 100;
    params.activeTopK = 100;
    params.activeClearanceMargin = 0.0125;

    params.stop = struct( ...
        'enable', true, ...
        'minIter', 10, ...
        'window', 8, ...
        'tolRelJ', 1e-2, ...
        'tolGrad', 1e-1, ...
        'tolStep', 1e-3, ...
        'patience', 10, ...
        'tolBestRel', 1e-4, ...
        'requireSafe', true, ...
        'clearanceMargin', 0.0);

    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

function params = makeEvaluationParams(cfg)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.degree = cfg.degree;
    params.envOpts = struct( ...
        'uRange', [0, 1], ...
        'vSearchRange', [0, 1], ...
        'nU', 480, ...
        'epsV', 1e-8, ...
        'tolDen', 1e-10);
    params.pathSampleN = 1200;
    params.pointClearanceResolution = 0.001;
end

%% Plotting

function drawComparisonFigure(cases, cfg, filePath)
    fig = figure( ...
        'Color', 'w', ...
        'Visible', cfg.figureVisible, ...
        'Units', 'centimeters', ...
        'Position', [2, 2, cfg.figureWidthCm, cfg.figureHeightCm]);
    layout = tiledlayout(fig, 1, numel(cases), ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    legendHandles = gobjects(3, 1);
    for i = 1:numel(cases)
        ax = nexttile(layout, i);
        hold(ax, 'on');
        axis(ax, 'equal');
        axis(ax, 'off');
        xlim(ax, cfg.bounds(1,:) + [-0.01, 0.01]);
        ylim(ax, cfg.bounds(2,:) + [-0.01, 0.01]);
        set(ax, 'FontName', cfg.fontName, 'FontSize', 10);

        drawObstacles(ax, cases(i).obstacles, cfg.dMin);
        drawBoundary(ax, cfg.bounds);

        hInit = plot(ax, cases(i).initMetrics.pathSample(:,1), ...
            cases(i).initMetrics.pathSample(:,2), '--', ...
            'Color', [0.18, 0.38, 0.76], 'LineWidth', 1.3);

        if cases(i).rrtscInfo.success
            hRRTSC = plot(ax, cases(i).rrtscMetrics.pathSample(:,1), ...
                cases(i).rrtscMetrics.pathSample(:,2), '-.', ...
                'Color', [0.88, 0.45, 0.12], 'LineWidth', 1.4);
        else
            hRRTSC = plot(ax, nan, nan, '-.', ...
                'Color', [0.88, 0.45, 0.12], 'LineWidth', 1.4);
        end

        hCSSC = plot(ax, cases(i).optMetrics.pathSample(:,1), ...
            cases(i).optMetrics.pathSample(:,2), '-', ...
            'Color', [0.05, 0.55, 0.28], 'LineWidth', 1.7);

        drawStartGoal(ax, cfg);
        title(ax, sprintf(['Trial %d, seed %d\n' ...
            'init %.3f | RRTSC %.3f | CSSC %.3f'], ...
            cases(i).trialIndex, cases(i).envSeed, ...
            cases(i).initMetrics.minClear, ...
            cases(i).rrtscMetrics.minClear, ...
            cases(i).optMetrics.minClear), ...
            'FontName', cfg.fontName, 'FontSize', 10, ...
            'FontWeight', 'normal');

        if i == 1
            legendHandles = [hInit; hRRTSC; hCSSC];
        end
    end

    lgd = legend(legendHandles, ...
        {'CSSC B-spline initial', 'RRTSC-2D', 'CSSC optimized'}, ...
        'NumColumns', 3, 'Location', 'southoutside', ...
        'FontName', cfg.fontName, 'FontSize', 10, 'Box', 'off');
    lgd.Layout.Tile = 'south';

    exportgraphics(fig, filePath, ...
        'Resolution', cfg.figureResolution, ...
        'BackgroundColor', 'white');
end

function drawObstacles(ax, obstacles, dMin)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                t = linspace(0, 2*pi, 120).';
                vertices = obs.center + obs.radius * [cos(t), sin(t)];
                safeVertices = obs.center + (obs.radius + dMin) * ...
                    [cos(t), sin(t)];

            case {'rect', 'rectangle', 'box'}
                vertices = rectangleVertices(obs.center, obs.halfSize, obs.yaw);
                safeVertices = rectangleVertices( ...
                    obs.center, obs.halfSize + dMin, obs.yaw);

            case 'polygon'
                vertices = obs.vertices;
                safeVertices = zeros(0, 2);

            otherwise
                continue;
        end

        patch(ax, vertices(:,1), vertices(:,2), [0.82, 0.83, 0.84], ...
            'FaceAlpha', 0.95, 'EdgeColor', [0.20, 0.21, 0.22], ...
            'LineWidth', 0.8, 'HandleVisibility', 'off');
        if ~isempty(safeVertices)
            plot(ax, [safeVertices(:,1); safeVertices(1,1)], ...
                [safeVertices(:,2); safeVertices(1,2)], '--', ...
                'Color', [0.72, 0.18, 0.18], 'LineWidth', 0.7, ...
                'HandleVisibility', 'off');
        end
    end
end

function vertices = rectangleVertices(center, halfSize, yaw)
    local = [
        -halfSize(1), -halfSize(2)
         halfSize(1), -halfSize(2)
         halfSize(1),  halfSize(2)
        -halfSize(1),  halfSize(2)
    ];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    vertices = local * R.' + center;
end

function drawBoundary(ax, bounds)
    rectangle(ax, 'Position', [bounds(1,1), bounds(2,1), ...
        diff(bounds(1,:)), diff(bounds(2,:))], ...
        'EdgeColor', [0.08, 0.08, 0.08], 'LineWidth', 1.1);
end

function drawStartGoal(ax, cfg)
    plot(ax, cfg.startPt(1), cfg.startPt(2), 'o', ...
        'MarkerSize', 5.5, 'MarkerFaceColor', [0.08, 0.58, 0.55], ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'HandleVisibility', 'off');
    plot(ax, cfg.goalPt(1), cfg.goalPt(2), 's', ...
        'MarkerSize', 5.5, 'MarkerFaceColor', [0.93, 0.64, 0.12], ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'HandleVisibility', 'off');
end

function len = polylineLengthLocal(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end

function value = emptyCase()
    value = struct( ...
        'trialIndex', nan, ...
        'envSeed', nan, ...
        'plannerSeed', nan, ...
        'obstacles', [], ...
        'envInfo', [], ...
        'pathRRT', zeros(0, 2), ...
        'pathShort', zeros(0, 2), ...
        'Pinit', zeros(0, 2), ...
        'Popt', zeros(0, 2), ...
        'Prrtsc', zeros(0, 2), ...
        'pathRRTSC', zeros(0, 2), ...
        'splineInfo', [], ...
        'rrtInfo', [], ...
        'shortcutInfo', [], ...
        'optInfo', [], ...
        'rrtscInfo', [], ...
        'initMetrics', [], ...
        'optMetrics', [], ...
        'rrtscMetrics', []);
end
