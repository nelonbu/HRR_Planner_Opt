clear; clc; close all;

%% Demo: multi-scenario RRT initialization and CSSC optimization tests
% Figure 1: original obstacles.
% Figure 2: RRT/shortcut polyline paths.
% Figure 3: B-spline initial paths converted from RRT paths.
% Figure 4: optimized CSSC paths.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end

saveFigure = false;
enableOptimization = true;

common = struct();
common.bounds = [0, 1; -0.4, 0.4];
common.startPt = [0.05, 0.0];
common.goalPt = [0.95, 0.0];

envSet = buildEnvironmentSet(common);
optParamsBase = makeOptimizationParams();
envSet = planInitialPathsForEnvironments(envSet, optParamsBase);
if enableOptimization
    envSet = optimizeEnvironments(envSet, optParamsBase);
end

figObs = figure('Color','w', ...
    'Name','Multi-scenario tests: original obstacles', ...
    'Position',[40 60 1450 780]);
drawEnvironmentFigure(envSet, 'obstacles');

figRRT = figure('Color','w', ...
    'Name','Multi-scenario tests: RRT paths', ...
    'Position',[80 90 1450 780]);
drawEnvironmentFigure(envSet, 'rrt');

figSpline = figure('Color','w', ...
    'Name','Multi-scenario tests: B-spline initial paths', ...
    'Position',[120 120 1450 780]);
drawEnvironmentFigure(envSet, 'bspline');

figOpt = [];
if enableOptimization
    figOpt = figure('Color','w', ...
        'Name','Multi-scenario tests: optimized CSSC paths', ...
        'Position',[160 150 1450 780]);
    drawEnvironmentFigure(envSet, 'optimized');
end

fprintf('\n[multi-scenario test summary]\n');
for i = 1:numel(envSet)
    env = envSet{i};
    if env.rrtInfo.success && env.optInfo.success
        fprintf('%2d | %-24s | obs=%2d | RRT ok | nCtrl=%2d | initClear=%.4f | finalClear=%.4f | iter=%3d\n', ...
            i, env.id, numel(env.obstacles), size(env.Pinit, 1), ...
            env.initMinClear, env.optInfo.info.finalMinClear, env.optInfo.info.numIterActual);
    elseif env.rrtInfo.success
        fprintf('%2d | %-24s | obs=%2d | RRT ok | nodes=%3d | short=%3d | nCtrl=%2d | opt skipped/failed\n', ...
            i, env.id, numel(env.obstacles), env.rrtInfo.numNodes, ...
            size(env.pathShort, 1), size(env.Pinit, 1));
    else
        fprintf('%2d | %-24s | obs=%2d | RRT failed | %s\n', ...
            i, env.id, numel(env.obstacles), env.rrtInfo.message);
    end
end

if saveFigure
    outDir = fullfile(projectRoot, 'results', 'tmp');
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    exportgraphics(figObs, fullfile(outDir, 'environment_obstacles.png'), 'Resolution', 220);
    exportgraphics(figRRT, fullfile(outDir, 'environment_rrt_paths.png'), 'Resolution', 220);
    exportgraphics(figSpline, fullfile(outDir, 'environment_bspline_paths.png'), 'Resolution', 220);
    if enableOptimization && ~isempty(figOpt)
        exportgraphics(figOpt, fullfile(outDir, 'environment_optimized_paths.png'), 'Resolution', 220);
    end
    fprintf('[saved] %s\n', outDir);
end

%% Environment setup

function envSet = buildEnvironmentSet(common)
    structuredTypes = {'singleSlit', 'offsetDoubleSlit', 'fourRectSChannel', 'staggeredBaffles3'};
    envSet = cell(8, 1);

    for i = 1:numel(structuredTypes)
        [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(structuredTypes{i}, common);
        envInfo.obstacles = obstacles;
        envInfo.source = 'structured';
        envSet{i} = envInfo;
    end

    randomCases = {
        'randomCircles', 'Random Circles', makeRandomOpts(common, 21)
        'randomRects',   'Random Rects',   makeRandomOpts(common, 22)
        'randomMixed',   'Random Mixed',   makeRandomOpts(common, 23)
        'randomMixed',   'Dense Mixed',    makeDenseRandomOpts(common, 24)
    };

    for i = 1:size(randomCases, 1)
        envType = randomCases{i, 1};
        displayName = randomCases{i, 2};
        opts = randomCases{i, 3};

        [obstacles, envInfo] = generateCSSCEnvironment2D(envType, opts);
        envInfo.name = displayName;
        envInfo.id = lower(strrep(displayName, ' ', '_'));
        envInfo.obstacles = obstacles;
        envInfo.obstacleLabels = {};
        envInfo.source = 'random';
        envSet{numel(structuredTypes) + i} = envInfo;
    end
end

function opts = makeRandomOpts(common, seed)
    opts = common;
    opts.seed = seed;
    opts.minGap = 0.025;
    opts.maxTry = 3000;
end

function opts = makeDenseRandomOpts(common, seed)
    opts = makeRandomOpts(common, seed);
    opts.nCircle = 10;
    opts.nRect = 8;
    opts.radiusRange = [0.018, 0.040];
    opts.halfSizeXRange = [0.018, 0.045];
    opts.halfSizeYRange = [0.025, 0.065];
    opts.minGap = 0.015;
end

%% RRT, spline initialization, and optimization

function params = makeOptimizationParams()
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
    params.numIter = 120;
    params.lr = 0.001;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 40;
    params.saveInterval = 40;

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

function envSet = planInitialPathsForEnvironments(envSet, paramsBase)
    degree = paramsBase.degree;
    nominalL = paramsBase.L;

    for i = 1:numel(envSet)
        env = envSet{i};
        rrtOpts = makeRRTOpts(env, i);

        [pathRRT, rrtInfo] = runRRTWithRestarts(env, rrtOpts);
        env.pathRRT = pathRRT;
        env.rrtInfo = rrtInfo;
        env.pathShort = zeros(0, 2);
        env.Pinit = zeros(0, 2);
        env.splineInfo = struct();
        env.Popt = zeros(0, 2);
        env.optState = struct();
        env.optInfo = struct('success', false, 'message', 'Optimization not run.');

        if rrtInfo.success
            shortcutOpts = rrtOpts;
            shortcutOpts.seed = 7000 + i;
            shortcutOpts.numShortcut = 160;
            [pathShort, shortcutInfo] = shortcutPath2D(pathRRT, env.obstacles, shortcutOpts);

            approxLen = polylineLength(pathShort);
            nCtrl = max(degree + 1, ceil(approxLen / (0.5 * nominalL)) + 1);

            splineOpts = struct('degree', degree, 'nCtrl', nCtrl);
            [Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, splineOpts);
            splineInfo.shortcutInfo = shortcutInfo;

            paramsEval = paramsBase;
            paramsEval.knot = splineInfo.knot;
            paramsEval.enablePathSample = false;
            paramsEval.enablePointClearance = false;
            paramsEval.printEvalTiming = false;

            initState = evaluateCSSCGlobal(Pinit, env.obstacles, paramsEval);

            env.pathShort = pathShort;
            env.Pinit = Pinit;
            env.Pref = Pinit;
            env.splineInfo = splineInfo;
            env.initState = initState;
            env.initMinClear = initState.minClear;
        end

        envSet{i} = env;
    end
end

function envSet = optimizeEnvironments(envSet, paramsBase)
    for i = 1:numel(envSet)
        env = envSet{i};
        env.Popt = zeros(0, 2);
        env.optState = struct();
        env.optInfo = struct('success', false, 'message', 'RRT failed or no B-spline init.');

        if ~env.rrtInfo.success || isempty(env.Pinit)
            envSet{i} = env;
            continue;
        end

        params = paramsBase;
        params.knot = env.splineInfo.knot;

        fprintf('\n[optimize case %d/%d] %s\n', i, numel(envSet), env.id);
        try
            [Popt, info] = optimizeCSSC2D(env.Pinit, env.Pref, env.obstacles, params);

            paramsEval = params;
            paramsEval.enablePathSample = false;
            paramsEval.enablePointClearance = false;
            paramsEval.printEvalTiming = false;
            optState = evaluateCSSCGlobal(Popt, env.obstacles, paramsEval);

            env.Popt = Popt;
            env.optState = optState;
            env.optInfo = struct();
            env.optInfo.success = true;
            env.optInfo.message = 'Optimization finished.';
            env.optInfo.params = params;
            env.optInfo.info = info;
        catch ME
            env.optInfo = struct();
            env.optInfo.success = false;
            env.optInfo.message = ME.message;
        end

        envSet{i} = env;
    end
end

function rrtOpts = makeRRTOpts(env, idx)
    rrtOpts = struct();
    rrtOpts.bounds = env.bounds;
    rrtOpts.stepSize = 0.030;
    rrtOpts.goalBias = 0.16;
    rrtOpts.goalTol = 0.035;
    rrtOpts.maxIter = 5000;
    rrtOpts.collisionResolution = 0.004;
    rrtOpts.inflateRadius = 0.0;
    rrtOpts.seed = 3000 + idx;
    rrtOpts.numRestart = 6;
end

function [path, info] = runRRTWithRestarts(env, rrtOpts)
    path = zeros(0, 2);
    info = struct('success', false, 'message', 'RRT failed.', ...
        'numIter', 0, 'numNodes', 0, 'numRestartUsed', 0);

    for trial = 1:rrtOpts.numRestart
        optsTrial = rrtOpts;
        optsTrial.seed = rrtOpts.seed + trial - 1;
        [pathTrial, infoTrial] = planRRT2D(env.startPt, env.goalPt, env.obstacles, optsTrial);

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

%% Figure helpers

function drawEnvironmentFigure(envSet, mode)
    tiledlayout(2, 4, 'Padding','compact', 'TileSpacing','compact');
    for i = 1:numel(envSet)
        env = envSet{i};
        nexttile;
        hold on; grid on; axis equal;
        title(sprintf('%d. %s', i, env.name), 'Interpreter','none');
        xlabel('x'); ylabel('y');

        drawBounds(env.bounds);
        drawObstacles(env.obstacles);
        if isfield(env, 'obstacleLabels')
            drawObstacleLabels(env.obstacles, env.obstacleLabels);
        end
        drawStartGoal(env);

        switch lower(mode)
            case 'obstacles'
                % Obstacles only.

            case 'rrt'
                drawRRTPath(env);

            case 'bspline'
                drawBSplinePath(env);

            case 'optimized'
                drawOptimizedPath(env);

            otherwise
                error('Unknown draw mode: %s', mode);
        end

        xlim(env.bounds(1,:));
        ylim(env.bounds(2,:));
    end
end

function drawStartGoal(env)
    plot(env.startPt(1), env.startPt(2), 'go', ...
        'MarkerFaceColor','g', 'DisplayName','start');
    plot(env.goalPt(1), env.goalPt(2), 'rp', ...
        'MarkerFaceColor','r', 'DisplayName','goal');
end

function drawRRTPath(env)
    if ~env.rrtInfo.success
        text(mean(env.bounds(1,:)), mean(env.bounds(2,:)), 'RRT failed', ...
            'HorizontalAlignment','center', 'FontWeight','bold', ...
            'Color',[0.75 0.05 0.05]);
        return;
    end

    plot(env.pathRRT(:,1), env.pathRRT(:,2), '-', ...
        'Color',[0.72 0.72 0.72], 'LineWidth', 0.8, ...
        'HandleVisibility','off');
    plot(env.pathShort(:,1), env.pathShort(:,2), 'b-', ...
        'LineWidth', 1.8, 'HandleVisibility','off');
end

function drawBSplinePath(env)
    if ~env.rrtInfo.success || isempty(env.Pinit)
        text(mean(env.bounds(1,:)), mean(env.bounds(2,:)), 'No spline init', ...
            'HorizontalAlignment','center', 'FontWeight','bold', ...
            'Color',[0.75 0.05 0.05]);
        return;
    end

    wPlot = linspace(0, 1, 160);
    curve = evalBSplinePath2D(env.Pinit, wPlot, ...
        env.splineInfo.degree, env.splineInfo.knot);

    plot(env.pathShort(:,1), env.pathShort(:,2), '-', ...
        'Color',[0.72 0.72 0.72], 'LineWidth', 0.8, ...
        'HandleVisibility','off');
    plot(env.Pinit(:,1), env.Pinit(:,2), 'ko--', ...
        'MarkerSize', 3.0, 'LineWidth', 0.8, ...
        'HandleVisibility','off');
    plot(curve(:,1), curve(:,2), 'r-', ...
        'LineWidth', 1.8, 'HandleVisibility','off');
end

function drawOptimizedPath(env)
    if ~env.rrtInfo.success || isempty(env.Pinit)
        drawCenteredMessage(env, 'No spline init');
        return;
    end
    if ~isfield(env, 'optInfo') || ~env.optInfo.success || isempty(env.Popt)
        drawBSplinePath(env);
        if isfield(env, 'optInfo') && isfield(env.optInfo, 'message')
            drawCenteredMessage(env, sprintf('Opt failed: %s', shortenText(env.optInfo.message, 34)));
        else
            drawCenteredMessage(env, 'Opt failed');
        end
        return;
    end

    params = env.optInfo.params;
    params.enablePathSample = true;
    params.enablePointClearance = false;
    params.printEvalTiming = false;

    stateInit = evaluateCSSCGlobal(env.Pinit, env.obstacles, params);
    stateOpt = evaluateCSSCGlobal(env.Popt, env.obstacles, params);

    plot(stateInit.pathSample(:,1), stateInit.pathSample(:,2), '--', ...
        'Color',[0.55 0.55 0.55], 'LineWidth', 1.0, ...
        'HandleVisibility','off');
    plot(stateOpt.pathSample(:,1), stateOpt.pathSample(:,2), 'r-', ...
        'LineWidth', 1.8, 'HandleVisibility','off');
    plot(env.Popt(:,1), env.Popt(:,2), 'ro-', ...
        'MarkerSize', 3.0, 'LineWidth', 0.8, ...
        'HandleVisibility','off');

    text(env.bounds(1,1) + 0.02, env.bounds(2,2) - 0.04, ...
        sprintf('min %.3f', stateOpt.minClear), ...
        'Color',[0.75 0.05 0.05], 'FontWeight','bold', ...
        'HandleVisibility','off');
end

function drawCenteredMessage(env, msg)
    text(mean(env.bounds(1,:)), mean(env.bounds(2,:)), msg, ...
        'HorizontalAlignment','center', 'FontWeight','bold', ...
        'Color',[0.75 0.05 0.05], 'HandleVisibility','off');
end

function out = shortenText(txt, maxLen)
    txt = char(txt);
    if numel(txt) <= maxLen
        out = txt;
    else
        out = [txt(1:maxLen-3), '...'];
    end
end

function drawBounds(bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot([x(1), x(2), x(2), x(1), x(1)], ...
         [y(1), y(1), y(2), y(2), y(1)], ...
         'k-', 'LineWidth', 1.0, 'HandleVisibility','off');
end

function drawObstacles(obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(obs.type)
            case 'circle'
                drawCircle(obs.center, obs.radius, [0.20 0.35 0.80], 1.25, '-');

            case {'rect','rectangle','box'}
                drawRect(obs.center, obs.halfSize, obs.yaw, [0.25 0.25 0.25], 1.25, '-');
        end
    end
end

function drawObstacleLabels(obstacles, labels)
    if isempty(labels)
        return;
    end

    nLabel = min(numel(obstacles), numel(labels));
    for i = 1:nLabel
        text(obstacles(i).center(1), obstacles(i).center(2), labels{i}, ...
            'HorizontalAlignment','center', ...
            'VerticalAlignment','middle', ...
            'FontWeight','bold', ...
            'Color',[0.05 0.05 0.05], ...
            'HandleVisibility','off');
    end
end

function drawCircle(c, r, color, lw, style)
    th = linspace(0, 2*pi, 120);
    fill(c(1) + r*cos(th), c(2) + r*sin(th), color, ...
        'FaceAlpha',0.18, 'EdgeColor',color, 'LineWidth',lw, ...
        'LineStyle',style, 'HandleVisibility','off');
end

function drawRect(c, h, yaw, color, lw, style)
    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];

    pts = [
        -h(1), -h(2)
         h(1), -h(2)
         h(1),  h(2)
        -h(1),  h(2)
        -h(1), -h(2)
    ];

    pts = (R * pts.').' + c;
    fill(pts(:,1), pts(:,2), color, ...
        'FaceAlpha',0.18, 'EdgeColor',color, 'LineWidth',lw, ...
        'LineStyle',style, 'HandleVisibility','off');
end
