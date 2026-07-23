clear; clc; close all;

%% Debug scene 3: four-rectangle S-channel optimization
% This script focuses on the structured scene 3 only.
% Figure 1: B-spline initial path.
% Figure 2: optimized CSSC path.
% Figure 3: objective and objective-component histories.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

saveFigure = false;

%% 1. Scene 3 environment parameters

envOpts = struct();
envOpts.bounds = [0, 1; -0.4, 0.4];
envOpts.startPt = [0.05, 0.0];
envOpts.goalPt = [0.95, 0.0];
envOpts.dGap = 0.1;
envOpts.W = 0.64;
envOpts.H = envOpts.bounds(2,2) - envOpts.bounds(2,1);
envOpts.seed = 1199;%31;
envOpts.yInOutRange = [-0.20, 0.20];

[obstacles, envInfo] = generateCSSCStructuredEnvironment2D('fourRectSChannel', envOpts);
envInfo.obstacles = obstacles;

%% 2. RRT path and B-spline initialization

params = makeScene3OptimizationParams();
rrtOpts = makeScene3RRTOpts(envInfo);

[pathRRT, rrtInfo] = runRRTWithRestarts(envInfo, rrtOpts);
if ~rrtInfo.success
    error('Scene 3 RRT failed after %d restarts: %s', ...
        rrtOpts.numRestart, rrtInfo.message);
end

shortcutOpts = rrtOpts;
shortcutOpts.seed = 7003;
shortcutOpts.numShortcut = 200;
[pathShort, shortcutInfo] = shortcutPath2D(pathRRT, obstacles, shortcutOpts);

approxLen = polylineLength(pathShort);
nCtrl = max(params.degree + 1, ceil(approxLen / (0.5 * params.L)) + 1);
splineOpts = struct('degree', params.degree, 'nCtrl', nCtrl);
[Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, splineOpts);

Pref = Pinit;
params.knot = splineInfo.knot;

paramsEval = makePlotEvalParams(params);
stateInit = evaluateCSSCGlobal(Pinit, obstacles, paramsEval);

%% 3. CSSC optimization

fprintf('\n[scene 3 debug optimization]\n');
fprintf('  scene       : %s\n', envInfo.name);
fprintf('  RRT nodes   : %d\n', rrtInfo.numNodes);
fprintf('  shortcut    : %d -> %d nodes, accepted %d/%d\n', ...
    shortcutInfo.initialNodes, shortcutInfo.finalNodes, ...
    shortcutInfo.numAccepted, shortcutInfo.numAttempt);
fprintf('  B-spline    : degree=%d, nCtrl=%d\n', ...
    splineInfo.degree, splineInfo.nCtrl);
fprintf('  init minClr : %.6f\n\n', stateInit.minClear);

[Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, params);
stateOpt = evaluateCSSCGlobal(Popt, obstacles, paramsEval);

%% 4. Focused result and diagnostic figures

figInit = figure('Color','w', ...
    'Name','Scene 3 debug: B-spline initial path', ...
    'Position',[80 80 900 620]);
drawSceneBase(envInfo);
drawBSplineInitial(Pinit, splineInfo, stateInit);
title(sprintf('Scene 3 B-spline Init | minClear = %.4f, dMin = %.4f', ...
    stateInit.minClear, params.dMin), 'Interpreter','none');

figOpt = figure('Color','w', ...
    'Name','Scene 3 debug: optimized path', ...
    'Position',[140 120 900 620]);
drawSceneBase(envInfo);
drawOptimizedPath(Pinit, Popt, stateInit, stateOpt);
title(sprintf('Scene 3 Optimized | minClear = %.4f, dMin = %.4f, bestIter = %d, runIter = %d', ...
    stateOpt.minClear, params.dMin, optInfo.bestIter, optInfo.numIterActual), 'Interpreter','none');

figJ = figure('Color','w', ...
    'Name','Scene 3 debug: objective history', ...
    'Position',[200 160 900 620]);
drawObjectiveHistory(optInfo);

fprintf('\n[scene 3 debug summary]\n');
fprintf('  init minClear  : %.6f\n', stateInit.minClear);
fprintf('  final minClear : %.6f\n', stateOpt.minClear);
fprintf('  final J        : %.6g\n', optInfo.finalJ);
fprintf('  best iter/J    : %d / %.6g\n', optInfo.bestIter, optInfo.bestJ);
fprintf('  stop reason    : %s at iter %d\n', optInfo.stopReason, optInfo.numIterActual);

if saveFigure
    outDir = fullfile(projectRoot, 'results', 'tmp', 'scene3_s_channel_debug');
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    exportgraphics(figInit, fullfile(outDir, 'scene3_bspline_initial.png'), 'Resolution', 220);
    exportgraphics(figOpt, fullfile(outDir, 'scene3_optimized.png'), 'Resolution', 220);
    exportgraphics(figJ, fullfile(outDir, 'scene3_objective_history.png'), 'Resolution', 220);
    fprintf('[saved] %s\n', outDir);
end

%% Local parameters

function params = makeScene3OptimizationParams()
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
    params.printInterval = 20;
    params.saveInterval = 20;

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

function rrtOpts = makeScene3RRTOpts(envInfo)
    rrtOpts = struct();
    rrtOpts.bounds = envInfo.bounds;
    rrtOpts.stepSize = 0.030;
    rrtOpts.goalBias = 0.16;
    rrtOpts.goalTol = 0.035;
    rrtOpts.maxIter = 5000;
    rrtOpts.collisionResolution = 0.004;
    rrtOpts.inflateRadius = 0.0;
    rrtOpts.seed = 3003;
    rrtOpts.numRestart = 8;
end

function paramsEval = makePlotEvalParams(params)
    paramsEval = params;
    paramsEval.enablePathSample = true;
    paramsEval.pathSampleN = 600;
    paramsEval.enablePointClearance = false;
    paramsEval.enableObstacleMetadata = false;
    paramsEval.printEvalTiming = false;
end

%% Local RRT helpers

function [path, info] = runRRTWithRestarts(envInfo, rrtOpts)
    path = zeros(0, 2);
    info = struct('success', false, 'message', 'RRT failed.', ...
        'numIter', 0, 'numNodes', 0, 'numRestartUsed', 0);

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

%% Figure helpers

function drawSceneBase(envInfo)
    hold on; grid on; axis equal;
    xlabel('x'); ylabel('y');
    drawBounds(envInfo.bounds);
    drawObstacles(envInfo.obstacles);
    drawObstacleLabels(envInfo.obstacles, envInfo.obstacleLabels);
    plot(envInfo.startPt(1), envInfo.startPt(2), 'go', ...
        'MarkerFaceColor','g', 'DisplayName','start');
    plot(envInfo.goalPt(1), envInfo.goalPt(2), 'rp', ...
        'MarkerFaceColor','r', 'DisplayName','goal');
    xlim(envInfo.bounds(1,:));
    ylim(envInfo.bounds(2,:));
end

function drawBSplineInitial(Pinit, splineInfo, stateInit)
    if isempty(stateInit.pathSample)
        wPlot = linspace(0, 1, 600);
        curve = evalBSplinePath2D(Pinit, wPlot, splineInfo.degree, splineInfo.knot);
    else
        curve = stateInit.pathSample;
    end

    plot(Pinit(:,1), Pinit(:,2), 'ko--', ...
        'MarkerSize', 4.0, 'LineWidth', 0.8, ...
        'DisplayName','control polygon');
    plot(curve(:,1), curve(:,2), 'r-', ...
        'LineWidth', 2.0, 'DisplayName','B-spline init');
    drawMinClearPoint(stateInit, [0.85 0.05 0.05]);
    legend('Location','best');
end

function drawOptimizedPath(Pinit, Popt, stateInit, stateOpt)
    plot(stateInit.pathSample(:,1), stateInit.pathSample(:,2), '--', ...
        'Color',[0.55 0.55 0.55], 'LineWidth', 1.2, ...
        'DisplayName','B-spline init');
    plot(Pinit(:,1), Pinit(:,2), 'o--', ...
        'Color',[0.55 0.55 0.55], 'MarkerSize', 3.0, ...
        'LineWidth', 0.7, 'HandleVisibility','off');

    plot(Popt(:,1), Popt(:,2), 'ro-', ...
        'MarkerSize', 4.0, 'LineWidth', 0.8, ...
        'DisplayName','optimized control');
    plot(stateOpt.pathSample(:,1), stateOpt.pathSample(:,2), 'r-', ...
        'LineWidth', 2.0, 'DisplayName','optimized path');
    drawMinClearPoint(stateOpt, [0.85 0.05 0.05]);
    legend('Location','best');
end

function drawObjectiveHistory(info)
    iter = (1:numel(info.Jhist)).';

    tiledlayout(2, 1, 'Padding','compact', 'TileSpacing','compact');

    nexttile;
    plot(iter, info.Jhist, 'k-', 'LineWidth', 1.8, 'DisplayName','J');
    hold on;
    if isfield(info, 'bestIter') && isfield(info, 'bestJ') && ...
       info.bestIter >= 1 && info.bestIter <= numel(info.Jhist) && isfinite(info.bestJ)
        plot(info.bestIter, info.bestJ, 'rp', ...
            'MarkerSize', 11, 'MarkerFaceColor','r', ...
            'DisplayName','best J');
    end
    grid on;
    xlabel('iteration');
    ylabel('J');
    title('Total Objective');
    legend('Location','best');

    nexttile;
    hold on; grid on;
    componentFields = getObjectiveComponentFields(info);
    for k = 1:numel(componentFields)
        f = componentFields{k};
        histName = [f 'Hist'];
        if ~isfield(info, histName)
            continue;
        end

        y = info.(histName);
        if isempty(y) || all(isnan(y))
            continue;
        end

        plot(iter, y(:), 'LineWidth', 1.25, 'DisplayName', f);
    end
    xlabel('iteration');
    ylabel('component value');
    title('Objective Components');
    legend('Location','best', 'Interpreter','none');
end

function componentFields = getObjectiveComponentFields(info)
    if isfield(info, 'objectiveComponentFields') && ~isempty(info.objectiveComponentFields)
        componentFields = info.objectiveComponentFields;
    else
        componentFields = {'Jobs','Jclear','Jreg','Jref','Jsmooth','Jlen','Jtrust'};
    end
end

function drawMinClearPoint(state, color)
    if isfield(state, 'minPoint') && all(isfinite(state.minPoint))
        plot(state.minPoint(1), state.minPoint(2), 'p', ...
            'MarkerSize', 11, 'MarkerFaceColor', color, ...
            'MarkerEdgeColor','k', 'DisplayName','min clearance');
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
