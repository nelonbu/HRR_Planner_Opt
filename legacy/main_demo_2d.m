clear; clc; close all;

%% ============================================================
%  Main demo
%  Units: mm
% ============================================================

%% 1. Basic config
p_start = [0, 0];
p_goal  = [1000, 0];

N = 80;
L_pair = 150;

%% 2. Build obstacles and SDF map
mapConfig.xRange = [-150, 1150];
mapConfig.yRange = [-350, 350];

mapConfig.numObstacles = 12;
mapConfig.radiusRange = [50, 90];
mapConfig.startGoalSafeRadius = 180;
mapConfig.minObstacleGap = 20;
mapConfig.seed = 143;

[obstacles, sdfMap] = buildMapAndSDF2D(mapConfig, p_start, p_goal);

%% 3. Initial path
alpha = linspace(0, 1, N)';
P0 = (1 - alpha) .* p_start + alpha .* p_goal;

% Small perturbation to avoid symmetric local minimum
dir = p_goal - p_start;
dir = dir / norm(dir);
normal = [-dir(2), dir(1)];

perturbAmp = 15;
P0_init = P0;
P0_init(2:end-1, :) = P0_init(2:end-1, :) + ...
    perturbAmp * sin(pi * alpha(2:end-1)) .* normal;

%% 4. Optimizer config

% Pair construction
optConfig.L_pair = L_pair;

% Collision / clearance
optConfig.robotRadius = 5;      % line segment tube radius, mm
optConfig.d_min       = 20;     % hard safety clearance, mm
optConfig.d_pref      = 35;     % soft buffer clearance, mm
% 建议 d_pref 不要太大，否则会把路径过度推离障碍物。
% 这里取 d_min + 15 mm。

% Objective weights
optConfig.wObs    = 1200.0;     % hard obstacle penalty
optConfig.wClear  = 10.0;       % soft clearance buffer penalty
optConfig.wRef    = 0.02;       % reference path penalty
optConfig.wSmooth = 50.0;       % second-difference smoothness

% New adjacent spacing quality penalty
optConfig.wSpacingNear = 1.0;   % keep adjacent spacing close to reference
optConfig.wSpacingBand = 80.0;  % sharp penalty outside [0.5, 1.5] ratio

optConfig.spacingLowerRatio = 0.5;
optConfig.spacingUpperRatio = 1.5;

% Softplus smoothness parameters
optConfig.epsObs     = 3.0;     % mm
optConfig.epsClear   = 6.0;     % mm
optConfig.epsSpacing = 0.05;    % dimensionless ratio
optConfig.epsDist    = 1e-3;    % mm, smooth distance epsilon

% Segment sampling
optConfig.samplePerSeg = 8;

% Trust region
optConfig.rTrust = 180;         % mm

% Outer-inner optimization
optConfig.numOuter  = 15;
optConfig.innerIter = 200;

% Adam parameters
optConfig.lr       = 0.035;
optConfig.beta1    = 0.9;
optConfig.beta2    = 0.999;
optConfig.epsAdam  = 1e-8;
optConfig.gradClip = 500.0;

% Save snapshots instead of live plotting
optConfig.saveInterval = 40;

% For plotting diagnostic bounds only
avgInitSpacing = computePathLengthLocal(P0) / (N - 1);
optConfig.avgInitSpacing = avgInitSpacing;
optConfig.spacingLowerAbs = optConfig.spacingLowerRatio * avgInitSpacing;
optConfig.spacingUpperAbs = optConfig.spacingUpperRatio * avgInitSpacing;

% Timing debug
optConfig.enableTimingDebug = true;
optConfig.timingPrintInterval = 100;
optConfig.metricDenseQ = 50;

optConfig.metricEvalInterval = 20;
optConfig.metricDenseQ = 30;

%% 5. Optimize first, no live visualization
[Popt, info] = optimizePathSegments2D(P0_init, P0, sdfMap, optConfig);

%% 6. Save data
save('optimization_result_2d.mat', ...
    'P0', 'P0_init', 'Popt', 'info', 'obstacles', 'sdfMap', 'optConfig');

fprintf('\nSaved result to optimization_result_2d.mat\n');

%% 7. Replay visualization after optimization
replayOptimization2D(P0, P0_init, Popt, info, obstacles, optConfig);

%% 8. Plot histories
plotOptimizationHistory(info, optConfig);


%% ============================================================
%  Local visualization functions
% ============================================================

function replayOptimization2D(P0, P0_init, Popt, info, obstacles, optConfig)
    figure('Name', 'Optimization replay');
    hold on; grid on; axis equal;

    drawObstacles2D(obstacles);

    hPair = plot(nan, nan, '-', ...
        'Color', [0.60, 0.60, 0.60], ...
        'LineWidth', 0.8);

    hInit = plot(P0(:,1), P0(:,2), 'k--', ...
        'LineWidth', 1.4);

    hInitPert = plot(P0_init(:,1), P0_init(:,2), ...
        'Color', [0.2, 0.2, 0.2], ...
        'LineStyle', ':', ...
        'LineWidth', 1.2);

    hPath = plot(nan, nan, 'b-', ...
        'LineWidth', 2.2);

    hPts = plot(nan, nan, 'bo', ...
        'MarkerSize', 3);

    plot(P0(1,1), P0(1,2), 'go', ...
        'MarkerSize', 10, ...
        'LineWidth', 2);

    plot(P0(end,1), P0(end,2), 'ro', ...
        'MarkerSize', 10, ...
        'LineWidth', 2);

    xlim([-150, 1150]);
    ylim([-350, 350]);

    xlabel('x / mm');
    ylabel('y / mm');

    titleHandle = title('Optimization replay');

    legend([hPair, hInit, hInitPert, hPath, hPts], ...
        {'Pair segments', ...
         'Straight reference path', ...
         'Perturbed initial path', ...
         'Optimized path process', ...
         'Path points'}, ...
        'Location', 'bestoutside');

    for k = 1:numel(info.snapshots)
        P = info.snapshots{k};
        pairs = info.snapshotPairs{k};
        iter = info.snapshotIters(k);

        [xPair, yPair] = pairSegmentsToXY(P, pairs);

        set(hPair, 'XData', xPair, 'YData', yPair);
        set(hPath, 'XData', P(:,1), 'YData', P(:,2));
        set(hPts,  'XData', P(:,1), 'YData', P(:,2));

        if iter <= numel(info.Jhist)
            titleText = sprintf(['Replay iter %d | J = %.1f | clear = %.1f mm | ', ...
                'len = %.1f mm | adj = %.1f~%.1f mm'], ...
                iter, ...
                info.Jhist(iter), ...
                info.minClearHist(iter), ...
                info.lenHist(iter), ...
                info.minAdjDistHist(iter), ...
                info.maxAdjDistHist(iter));
        else
            titleText = sprintf('Replay iter %d', iter);
        end

        set(titleHandle, 'String', titleText);

        drawnow;
        pause(0.03);
    end

    % Final emphasize
    pairsFinal = info.pairsFinal;
    [xPair, yPair] = pairSegmentsToXY(Popt, pairsFinal);

    set(hPair, 'XData', xPair, 'YData', yPair);
    set(hPath, 'XData', Popt(:,1), 'YData', Popt(:,2));
    set(hPts,  'XData', Popt(:,1), 'YData', Popt(:,2));

    set(titleHandle, 'String', 'Final optimized result');
end


function plotOptimizationHistory(info, optConfig)
    figure('Name', 'Optimization history');

    subplot(5,1,1);
    plot(info.Jhist, 'LineWidth', 1.3);
    grid on;
    ylabel('Objective');

    subplot(5,1,2);
    plot(info.minClearHist, 'LineWidth', 1.3);
    hold on;
    yline(optConfig.d_min, 'r--', 'd_{min}');
    yline(optConfig.d_pref, 'k--', 'd_{pref}');
    grid on;
    ylabel('Clear / mm');

    subplot(5,1,3);
    plot(info.lenHist, 'LineWidth', 1.3);
    grid on;
    ylabel('Length / mm');

    subplot(5,1,4);
    plot(info.minAdjDistHist, 'LineWidth', 1.3);
    hold on;
    yline(optConfig.spacingLowerAbs, 'r--', '0.5 d_0');
    grid on;
    ylabel('Min adj / mm');

    subplot(5,1,5);
    plot(info.maxAdjDistHist, 'LineWidth', 1.3);
    hold on;
    yline(optConfig.spacingUpperAbs, 'r--', '1.5 d_0');
    grid on;
    xlabel('Iteration');
    ylabel('Max adj / mm');
end


function drawObstacles2D(obstacles)
    theta = linspace(0, 2*pi, 150);

    for k = 1:size(obstacles, 1)
        cx = obstacles(k, 1);
        cy = obstacles(k, 2);
        r  = obstacles(k, 3);

        x = cx + r * cos(theta);
        y = cy + r * sin(theta);

        fill(x, y, [0.85, 0.85, 0.85], ...
            'EdgeColor', [0.25, 0.25, 0.25], ...
            'LineWidth', 1.2);
    end
end


function [xAll, yAll] = pairSegmentsToXY(P, pairs)
    xAll = nan(3 * size(pairs, 1), 1);
    yAll = nan(3 * size(pairs, 1), 1);

    idx = 1;

    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        xAll(idx) = P(i, 1);
        yAll(idx) = P(i, 2);

        xAll(idx + 1) = P(j, 1);
        yAll(idx + 1) = P(j, 2);

        xAll(idx + 2) = nan;
        yAll(idx + 2) = nan;

        idx = idx + 3;
    end
end


function len = computePathLengthLocal(P)
    d = diff(P, 1, 1);
    len = sum(sqrt(sum(d.^2, 2)));
end