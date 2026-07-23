clear; clc; close all;

%% Debug segment-obstacle clearance timing
% This script profiles the segment-clearance loop used by evaluateCSSCGlobal:
%
%   for each valid chord [M,N]
%       querySegmentObstacleClearance2D(M, N, obstacles)
%
% It uses the same small-scale B-spline path and obstacle style as
% run_demo_cssc2d, then prints timing percentages for the query internals.

try
    initCSSCProjectPath;
catch
    % If this script is already under a path-managed session, continue.
end

%% 1. Demo-like path and obstacles
obstacles = [
    obsRect2D([0.50,  0.08], [0.10, 0.125], 0.0)
    obsRect2D([0.50, -0.24], [0.10, 0.125], 0.0)
];

params = struct();
params.L = 0.15;
params.degree = 3;

startPt = [0.0, 0.0];
goalPt  = [1.0, 0.0];

pathLenApprox = norm(goalPt - startPt);
hCtrl = 0.5 * params.L;
nCtrl = max(params.degree + 1, ceil(pathLenApprox / hCtrl) + 1);

x = linspace(startPt(1), goalPt(1), nCtrl).';
y = 0.015 * sin(2*pi*(x - x(1)) / (x(end) - x(1)));
P = [x, y];
P(1,:) = startPt;
P(end,:) = goalPt;

params.knot = makeClampedUniformKnot(size(P,1), params.degree);

opts = struct();
opts.uRange = [0, 1];
opts.vSearchRange = [0, 1];
opts.nU = 140;
opts.epsV = 1e-6;
opts.tolDen = 1e-6;
opts.maxNewtonIter = 6;
opts.vResidualTol = 1e-5;
opts.vAcceptTol = 5e-4;
opts.fallbackNGrid = 30;
opts.enableFallback = true;

rFun  = @(w) evalBSplinePath2D(P, w, params.degree, params.knot);
drFun = @(w) evalBSplineDerivLocal(P, w, params.degree, params.knot);

env = fixedChordEnvelope(rFun, drFun, params.L, opts);
validLineMask = env.validLine & all(isfinite(env.M), 2) & all(isfinite(env.N), 2);
idxValidLine = find(validLineMask);

fprintf('\n[segment clearance timing setup]\n');
fprintf('  nCtrl                    : %d\n', nCtrl);
fprintf('  nU                       : %d\n', opts.nU);
fprintf('  validLine                : %d / %d\n', numel(idxValidLine), numel(env.u));
fprintf('  obstacles                : %d\n', numel(obstacles));
fprintf('  segment-obstacle queries : %d\n', numel(idxValidLine) * numel(obstacles));

%% 2. Baseline loop timing without instrumentation
nWarmup = 3;
nRepeat = 10;

for k = 1:nWarmup
    runSegmentClearanceLoop(env, idxValidLine, obstacles, false);
end

baselineTimes = zeros(nRepeat, 1);
for k = 1:nRepeat
    t = tic;
    runSegmentClearanceLoop(env, idxValidLine, obstacles, false);
    baselineTimes(k) = toc(t);
end

fprintf('\n[baseline loop timing]\n');
fprintf('  repeats                  : %d\n', nRepeat);
fprintf('  mean / median            : %.3f / %.3f ms\n', ...
    1000 * mean(baselineTimes), 1000 * median(baselineTimes));
fprintf('  min / max                : %.3f / %.3f ms\n', ...
    1000 * min(baselineTimes), 1000 * max(baselineTimes));

%% 3. Instrumented timing breakdown
nProfileRepeat = 5;
profileTimes = zeros(nProfileRepeat, 1);
timingSum = emptyTimingSum();

for k = 1:nProfileRepeat
    t = tic;
    [~, timingRun] = runSegmentClearanceLoop(env, idxValidLine, obstacles, true);
    profileTimes(k) = toc(t);
    timingSum = addTiming(timingSum, timingRun);
end

fprintf('\n[instrumented loop timing]\n');
fprintf('  repeats                  : %d\n', nProfileRepeat);
fprintf('  measured mean / median   : %.3f / %.3f ms\n', ...
    1000 * mean(profileTimes), 1000 * median(profileTimes));
fprintf('  accumulated query total  : %.3f ms\n', 1000 * timingSum.total);
fprintf('  avg candidates / rect    : %.2f\n', ...
    timingSum.nRectCandidates / max(1, timingSum.nRect));

printTimingBreakdown(timingSum);

%% 4. Optional geometry plot
doPlot = true;
if doPlot
    figure('Color','w','Name','segment clearance timing debug','Position',[100 100 900 520]);
    hold on; grid on; axis equal;
    title('Segment clearance timing sample');
    xlabel('x'); ylabel('y');

    drawObstacles(obstacles);

    wPlot = linspace(opts.uRange(1), opts.uRange(2), 500).';
    pathSample = evalBSplinePath2D(P, wPlot, params.degree, params.knot);
    plot(pathSample(:,1), pathSample(:,2), 'k-', 'LineWidth', 2.0, ...
        'DisplayName', 'B-spline path');
    plot(P(:,1), P(:,2), 'ro-', 'MarkerFaceColor', 'r', ...
        'DisplayName', 'control points');

    drawSparseChords(env, 35);
    legend('Location','bestoutside');
end

%% Local helpers

function [state, timingSum] = runSegmentClearanceLoop(env, idxValidLine, obstacles, enableTiming)
    n = numel(env.u);
    clearanceSegment = nan(n,1);
    closestPoint = nan(n,2);
    closestAlpha = nan(n,1);
    closestNormal = nan(n,2);
    nearestObsId = nan(n,1);

    timingSum = emptyTimingSum();

    for kk = 1:numel(idxValidLine)
        i = idxValidLine(kk);

        out = querySegmentObstacleClearance2D( ...
            env.M(i,:), env.N(i,:), obstacles, enableTiming);

        clearanceSegment(i) = out.clearance;
        closestPoint(i,:) = out.closestPoint;
        closestAlpha(i) = out.alpha;
        closestNormal(i,:) = out.normal;
        nearestObsId(i) = out.obsId;

        if enableTiming && isfield(out, 'timing')
            timingSum = addTiming(timingSum, out.timing);
        end
    end

    state = struct();
    state.clearanceSegment = clearanceSegment;
    state.closestPoint = closestPoint;
    state.closestAlpha = closestAlpha;
    state.closestNormal = closestNormal;
    state.nearestObsId = nearestObsId;
end

function dpos = evalBSplineDerivLocal(P, w, degree, knot)
    [~, dpos] = evalBSplinePath2D(P, w, degree, knot);
end

function timing = emptyTimingSum()
    timing.total = 0;
    timing.setup = 0;
    timing.typeDispatch = 0;
    timing.circle = 0;
    timing.rect = 0;
    timing.rectSetup = 0;
    timing.rectCandidates = 0;
    timing.rectFilterUnique = 0;
    timing.rectSDFLoop = 0;
    timing.rectFinalize = 0;
    timing.generic = 0;
    timing.outputPack = 0;
    timing.nCircle = 0;
    timing.nRect = 0;
    timing.nGeneric = 0;
    timing.nRectCandidates = 0;
end

function out = addTiming(a, b)
    out = a;
    f = fieldnames(a);
    for i = 1:numel(f)
        if isfield(b, f{i})
            out.(f{i}) = out.(f{i}) + b.(f{i});
        end
    end
end

function printTimingBreakdown(timing)
    total = max(timing.total, eps);

    fprintf('\n[querySegmentObstacleClearance2D breakdown]\n');
    printLine('setup M/N/e', timing.setup, total);
    printLine('type dispatch', timing.typeDispatch, total);
    printLine('circle total', timing.circle, total);
    printLine('rect total', timing.rect, total);
    printLine('  rect setup/rotate', timing.rectSetup, total);
    printLine('  rect candidates', timing.rectCandidates, total);
    printLine('  rect filter/dedup', timing.rectFilterUnique, total);
    printLine('  rect SDF loop', timing.rectSDFLoop, total);
    printLine('  rect finalize', timing.rectFinalize, total);
    printLine('generic total', timing.generic, total);
    printLine('output struct pack', timing.outputPack, total);
    fprintf('  calls by type            : circle=%d rect=%d generic=%d\n', ...
        timing.nCircle, timing.nRect, timing.nGeneric);
end

function printLine(name, value, total)
    fprintf('  %-22s : %8.3f ms  (%5.1f%%)\n', ...
        name, 1000 * value, 100 * value / total);
end

function drawObstacles(obstacles)
    for k = 1:numel(obstacles)
        obs = obstacles(k);
        switch lower(obs.type)
            case 'circle'
                drawCircle(obs.center, obs.radius, [0.2 0.2 0.2], 1.5, '-');
            case {'rect', 'rectangle', 'box'}
                drawRect(obs.center, obs.halfSize, obs.yaw, [0.2 0.2 0.2], 1.5, '-');
        end
    end
end

function drawSparseChords(env, nDraw)
    idx = find(env.validLine);
    if isempty(idx)
        return;
    end

    skip = max(1, floor(numel(idx) / nDraw));
    first = true;

    for ii = 1:skip:numel(idx)
        k = idx(ii);
        M = env.M(k,:);
        N = env.N(k,:);
        if all(isfinite(M)) && all(isfinite(N))
            if first
                vis = 'on';
                first = false;
            else
                vis = 'off';
            end
            plot([M(1), N(1)], [M(2), N(2)], '-', ...
                'Color', [0.75 0.75 0.75], ...
                'HandleVisibility', vis, ...
                'DisplayName', 'sample chords');
        end
    end
end

function drawCircle(c, r, color, lw, style)
    th = linspace(0, 2*pi, 160);
    plot(c(1) + r*cos(th), c(2) + r*sin(th), ...
        'Color', color, 'LineWidth', lw, 'LineStyle', style, ...
        'HandleVisibility','off');
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

    plot(pts(:,1), pts(:,2), ...
        'Color', color, 'LineWidth', lw, 'LineStyle', style, ...
        'HandleVisibility','off');
end
