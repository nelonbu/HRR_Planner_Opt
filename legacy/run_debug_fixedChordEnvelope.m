clear; clc; close all;

%% Compare fixedChordEnvelope.m and fixedChordEnvelope_old.m
% This script isolates the fixed-chord envelope solver for timing and
% accuracy comparison. The path setup mirrors run_demo_cssc2d: path length
% about 1, L = 0.15, cubic B-spline, and the same control-point density rule.

try
    initCSSCProjectPath;
catch
    % If this script is already under a path-managed session, continue.
end

%% 1. Path and envelope parameters
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
opts.nVGrid = 180;
opts.epsV = 1e-6;
opts.tolDen = 1e-6;
opts.lambdaTol = 1e-9;

% New-solver options. The old solver ignores these fields.
opts.maxNewtonIter = 6;
opts.vResidualTol = 1e-5;
opts.vAcceptTol = 5e-4;
opts.fallbackNGrid = 30;
opts.enableFallback = true;

rFun  = @(w) evalBSplinePath2D(P, w, params.degree, params.knot);
drFun = @(w) evalBSplineDerivLocal(P, w, params.degree, params.knot);

%% 2. Single-run comparison
[envNew, tNew] = runEnvelope(@fixedChordEnvelope, rFun, drFun, params.L, opts);
[envOld, tOld] = runEnvelope(@fixedChordEnvelope_old, rFun, drFun, params.L, opts);

cmp = compareEnvelopes(envNew, envOld, params.L);

printEnvelopeSummary(envNew, tNew, params.L, 'new fixedChordEnvelope');
printEnvelopeSummary(envOld, tOld, params.L, 'old fixedChordEnvelope');
printComparisonSummary(cmp, tNew, tOld);

%% 3. Repeated timing comparison
nWarmup = 5;
nRepeat = 5;

for k = 1:nWarmup
    fixedChordEnvelope(rFun, drFun, params.L, opts);
    fixedChordEnvelope_old(rFun, drFun, params.L, opts);
end

timesNew = zeros(nRepeat, 1);
timesOld = zeros(nRepeat, 1);

for k = 1:nRepeat
    t = tic;
    fixedChordEnvelope(rFun, drFun, params.L, opts);
    timesNew(k) = toc(t);

    t = tic;
    fixedChordEnvelope_old(rFun, drFun, params.L, opts);
    timesOld(k) = toc(t);
end

fprintf('\n[repeated timing comparison]\n');
fprintf('  warmup runs              : %d\n', nWarmup);
fprintf('  repeated runs            : %d\n', nRepeat);
fprintf('  new mean / median        : %.3f / %.3f ms\n', ...
    1000 * mean(timesNew), 1000 * median(timesNew));
fprintf('  old mean / median        : %.3f / %.3f ms\n', ...
    1000 * mean(timesOld), 1000 * median(timesOld));
fprintf('  speedup mean / median    : %.2fx / %.2fx\n', ...
    mean(timesOld) / mean(timesNew), median(timesOld) / median(timesNew));
fprintf('  new min / max            : %.3f / %.3f ms\n', ...
    1000 * min(timesNew), 1000 * max(timesNew));
fprintf('  old min / max            : %.3f / %.3f ms\n', ...
    1000 * min(timesOld), 1000 * max(timesOld));

%% 4. Comparison figure
plotEnvelopeComparison(P, params, opts, envNew, envOld, cmp, timesNew, timesOld);

%% Local helpers

function [env, dt] = runEnvelope(funHandle, rFun, drFun, L, opts)
    t = tic;
    env = funHandle(rFun, drFun, L, opts);
    dt = toc(t);
end

function dpos = evalBSplineDerivLocal(P, w, degree, knot)
    [~, dpos] = evalBSplinePath2D(P, w, degree, knot);
end

function cmp = compareEnvelopes(envNew, envOld, L)
    commonLine = envNew.validLine & envOld.validLine ...
        & all(isfinite(envNew.M), 2) & all(isfinite(envOld.M), 2) ...
        & all(isfinite(envNew.N), 2) & all(isfinite(envOld.N), 2);
    commonSegment = commonLine & envNew.validSegment & envOld.validSegment ...
        & all(isfinite(envNew.G), 2) & all(isfinite(envOld.G), 2);
    commonV = commonLine & isfinite(envNew.v) & isfinite(envOld.v);

    cmp = struct();
    cmp.commonLine = commonLine;
    cmp.commonSegment = commonSegment;
    cmp.commonV = commonV;
    cmp.numCommonLine = sum(commonLine);
    cmp.numCommonSegment = sum(commonSegment);

    if any(commonSegment)
        gDiff = sqrt(sum((envNew.G(commonSegment,:) - envOld.G(commonSegment,:)).^2, 2));
        cmp.meanGDiff = mean(gDiff);
        cmp.maxGDiff = max(gDiff);
    else
        cmp.meanGDiff = nan;
        cmp.maxGDiff = nan;
    end

    if any(commonV)
        vDiff = abs(envNew.v(commonV) - envOld.v(commonV));
        cmp.meanVDiff = mean(vDiff);
        cmp.maxVDiff = max(vDiff);
    else
        cmp.meanVDiff = nan;
        cmp.maxVDiff = nan;
    end

    cmp.newResidual = chordResidual(envNew, L);
    cmp.oldResidual = chordResidual(envOld, L);
    cmp.newDL = cmp.newResidual;
    cmp.oldDL = cmp.oldResidual;
end

function residual = chordResidual(env, L)
    validLine = env.validLine & all(isfinite(env.M), 2) & all(isfinite(env.N), 2);
    chordLen = sqrt(sum((env.N(validLine,:) - env.M(validLine,:)).^2, 2));
    residual = chordLen - L;
end

function printEnvelopeSummary(env, dt, L, label)
    residual = chordResidual(env, L);

    fprintf('\n[%s]\n', label);
    fprintf('  total time               : %.3f ms\n', 1000 * dt);
    fprintf('  nU                       : %d\n', numel(env.u));
    fprintf('  validLine                : %d / %d\n', sum(env.validLine), numel(env.u));
    fprintf('  validSegment             : %d / %d\n', sum(env.validSegment), numel(env.u));

    if isfield(env, 'stats')
        fprintf('  Newton success           : %d\n', env.stats.numNewtonSuccess);
        fprintf('  fallback used            : %d\n', env.stats.numFallbackUsed);
        fprintf('  avg Newton iter          : %.3f\n', env.stats.avgNewtonIters);
    end

    if ~isempty(residual)
        fprintf('  dL=||N-M||-L mean abs    : %.3e\n', mean(abs(residual)));
        fprintf('  dL=||N-M||-L max abs     : %.3e\n', max(abs(residual)));
        fprintf('  dL=||N-M||-L signed min/max: %.3e / %.3e\n', min(residual), max(residual));
    end
end

function printComparisonSummary(cmp, tNew, tOld)
    fprintf('\n[accuracy / efficiency comparison]\n');
    fprintf('  old/new time ratio       : %.2fx\n', tOld / tNew);
    fprintf('  common validLine         : %d\n', cmp.numCommonLine);
    fprintf('  common validSegment      : %d\n', cmp.numCommonSegment);
    fprintf('  G diff mean / max        : %.3e / %.3e\n', cmp.meanGDiff, cmp.maxGDiff);
    fprintf('  v diff mean / max        : %.3e / %.3e\n', cmp.meanVDiff, cmp.maxVDiff);
    fprintf('  new dL mean abs / max abs: %.3e / %.3e\n', ...
        mean(abs(cmp.newDL)), max(abs(cmp.newDL)));
    fprintf('  old dL mean abs / max abs: %.3e / %.3e\n', ...
        mean(abs(cmp.oldDL)), max(abs(cmp.oldDL)));
    fprintf('  new dL signed min / max  : %.3e / %.3e\n', min(cmp.newDL), max(cmp.newDL));
    fprintf('  old dL signed min / max  : %.3e / %.3e\n', min(cmp.oldDL), max(cmp.oldDL));
end

function plotEnvelopeComparison(P, params, opts, envNew, envOld, cmp, timesNew, timesOld)
    fig = figure('Color','w','Name','fixedChordEnvelope new vs old','Position',[80 80 1350 820]);

    wPlot = linspace(opts.uRange(1), opts.uRange(2), 500).';
    pathSample = evalBSplinePath2D(P, wPlot, params.degree, params.knot);

    subplot(2,2,1); hold on; grid on; axis equal;
    title('Geometry comparison');
    xlabel('x'); ylabel('y');
    plot(pathSample(:,1), pathSample(:,2), 'k-', 'LineWidth', 2.0, ...
        'DisplayName', 'B-spline path');
    plot(P(:,1), P(:,2), 'ko-', 'MarkerFaceColor', 'w', ...
        'DisplayName', 'control points');
    drawSparseChords(envNew, 25, [0.75 0.75 0.75]);
    plotValidG(envOld, [0.85 0.25 0.15], 'old G');
    plotValidG(envNew, [0.1 0.45 0.85], 'new G');
    legend('Location','bestoutside');

    subplot(2,2,2); hold on; grid on;
    title('Envelope point error');
    xlabel('u'); ylabel('||G_{new} - G_{old}||');
    if any(cmp.commonSegment)
        u = envNew.u(cmp.commonSegment);
        gDiff = sqrt(sum((envNew.G(cmp.commonSegment,:) - envOld.G(cmp.commonSegment,:)).^2, 2));
        plot(u, gDiff, 'b-', 'LineWidth', 1.6);
    end

    subplot(2,2,3); hold on; grid on;
    title('v(u) comparison');
    xlabel('u'); ylabel('v');
    plot(envOld.u, envOld.v, 'r--', 'LineWidth', 1.4, 'DisplayName', 'old');
    plot(envNew.u, envNew.v, 'b-', 'LineWidth', 1.4, 'DisplayName', 'new');
    legend('Location','best');

    subplot(2,2,4); hold on; grid on;
    title('Chord length equation residual');
    xlabel('sample'); ylabel('dL = ||N-M|| - L');
    plot(cmp.oldDL, 'r--', 'LineWidth', 1.2, 'DisplayName', 'old dL');
    plot(cmp.newDL, 'b-', 'LineWidth', 1.2, 'DisplayName', 'new dL');
    legend('Location','best');

    try
        exportgraphics(fig, fullfile(pwd, 'fixedChordEnvelope_comparison.png'), 'Resolution', 180);
    catch
    end
end

function plotValidG(env, color, name)
    validG = env.validSegment & all(isfinite(env.G), 2);
    if any(validG)
        plot(env.G(validG,1), env.G(validG,2), '.', ...
            'Color', color, 'DisplayName', name);
    end
end

function drawSparseChords(env, nDraw, color)
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
                'Color', color, ...
                'HandleVisibility', vis, ...
                'DisplayName', 'new sample chords');
        end
    end
end
