clear; clc; close all;

%% ============================================================
% Multi-case comparison: fixedChordEnvelope.m vs fixedChordEnvelope_old.m
%
% Tests:
%   1. Different curve shapes
%   2. Different chord lengths L
%   3. Different nU
%   4. Repeated timing
%   5. Accuracy comparison: residual, v error, G error
%% ============================================================

try
    initCSSCProjectPath;
catch
end

%% 1. Global settings
degree = 3;
startPt = [0.0, 0.0];
goalPt  = [1.0, 0.0];

nWarmup = 3;
nRepeat = 8;

baseOpts = struct();
baseOpts.uRange = [0, 1];
baseOpts.vSearchRange = [0, 1];
baseOpts.nVGrid = 180;
baseOpts.epsV = 1e-6;
baseOpts.tolDen = 1e-6;
baseOpts.lambdaTol = 1e-9;

% New-solver options. Old solver ignores these fields.
baseOpts.maxNewtonIter = 2;
baseOpts.vResidualTol = 1e-5;
baseOpts.vAcceptTol = 5e-4;
baseOpts.newtonTriggerTol = 5e-4;
baseOpts.maxNewtonStep = 0.20;
baseOpts.fallbackNGrid = 30;
baseOpts.enableFallback = true;

%% 2. Test cases
cases = makeTestCases();

fprintf('\n============================================================\n');
fprintf('Multi-case fixedChordEnvelope comparison\n');
fprintf('cases = %d, repeat = %d, warmup = %d\n', numel(cases), nRepeat, nWarmup);
fprintf('============================================================\n\n');

results = struct([]);
caseData = cell(numel(cases), 1);

for c = 1:numel(cases)
    spec = cases(c);

    %% Build B-spline path
    params = struct();
    params.L = spec.L;
    params.degree = degree;

    pathLenApprox = norm(goalPt - startPt);
    hCtrl = 0.5 * params.L;
    nCtrl = max(params.degree + 1, ceil(pathLenApprox / hCtrl) + 1);

    P = buildPath(spec, nCtrl, startPt, goalPt);
    params.knot = makeClampedUniformKnot(size(P,1), params.degree);

    opts = baseOpts;
    opts.nU = spec.nU;

    rFun  = @(w) evalBSplinePath2D(P, w, params.degree, params.knot);
    drFun = @(w) evalBSplineDerivLocal(P, w, params.degree, params.knot);

    %% Single outputs for accuracy comparison
    envNew = fixedChordEnvelope(rFun, drFun, params.L, opts);
    envOld = fixedChordEnvelope_old(rFun, drFun, params.L, opts);

    cmp = compareEnvelopes(envNew, envOld, params.L);

    %% Repeated timing
    timesNew = runTiming(@fixedChordEnvelope, rFun, drFun, params.L, opts, nWarmup, nRepeat);
    timesOld = runTiming(@fixedChordEnvelope_old, rFun, drFun, params.L, opts, nWarmup, nRepeat);

    %% Store result
    results(c).caseId = c;
    results(c).caseName = spec.name;
    results(c).shape = spec.shape;
    results(c).L = spec.L;
    results(c).amp = spec.amp;
    results(c).freq = spec.freq;
    results(c).nU = spec.nU;
    results(c).nCtrl = nCtrl;

    results(c).newMeanMs = 1000 * mean(timesNew);
    results(c).newMedianMs = 1000 * median(timesNew);
    results(c).oldMeanMs = 1000 * mean(timesOld);
    results(c).oldMedianMs = 1000 * median(timesOld);
    results(c).speedupMean = mean(timesOld) / mean(timesNew);
    results(c).speedupMedian = median(timesOld) / median(timesNew);

    results(c).newValidLine = sum(envNew.validLine);
    results(c).oldValidLine = sum(envOld.validLine);
    results(c).newValidSegment = sum(envNew.validSegment);
    results(c).oldValidSegment = sum(envOld.validSegment);
    results(c).validLineMismatch = nnz(envNew.validLine ~= envOld.validLine);
    results(c).validSegmentMismatch = nnz(envNew.validSegment ~= envOld.validSegment);

    results(c).commonLine = cmp.numCommonLine;
    results(c).commonSegment = cmp.numCommonSegment;

    results(c).newMeanAbsDL = safeMeanAbs(cmp.newDLFull);
    results(c).oldMeanAbsDL = safeMeanAbs(cmp.oldDLFull);
    results(c).newMaxAbsDL = safeMaxAbs(cmp.newDLFull);
    results(c).oldMaxAbsDL = safeMaxAbs(cmp.oldDLFull);

    results(c).meanGDiff = cmp.meanGDiff;
    results(c).maxGDiff = cmp.maxGDiff;
    results(c).meanVDiff = cmp.meanVDiff;
    results(c).maxVDiff = cmp.maxVDiff;

    caseData{c} = struct( ...
        'spec', spec, ...
        'params', params, ...
        'opts', opts, ...
        'P', P, ...
        'envNew', envNew, ...
        'envOld', envOld, ...
        'cmp', cmp, ...
        'timesNew', timesNew, ...
        'timesOld', timesOld);

    printOneCaseSummary(results(c), envNew, envOld);
end

%% 3. Summary table
T = struct2table(results);

fprintf('\n==================== Summary table ====================\n');
disp(T(:, {
    'caseId','caseName','L','nU','nCtrl', ...
    'newMedianMs','oldMedianMs','speedupMedian', ...
    'newValidLine','oldValidLine', ...
    'newMaxAbsDL','oldMaxAbsDL', ...
    'maxGDiff','maxVDiff'}));

%% 4. Plot summary
outDir = fullfile(pwd, 'results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

plotMultiCaseSummary(T, outDir);

%% 5. Plot worst cases
% Worst by new chord residual
[~, idWorstResidual] = max(T.newMaxAbsDL);
plotDetailedCase(caseData{idWorstResidual}, outDir, ...
    sprintf('worst residual case %d', idWorstResidual), ...
    'fixedChordEnvelope_worst_residual.png');

% Worst by G difference
[~, idWorstG] = max(T.maxGDiff);
plotDetailedCase(caseData{idWorstG}, outDir, ...
    sprintf('worst G-diff case %d', idWorstG), ...
    'fixedChordEnvelope_worst_Gdiff.png');

fprintf('\nFigures saved to:\n');
fprintf('  %s\n', fullfile(outDir, 'fixedChordEnvelope_multi_summary.png'));
fprintf('  %s\n', fullfile(outDir, 'fixedChordEnvelope_worst_residual.png'));
fprintf('  %s\n', fullfile(outDir, 'fixedChordEnvelope_worst_Gdiff.png'));

%% ============================================================
% Local functions
%% ============================================================

function cases = makeTestCases()
    % Preallocate empty struct array with the same fields as makeCase output.
    cases = repmat(makeCase('', '', NaN, NaN, NaN, NaN, NaN), 0, 1);

    cases(end+1) = makeCase('straight-L015-n140',       'sine',    0.15, 0.000, 1.0, 140, 1);
    cases(end+1) = makeCase('mild-sine-L015-n140',      'sine',    0.15, 0.015, 1.0, 140, 1);
    cases(end+1) = makeCase('medium-sine-L015-n140',    'sine',    0.15, 0.040, 1.5, 140, 1);
    cases(end+1) = makeCase('high-sine-L015-n140',      'sine',    0.15, 0.080, 2.0, 140, 1);

    cases(end+1) = makeCase('mild-sine-L010-n140',      'sine',    0.10, 0.030, 1.5, 140, 1);
    cases(end+1) = makeCase('mild-sine-L022-n140',      'sine',    0.22, 0.030, 1.5, 140, 1);

    cases(end+1) = makeCase('medium-sine-L015-n080',    'sine',    0.15, 0.040, 1.5,  80, 1);
    cases(end+1) = makeCase('medium-sine-L015-n220',    'sine',    0.15, 0.040, 1.5, 220, 1);

    cases(end+1) = makeCase('mixed-wave-L015-n140',     'mixed',   0.15, 0.040, 1.5, 140, 1);
    cases(end+1) = makeCase('mixed-wave-high-L015',     'mixed',   0.15, 0.080, 2.0, 140, 1);

    cases(end+1) = makeCase('random-smooth-1-L015',     'random',  0.15, 0.035, 1.0, 140, 11);
    cases(end+1) = makeCase('random-smooth-2-L015',     'random',  0.15, 0.060, 1.0, 140, 22);
end

function s = makeCase(name, shape, L, amp, freq, nU, seed)
    s = struct();
    s.name = name;
    s.shape = shape;
    s.L = L;
    s.amp = amp;
    s.freq = freq;
    s.nU = nU;
    s.seed = seed;
end

function P = buildPath(spec, nCtrl, startPt, goalPt)
    x = linspace(startPt(1), goalPt(1), nCtrl).';
    tau = (x - x(1)) / (x(end) - x(1));

    switch lower(spec.shape)
        case 'sine'
            y = spec.amp * sin(2*pi*spec.freq*tau);

        case 'mixed'
            y = spec.amp * ( ...
                0.70 * sin(2*pi*spec.freq*tau) + ...
                0.35 * sin(2*pi*(spec.freq + 1.0)*tau + 0.7));

        case 'random'
            rng(spec.seed);
            raw = randn(nCtrl, 1);
            kernel = [1; 2; 3; 2; 1];
            kernel = kernel / sum(kernel);
            y = spec.amp * conv(raw, kernel, 'same');

            % Force endpoints toward zero smoothly.
            window = sin(pi*tau);
            y = y .* window;

        otherwise
            error('Unknown shape: %s', spec.shape);
    end

    P = [x, y];
    P(1,:) = startPt;
    P(end,:) = goalPt;
end

function times = runTiming(funHandle, rFun, drFun, L, opts, nWarmup, nRepeat)
    for k = 1:nWarmup
        funHandle(rFun, drFun, L, opts);
    end

    times = zeros(nRepeat, 1);

    for k = 1:nRepeat
        t = tic;
        funHandle(rFun, drFun, L, opts);
        times(k) = toc(t);
    end
end

function dpos = evalBSplineDerivLocal(P, w, degree, knot)
    [~, dpos] = evalBSplinePath2D(P, w, degree, knot);
end

function cmp = compareEnvelopes(envNew, envOld, L)
    commonLine = envNew.validLine & envOld.validLine ...
        & all(isfinite(envNew.M), 2) ...
        & all(isfinite(envOld.M), 2) ...
        & all(isfinite(envNew.N), 2) ...
        & all(isfinite(envOld.N), 2);

    commonSegment = commonLine ...
        & envNew.validSegment ...
        & envOld.validSegment ...
        & all(isfinite(envNew.G), 2) ...
        & all(isfinite(envOld.G), 2);

    commonV = commonLine ...
        & isfinite(envNew.v) ...
        & isfinite(envOld.v);

    cmp = struct();
    cmp.commonLine = commonLine;
    cmp.commonSegment = commonSegment;
    cmp.commonV = commonV;
    cmp.numCommonLine = sum(commonLine);
    cmp.numCommonSegment = sum(commonSegment);

    cmp.newDLFull = chordResidualFull(envNew, L);
    cmp.oldDLFull = chordResidualFull(envOld, L);

    cmp.GDiffFull = nan(size(envNew.u));
    if any(commonSegment)
        cmp.GDiffFull(commonSegment) = sqrt(sum( ...
            (envNew.G(commonSegment,:) - envOld.G(commonSegment,:)).^2, 2));
        cmp.meanGDiff = mean(cmp.GDiffFull(commonSegment), 'omitnan');
        cmp.maxGDiff = max(cmp.GDiffFull(commonSegment), [], 'omitnan');
    else
        cmp.meanGDiff = nan;
        cmp.maxGDiff = nan;
    end

    cmp.vDiffFull = nan(size(envNew.u));
    if any(commonV)
        cmp.vDiffFull(commonV) = abs(envNew.v(commonV) - envOld.v(commonV));
        cmp.meanVDiff = mean(cmp.vDiffFull(commonV), 'omitnan');
        cmp.maxVDiff = max(cmp.vDiffFull(commonV), [], 'omitnan');
    else
        cmp.meanVDiff = nan;
        cmp.maxVDiff = nan;
    end
end

function residualFull = chordResidualFull(env, L)
    residualFull = nan(size(env.u));

    validLine = env.validLine ...
        & all(isfinite(env.M), 2) ...
        & all(isfinite(env.N), 2);

    if any(validLine)
        chordLen = sqrt(sum((env.N(validLine,:) - env.M(validLine,:)).^2, 2));
        residualFull(validLine) = chordLen - L;
    end
end

function v = safeMeanAbs(x)
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = mean(abs(x));
    end
end

function v = safeMaxAbs(x)
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = max(abs(x));
    end
end

function printOneCaseSummary(R, envNew, envOld)
    fprintf('\n[case %02d] %s\n', R.caseId, R.caseName);
    fprintf('  L=%.3f, nU=%d, nCtrl=%d\n', R.L, R.nU, R.nCtrl);
    fprintf('  new/old median time   : %.3f / %.3f ms\n', R.newMedianMs, R.oldMedianMs);
    fprintf('  speedup median        : %.2fx\n', R.speedupMedian);
    fprintf('  validLine new/old     : %d / %d\n', sum(envNew.validLine), sum(envOld.validLine));
    fprintf('  validSegment new/old  : %d / %d\n', sum(envNew.validSegment), sum(envOld.validSegment));
    fprintf('  dL max abs new/old    : %.3e / %.3e\n', R.newMaxAbsDL, R.oldMaxAbsDL);
    fprintf('  G diff mean/max       : %.3e / %.3e\n', R.meanGDiff, R.maxGDiff);

    if isfield(envNew, 'stats')
        if isfield(envNew.stats, 'numNewtonUsed')
            fprintf('  new Newton used       : %d\n', envNew.stats.numNewtonUsed);
        end
        fprintf('  new Newton success    : %d\n', envNew.stats.numNewtonSuccess);
        fprintf('  new avg Newton iter   : %.3f\n', envNew.stats.avgNewtonIters);
    end
end

function plotMultiCaseSummary(T, outDir)
    ids = T.caseId;
    labels = cellstr(num2str(ids));

    fig = figure('Color','w','Name','fixedChordEnvelope multi-case summary', ...
        'Position',[80 80 1400 780]);

    subplot(2,2,1); hold on; grid on;
    data = [T.oldMedianMs, T.newMedianMs];
    data = max(data, 1e-12);
    bar(ids, data);
    set(gca, 'YScale', 'log');
    xlabel('case id');
    ylabel('median time (ms, log)');
    title('Runtime');
    legend('old', 'new', 'Location', 'best');
    xticks(ids); xticklabels(labels);

    subplot(2,2,2); hold on; grid on;
    bar(ids, T.speedupMedian);
    xlabel('case id');
    ylabel('speedup old/new');
    title('Speedup');
    xticks(ids); xticklabels(labels);

    subplot(2,2,3); hold on; grid on;
    data = [T.oldMaxAbsDL, T.newMaxAbsDL];
    data = max(data, 1e-12);
    bar(ids, data);
    set(gca, 'YScale', 'log');
    xlabel('case id');
    ylabel('max |dL|');
    title('Chord length residual');
    legend('old', 'new', 'Location', 'best');
    xticks(ids); xticklabels(labels);

    subplot(2,2,4); hold on; grid on;
    data = [T.maxVDiff, T.maxGDiff];
    data = max(data, 1e-12);
    bar(ids, data);
    set(gca, 'YScale', 'log');
    xlabel('case id');
    ylabel('error vs old');
    title('New vs old output difference');
    legend('max |v diff|', 'max ||G diff||', 'Location', 'best');
    xticks(ids); xticklabels(labels);

    exportgraphics(fig, fullfile(outDir, 'fixedChordEnvelope_multi_summary.png'), ...
        'Resolution', 180);
end

function plotDetailedCase(data, outDir, figTitle, fileName)
    P = data.P;
    params = data.params;
    opts = data.opts;
    envNew = data.envNew;
    envOld = data.envOld;
    cmp = data.cmp;

    fig = figure('Color','w','Name',figTitle, ...
        'Position',[100 80 1400 820]);

    wPlot = linspace(opts.uRange(1), opts.uRange(2), 500).';
    pathSample = evalBSplinePath2D(P, wPlot, params.degree, params.knot);

    subplot(2,2,1); hold on; grid on; axis equal;
    title(['Geometry: ' data.spec.name], 'Interpreter','none');
    xlabel('x'); ylabel('y');
    plot(pathSample(:,1), pathSample(:,2), 'k-', 'LineWidth', 2.0, ...
        'DisplayName','B-spline path');
    plot(P(:,1), P(:,2), 'ko-', 'MarkerFaceColor','w', ...
        'DisplayName','control points');
    drawSparseChords(envNew, 25, [0.75 0.75 0.75]);
    plotValidG(envOld, [0.85 0.25 0.15], 'old G');
    plotValidG(envNew, [0.1 0.45 0.85], 'new G');
    legend('Location','bestoutside');

    subplot(2,2,2); hold on; grid on;
    title('v(u)');
    xlabel('u'); ylabel('v');
    plot(envOld.u, envOld.v, 'r--', 'LineWidth', 1.3, 'DisplayName','old');
    plot(envNew.u, envNew.v, 'b-', 'LineWidth', 1.3, 'DisplayName','new');
    legend('Location','best');

    subplot(2,2,3); hold on; grid on;
    title('Chord length residual');
    xlabel('u'); ylabel('dL = ||N-M|| - L');
    plot(envOld.u, cmp.oldDLFull, 'r--', 'LineWidth', 1.3, 'DisplayName','old');
    plot(envNew.u, cmp.newDLFull, 'b-', 'LineWidth', 1.3, 'DisplayName','new');
    yline(0, 'k:');
    legend('Location','best');

    subplot(2,2,4); hold on; grid on;
    title('New vs old difference');
    xlabel('u'); ylabel('error');
    semilogy(envNew.u, max(cmp.vDiffFull, 1e-14), 'LineWidth', 1.3, ...
        'DisplayName','|v diff|');
    semilogy(envNew.u, max(cmp.GDiffFull, 1e-14), 'LineWidth', 1.3, ...
        'DisplayName','||G diff||');
    legend('Location','best');

    exportgraphics(fig, fullfile(outDir, fileName), 'Resolution', 180);
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