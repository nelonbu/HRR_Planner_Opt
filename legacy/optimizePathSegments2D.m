function [Popt, info] = optimizePathSegments2D(Pinit, Pref, sdfMap, params)
% optimizePathSegments2D
%
% Objective:
%   J = J_obs + J_clear + J_ref + J_spacing + J_smooth
%
% Timing debug:
%   objective+gradient
%   Adam/update/project
%   metrics/evaluation
%   snapshot saving
%   pair construction
%
% Inputs:
%   Pinit  : actual optimization initial path
%   Pref   : reference path
%   sdfMap : SDF query data
%   params : optimization config
%
% Outputs:
%   Popt : optimized path
%   info : history, snapshots, timing data

    params = setDefaultParams(params);

    P = Pinit;
    totalIter = params.numOuter * params.innerIter;

    %% ---------- History ----------
    info.Jhist = zeros(totalIter, 1);
    info.minClearHist = zeros(totalIter, 1);
    info.lenHist = zeros(totalIter, 1);
    info.maxKappaHist = zeros(totalIter, 1);
    info.maxAngleHist = zeros(totalIter, 1);
    info.minAdjDistHist = zeros(totalIter, 1);
    info.maxAdjDistHist = zeros(totalIter, 1);

    info.snapshots = {};
    info.snapshotPairs = {};
    info.snapshotIters = [];

    %% ---------- Timing debug storage ----------
    info.timing.totalIter = zeros(totalIter, 1);
    info.timing.objective = zeros(totalIter, 1);
    info.timing.update = zeros(totalIter, 1);
    info.timing.metrics = zeros(totalIter, 1);
    info.timing.snapshot = zeros(totalIter, 1);
    info.timing.pairBuildOuter = zeros(params.numOuter, 1);

    info.timing.numPairs = zeros(totalIter, 1);
    info.timing.objSdfQueries = zeros(totalIter, 1);
    info.timing.metricSdfQueries = zeros(totalIter, 1);

    globalIter = 0;

    %% ============================================================
    %  Outer-inner optimization
    % ============================================================
    for outer = 1:params.numOuter

        %% ---------- Pair construction timing ----------
        tPair = tic;
        pairs = constructPairsByLength(P, params.L_pair);
        pairBuildTime = toc(tPair);

        info.timing.pairBuildOuter(outer) = pairBuildTime;

        fprintf('\nOuter %d / %d | pair number = %d | pair build = %.3f ms\n', ...
            outer, params.numOuter, size(pairs, 1), pairBuildTime * 1000);

        M = zeros(size(P));
        V = zeros(size(P));

        for iter = 1:params.innerIter
            globalIter = globalIter + 1;

            tIter = tic;

            %% ---------- Objective and gradient timing ----------
            tObj = tic;
            [J, G] = objectiveAndGradient(P, Pref, pairs, sdfMap, params);
            dtObj = toc(tObj);

            %% ---------- Update timing ----------
            tUpdate = tic;

            % Fix endpoints
            G(1, :) = 0;
            G(end, :) = 0;

            % Gradient clipping
            gnorm = norm(G(:));
            if gnorm > params.gradClip
                G = G * params.gradClip / (gnorm + 1e-12);
            end

            % Adam update
            M = params.beta1 * M + (1 - params.beta1) * G;
            V = params.beta2 * V + (1 - params.beta2) * (G.^2);

            Mhat = M / (1 - params.beta1^iter);
            Vhat = V / (1 - params.beta2^iter);

            P = P - params.lr * Mhat ./ (sqrt(Vhat) + params.epsAdam);

            % Fix endpoints
            P(1, :) = Pref(1, :);
            P(end, :) = Pref(end, :);

            % Trust-region projection
            if isfield(params, 'rTrust') && params.rTrust > 0
                P = projectToTrustRegion(P, Pref, params.rTrust);
            end

            dtUpdate = toc(tUpdate);

            %% ---------- Metrics timing ----------
            tMetrics = tic;

            info.Jhist(globalIter) = J;

            needMetricEval = ...
                globalIter == 1 || ...
                mod(globalIter, params.metricEvalInterval) == 0 || ...
                globalIter == totalIter;
            
            if needMetricEval
                info.minClearHist(globalIter) = evaluateMinClearance( ...
                    P, pairs, sdfMap, params.robotRadius, params.metricDenseQ);
            
                info.lenHist(globalIter) = computePathLength(P);
                info.maxKappaHist(globalIter) = evaluateMaxTrueCurvature(P);
                info.maxAngleHist(globalIter) = evaluateMaxTurningAngle(P);
                info.minAdjDistHist(globalIter) = evaluateMinAdjacentDistance(P);
                info.maxAdjDistHist(globalIter) = evaluateMaxAdjacentDistance(P);
            else
                info.minClearHist(globalIter) = info.minClearHist(globalIter - 1);
                info.lenHist(globalIter) = info.lenHist(globalIter - 1);
                info.maxKappaHist(globalIter) = info.maxKappaHist(globalIter - 1);
                info.maxAngleHist(globalIter) = info.maxAngleHist(globalIter - 1);
                info.minAdjDistHist(globalIter) = info.minAdjDistHist(globalIter - 1);
                info.maxAdjDistHist(globalIter) = info.maxAdjDistHist(globalIter - 1);
            end

            dtMetrics = toc(tMetrics);

            %% ---------- Snapshot timing ----------
            tSnapshot = tic;

            if mod(globalIter, params.saveInterval) == 0 || ...
               globalIter == 1 || ...
               globalIter == totalIter

                info.snapshots{end+1} = P;
                info.snapshotPairs{end+1} = pairs;
                info.snapshotIters(end+1) = globalIter;
            end

            dtSnapshot = toc(tSnapshot);

            %% ---------- Total timing ----------
            dtIter = toc(tIter);

            info.timing.totalIter(globalIter) = dtIter;
            info.timing.objective(globalIter) = dtObj;
            info.timing.update(globalIter) = dtUpdate;
            info.timing.metrics(globalIter) = dtMetrics;
            info.timing.snapshot(globalIter) = dtSnapshot;

            info.timing.numPairs(globalIter) = size(pairs, 1);
            info.timing.objSdfQueries(globalIter) = ...
                size(pairs, 1) * (params.samplePerSeg + 1);
            info.timing.metricSdfQueries(globalIter) = ...
                size(pairs, 1) * (params.metricDenseQ + 1);

            %% ---------- Normal progress output ----------
            if mod(iter, 100) == 0
                fprintf(['  iter %4d | J = %.3f | clear = %.2f mm | ', ...
                         'len = %.2f mm | kappa = %.5f | angle = %.2f deg | ', ...
                         'adj = %.2f~%.2f mm\n'], ...
                    iter, J, ...
                    info.minClearHist(globalIter), ...
                    info.lenHist(globalIter), ...
                    info.maxKappaHist(globalIter), ...
                    rad2deg(info.maxAngleHist(globalIter)), ...
                    info.minAdjDistHist(globalIter), ...
                    info.maxAdjDistHist(globalIter));
            end

            %% ---------- Timing debug output ----------
            if params.enableTimingDebug && ...
               mod(globalIter, params.timingPrintInterval) == 0

                printTimingDebug(info, globalIter, params, pairBuildTime);
            end
        end
    end

    %% ============================================================
    %  Final evaluation
    % ============================================================
    Popt = P;
    info.pairsFinal = constructPairsByLength(Popt, params.L_pair);

    info.finalMinClear = evaluateMinClearance(Popt, info.pairsFinal, sdfMap, params.robotRadius, 100);
    info.finalLength = computePathLength(Popt);
    info.finalMaxKappa = evaluateMaxTrueCurvature(Popt);
    info.finalMaxAngle = evaluateMaxTurningAngle(Popt);
    info.finalMinAdjDist = evaluateMinAdjacentDistance(Popt);
    info.finalMaxAdjDist = evaluateMaxAdjacentDistance(Popt);

    fprintf('\n================ Final result ================\n');
    fprintf('Final minimum clearance: %.3f mm\n', info.finalMinClear);
    fprintf('Required minimum clearance: %.3f mm\n', params.d_min);

    fprintf('Final path length: %.3f mm\n', info.finalLength);

    fprintf('Final max true curvature: %.6f 1/mm\n', info.finalMaxKappa);
    fprintf('Final max turning angle: %.3f deg\n', rad2deg(info.finalMaxAngle));

    fprintf('Final min adjacent distance: %.3f mm\n', info.finalMinAdjDist);
    fprintf('Final max adjacent distance: %.3f mm\n', info.finalMaxAdjDist);

    if info.finalMinClear >= params.d_min
        fprintf('Collision check: PASS under sampled checking.\n');
    else
        fprintf('Collision check: WARNING, clearance violation exists.\n');
    end

    if params.enableTimingDebug
        printTimingSummary(info, params);
    end
end


%% ============================================================
%  Objective and gradients
% ============================================================

function [J, G] = objectiveAndGradient(P, Pref, pairs, sdfMap, params)
    N = size(P, 1);
    G = zeros(size(P));
    J = 0;

    %% ---------- 1. Obstacle and soft-clearance costs ----------
    Q = params.samplePerSeg;
    ts = linspace(0, 1, Q + 1);

    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        pi = P(i, :);
        pj = P(j, :);

        for q = 1:numel(ts)
            t = ts(q);
            x = (1 - t) * pi + t * pj;

            [phi, gradPhi] = querySDF2D(sdfMap, x);

            clearance = phi - params.robotRadius;

            dJdx = [0, 0];

            % Hard safety penalty
            gapObs = params.d_min - clearance;
            [spObs, dspObs] = softplusSmooth(gapObs, params.epsObs);

            J = J + params.wObs * spObs^2;

            % d gap / d x = - gradPhi
            dJdx = dJdx - 2 * params.wObs * spObs * dspObs * gradPhi;

            % Soft clearance buffer
            gapClear = params.d_pref - clearance;
            [spClear, dspClear] = softplusSmooth(gapClear, params.epsClear);

            J = J + params.wClear * spClear^2;

            dJdx = dJdx - 2 * params.wClear * spClear * dspClear * gradPhi;

            % Chain rule: x = (1 - t) * p_i + t * p_j
            G(i, :) = G(i, :) + (1 - t) * dJdx;
            G(j, :) = G(j, :) + t * dJdx;
        end
    end

    %% ---------- 2. Reference path cost ----------
    D = P - Pref;
    D(1, :) = 0;
    D(end, :) = 0;

    J = J + params.wRef * sum(D(:).^2);
    G = G + 2 * params.wRef * D;

    %% ---------- 3. Adjacent spacing quality cost ----------
    for i = 1:N-1
        dVec = P(i+1, :) - P(i, :);

        % Smooth distance to avoid singular gradient when dVec -> 0
        d = sqrt(sum(dVec.^2) + params.epsDist^2);

        d0Vec = Pref(i+1, :) - Pref(i, :);
        d0 = norm(d0Vec);

        if d0 < 1e-8
            continue;
        end

        r = d / d0;

        % Near-1 term
        errNear = r - 1.0;
        J = J + params.wSpacingNear * errNear^2;

        dJdr = 2 * params.wSpacingNear * errNear;

        % Upper-band term
        gapUpper = r - params.spacingUpperRatio;
        [spUpper, dspUpper] = softplusSmooth(gapUpper, params.epsSpacing);

        J = J + params.wSpacingBand * spUpper^2;
        dJdr = dJdr + 2 * params.wSpacingBand * spUpper * dspUpper;

        % Lower-band term
        gapLower = params.spacingLowerRatio - r;
        [spLower, dspLower] = softplusSmooth(gapLower, params.epsSpacing);

        J = J + params.wSpacingBand * spLower^2;

        % d gapLower / d r = -1
        dJdr = dJdr - 2 * params.wSpacingBand * spLower * dspLower;

        % Chain rule
        dJddVec = (dJdr / d0) * (dVec / d);

        G(i, :)   = G(i, :)   - dJddVec;
        G(i+1, :) = G(i+1, :) + dJddVec;
    end

    %% ---------- 4. Smoothness cost ----------
    for i = 2:N-1
        e = P(i+1, :) - 2 * P(i, :) + P(i-1, :);

        J = J + params.wSmooth * sum(e.^2);

        G(i-1, :) = G(i-1, :) + 2 * params.wSmooth * e;
        G(i, :)   = G(i, :)   - 4 * params.wSmooth * e;
        G(i+1, :) = G(i+1, :) + 2 * params.wSmooth * e;
    end
end


%% ============================================================
%  Pair construction
% ============================================================

function pairs = constructPairsByLength(P, L_pair)
    N = size(P, 1);
    pairs = zeros(N-1, 2);
    cnt = 0;

    for i = 1:N-1
        jBest = N;
        found = false;

        for j = i+1:N
            dCurr = norm(P(j, :) - P(i, :));

            if dCurr >= L_pair
                found = true;

                if j == i + 1
                    jBest = j;
                else
                    dPrev = norm(P(j-1, :) - P(i, :));

                    if abs(dPrev - L_pair) <= abs(dCurr - L_pair)
                        jBest = j - 1;
                    else
                        jBest = j;
                    end
                end

                break;
            end
        end

        if ~found
            jBest = N;
        end

        if jBest > i
            cnt = cnt + 1;
            pairs(cnt, :) = [i, jBest];
        end
    end

    pairs = pairs(1:cnt, :);
end


%% ============================================================
%  SDF query
% ============================================================

function [phi, gradPhi] = querySDF2D(sdfMap, x)
    switch sdfMap.type
        case 'circle_analytic'
            [phi, gradPhi] = circleUnionSDF2D(x, sdfMap.obstacles);
        otherwise
            error('Unknown sdfMap.type: %s', sdfMap.type);
    end
end


function [phi, gradPhi] = circleUnionSDF2D(x, obstacles)
    centers = obstacles(:, 1:2);
    radii = obstacles(:, 3);

    diff = x - centers;
    distToCenter = sqrt(sum(diff.^2, 2));

    phiAll = distToCenter - radii;
    [phi, idx] = min(phiAll);

    if distToCenter(idx) < 1e-10
        gradPhi = [1, 0];
    else
        gradPhi = diff(idx, :) / distToCenter(idx);
    end
end


%% ============================================================
%  Smooth penalty utilities
% ============================================================

function [y, dy] = softplusSmooth(x, epsVal)
% y  = eps * log(1 + exp(x / eps))
% dy = sigmoid(x / eps)

    z = x / epsVal;

    if z > 50
        y = x;
        dy = 1;
    elseif z < -50
        ez = exp(z);
        y = epsVal * ez;
        dy = ez;
    else
        y = epsVal * log1p(exp(z));
        dy = 1 / (1 + exp(-z));
    end
end


%% ============================================================
%  Evaluation utilities
% ============================================================

function minClear = evaluateMinClearance(P, pairs, sdfMap, robotRadius, denseQ)
    minClear = inf;
    ts = linspace(0, 1, denseQ + 1);

    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        for q = 1:numel(ts)
            t = ts(q);
            x = (1 - t) * P(i, :) + t * P(j, :);

            [phi, ~] = querySDF2D(sdfMap, x);
            clearance = phi - robotRadius;

            minClear = min(minClear, clearance);
        end
    end
end


function maxKappa = evaluateMaxTrueCurvature(P)
    maxKappa = 0;

    for i = 2:size(P, 1)-1
        pPrev = P(i-1, :);
        pMid  = P(i, :);
        pNext = P(i+1, :);

        a = pMid - pPrev;
        b = pNext - pMid;

        la = norm(a);
        lb = norm(b);
        chord = norm(pNext - pPrev);

        if la < 1e-8 || lb < 1e-8 || chord < 1e-8
            continue;
        end

        crossVal = abs(a(1) * b(2) - a(2) * b(1));
        kappa = 2 * crossVal / (la * lb * chord);

        maxKappa = max(maxKappa, kappa);
    end
end


function maxAngle = evaluateMaxTurningAngle(P)
    maxAngle = 0;

    for i = 2:size(P, 1)-1
        a = P(i, :) - P(i-1, :);
        b = P(i+1, :) - P(i, :);

        la = norm(a);
        lb = norm(b);

        if la < 1e-8 || lb < 1e-8
            continue;
        end

        c = dot(a, b) / (la * lb);
        c = max(-1, min(1, c));

        theta = acos(c);
        maxAngle = max(maxAngle, theta);
    end
end


function minAdjDist = evaluateMinAdjacentDistance(P)
    d = diff(P, 1, 1);
    dist = sqrt(sum(d.^2, 2));
    minAdjDist = min(dist);
end


function maxAdjDist = evaluateMaxAdjacentDistance(P)
    d = diff(P, 1, 1);
    dist = sqrt(sum(d.^2, 2));
    maxAdjDist = max(dist);
end


function len = computePathLength(P)
    d = diff(P, 1, 1);
    len = sum(sqrt(sum(d.^2, 2)));
end


function P = projectToTrustRegion(P, Pref, rTrust)
    for i = 2:size(P, 1)-1
        d = P(i, :) - Pref(i, :);
        nd = norm(d);

        if nd > rTrust
            P(i, :) = Pref(i, :) + d / nd * rTrust;
        end
    end
end


%% ============================================================
%  Timing debug
% ============================================================

function printTimingDebug(info, globalIter, params, pairBuildTime)
    win = params.timingPrintInterval;
    idx0 = max(1, globalIter - win + 1);
    idx = idx0:globalIter;

    tTotal = mean(info.timing.totalIter(idx));
    tObj = mean(info.timing.objective(idx));
    tUpdate = mean(info.timing.update(idx));
    tMetrics = mean(info.timing.metrics(idx));
    tSnapshot = mean(info.timing.snapshot(idx));

    safeTotal = max(tTotal, 1e-12);

    pctObj = 100 * tObj / safeTotal;
    pctUpdate = 100 * tUpdate / safeTotal;
    pctMetrics = 100 * tMetrics / safeTotal;
    pctSnapshot = 100 * tSnapshot / safeTotal;

    objQueries = round(mean(info.timing.objSdfQueries(idx)));
    metricQueries = round(mean(info.timing.metricSdfQueries(idx)));
    numPairs = round(mean(info.timing.numPairs(idx)));

    fprintf('\n[TIMING] recent %d iters, up to global iter %d\n', ...
        numel(idx), globalIter);

    fprintf('  total per iter       : %.3f ms\n', tTotal * 1000);
    fprintf('  objective+gradient   : %.3f ms  (%5.1f%%)\n', ...
        tObj * 1000, pctObj);
    fprintf('  Adam/update/project  : %.3f ms  (%5.1f%%)\n', ...
        tUpdate * 1000, pctUpdate);
    fprintf('  metrics/evaluation   : %.3f ms  (%5.1f%%)\n', ...
        tMetrics * 1000, pctMetrics);
    fprintf('  snapshot save        : %.3f ms  (%5.1f%%)\n', ...
        tSnapshot * 1000, pctSnapshot);
    fprintf('  pair build outer     : %.3f ms  (current outer)\n', ...
        pairBuildTime * 1000);

    fprintf('  pairs                : %d\n', numPairs);
    fprintf('  SDF queries objective: ~%d / iter\n', objQueries);
    fprintf('  SDF queries metrics  : ~%d / iter\n', metricQueries);

    if tMetrics > tObj
        fprintf('  [hint] metrics are more expensive than objective. Reduce metricDenseQ or evaluate metrics less often.\n');
    elseif tObj > 0.7 * tTotal
        fprintf('  [hint] objective dominates. Reduce samplePerSeg or vectorize obstacle/SDF evaluation.\n');
    end
end


function printTimingSummary(info, params)
    valid = info.timing.totalIter > 0;

    if ~any(valid)
        return;
    end

    tTotal = mean(info.timing.totalIter(valid));
    tObj = mean(info.timing.objective(valid));
    tUpdate = mean(info.timing.update(valid));
    tMetrics = mean(info.timing.metrics(valid));
    tSnapshot = mean(info.timing.snapshot(valid));

    fprintf('\n================ Timing summary ================\n');
    fprintf('Average total per iter      : %.3f ms\n', tTotal * 1000);
    fprintf('Average objective+gradient  : %.3f ms\n', tObj * 1000);
    fprintf('Average Adam/update/project : %.3f ms\n', tUpdate * 1000);
    fprintf('Average metrics/evaluation  : %.3f ms\n', tMetrics * 1000);
    fprintf('Average snapshot save       : %.3f ms\n', tSnapshot * 1000);

    fprintf('Average objective SDF queries: %.0f / iter\n', ...
        mean(info.timing.objSdfQueries(valid)));
    fprintf('Average metric SDF queries   : %.0f / iter\n', ...
        mean(info.timing.metricSdfQueries(valid)));

    fprintf('Current metricDenseQ         : %d\n', params.metricDenseQ);
    fprintf('Current samplePerSeg         : %d\n', params.samplePerSeg);
end


%% ============================================================
%  Default parameters
% ============================================================

function params = setDefaultParams(params)
    if ~isfield(params, 'wSpacingNear')
        if isfield(params, 'wLen')
            params.wSpacingNear = params.wLen;
        else
            params.wSpacingNear = 1.0;
        end
    end

    if ~isfield(params, 'wSpacingBand')
        params.wSpacingBand = 50.0;
    end

    if ~isfield(params, 'spacingLowerRatio')
        params.spacingLowerRatio = 0.5;
    end

    if ~isfield(params, 'spacingUpperRatio')
        params.spacingUpperRatio = 1.5;
    end

    if ~isfield(params, 'epsObs')
        params.epsObs = 3.0;
    end

    if ~isfield(params, 'epsClear')
        params.epsClear = 6.0;
    end

    if ~isfield(params, 'epsSpacing')
        params.epsSpacing = 0.05;
    end

    if ~isfield(params, 'epsDist')
        params.epsDist = 1e-3;
    end

    if ~isfield(params, 'saveInterval')
        params.saveInterval = 40;
    end

    if ~isfield(params, 'gradClip')
        params.gradClip = 500.0;
    end

    if ~isfield(params, 'beta1')
        params.beta1 = 0.9;
    end

    if ~isfield(params, 'beta2')
        params.beta2 = 0.999;
    end

    if ~isfield(params, 'epsAdam')
        params.epsAdam = 1e-8;
    end

    if ~isfield(params, 'enableTimingDebug')
        params.enableTimingDebug = false;
    end

    if ~isfield(params, 'timingPrintInterval')
        params.timingPrintInterval = 100;
    end

    if ~isfield(params, 'metricDenseQ')
        params.metricDenseQ = 50;
    end

    if ~isfield(params, 'metricEvalInterval')
        params.metricEvalInterval = 20;
    end
    
    if ~isfield(params, 'metricDenseQ')
        params.metricDenseQ = 30;
end
end