function [Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params)
%OPTIMIZECSSC2D Simple Adam optimizer with finite-difference gradients.
%
% Added timing debug:
%   - total iteration time
%   - main objective time
%   - finite-difference gradient time
%   - Adam update time
%   - objective call count
%   - objective internal timing: envelope / obstacle / regularization / curvature

    params = setDefaultParamsLocal(params);

    P = Pinit;
    P(1,:) = Pref(1,:);
    P(end,:) = Pref(end,:);

    x = packInterior(P);
    M = zeros(size(x));
    V = zeros(size(x));

    info.Jhist = zeros(params.numIter, 1);
    info.minClearHist = zeros(params.numIter, 1);
    info.snapshots = cell(0,1);
    info.snapshotIters = [];

    % ---------- Timing storage ----------
    info.timing.totalIter = zeros(params.numIter, 1);
    info.timing.mainObjective = zeros(params.numIter, 1);
    info.timing.gradient = zeros(params.numIter, 1);
    info.timing.update = zeros(params.numIter, 1);

    info.timing.objCalls = zeros(params.numIter, 1);
    info.timing.objAll = zeros(params.numIter, 1);
    info.timing.objAvg = zeros(params.numIter, 1);

    info.timing.envelope = zeros(params.numIter, 1);
    info.timing.obstacle = zeros(params.numIter, 1);
    info.timing.regularization = zeros(params.numIter, 1);
    info.timing.curvature = zeros(params.numIter, 1);
    info.timing.objectiveOther = zeros(params.numIter, 1);

    objFun = @(xx) objectiveFromVector(xx, P, Pref, obstacles, params);

    for iter = 1:params.numIter
        tIter = tic;

        % ---------- Main objective ----------
        tMainObj = tic;
        [J, details] = objFun(x);
        dtMainObj = toc(tMainObj);

        % ---------- Finite-difference gradient ----------
        tGrad = tic;
        [G, gradStats] = finiteDifferenceGradientTimed(objFun, x, params.fdStep);
        dtGrad = toc(tGrad);

        % ---------- Adam update ----------
        tUpdate = tic;

        gnorm = norm(G);
        if gnorm > params.gradClip
            G = G * params.gradClip / (gnorm + 1e-12);
        end

        M = params.beta1 * M + (1 - params.beta1) * G;
        V = params.beta2 * V + (1 - params.beta2) * (G.^2);

        Mhat = M / (1 - params.beta1^iter);
        Vhat = V / (1 - params.beta2^iter);

        x = x - params.lr * Mhat ./ (sqrt(Vhat) + params.epsAdam);

        dtUpdate = toc(tUpdate);

        dtIter = toc(tIter);

        % ---------- History ----------
        info.Jhist(iter) = J;
        info.minClearHist(iter) = details.minClear;

        if mod(iter, params.saveInterval) == 0 || iter == 1 || iter == params.numIter
            info.snapshots{end+1} = unpackInterior(x, P);
            info.snapshotIters(end+1) = iter;
        end

        % ---------- Timing aggregation ----------
        mainTiming = getTiming(details);
        totalTiming = addTiming(mainTiming, gradStats.timingSum);

        totalObjCalls = 1 + gradStats.numObjCalls;
        totalObjTime = dtMainObj + gradStats.objTotalTime;

        info.timing.totalIter(iter) = dtIter;
        info.timing.mainObjective(iter) = dtMainObj;
        info.timing.gradient(iter) = dtGrad;
        info.timing.update(iter) = dtUpdate;

        info.timing.objCalls(iter) = totalObjCalls;
        info.timing.objAll(iter) = totalObjTime;
        info.timing.objAvg(iter) = totalObjTime / max(1, totalObjCalls);

        info.timing.envelope(iter) = totalTiming.envelope;
        info.timing.obstacle(iter) = totalTiming.obstacle;
        info.timing.regularization(iter) = totalTiming.regularization;
        info.timing.curvature(iter) = totalTiming.curvature;

        knownObjTime = totalTiming.envelope + totalTiming.obstacle + ...
                       totalTiming.regularization + totalTiming.curvature;
        info.timing.objectiveOther(iter) = max(0, totalObjTime - knownObjTime);

        % ---------- Normal progress print ----------
        if mod(iter, params.printInterval) == 0 || iter == 1 || iter == params.numIter
            fprintf('iter %4d | J = %.6g | minClear = %.4f\n', ...
                iter, J, details.minClear);
        end

        % ---------- Timing debug print ----------
        if params.enableTimingDebug && ...
           (mod(iter, params.timingPrintInterval) == 0 || iter == 1 || iter == params.numIter)
            printTimingDebug(info, iter, params);
        end
    end

    Popt = unpackInterior(x, P);
    [info.finalJ, info.finalDetails] = objectiveCSSC2D(Popt, Pref, obstacles, params);
    info.finalMinClear = info.finalDetails.minClear;
end

%% ============================================================
% Local helpers
%% ============================================================

function [J, details] = objectiveFromVector(x, Ptemplate, Pref, obstacles, params)
    P = unpackInterior(x, Ptemplate);
    [J, details] = objectiveCSSC2D(P, Pref, obstacles, params);
end

function x = packInterior(P)
    X = P(2:end-1, :);
    x = X(:);
end

function P = unpackInterior(x, Ptemplate)
    P = Ptemplate;
    X = reshape(x, [], 2);
    P(2:end-1, :) = X;
end

function [G, stats] = finiteDifferenceGradientTimed(objFun, x, h)
    G = zeros(size(x));

    stats.numObjCalls = 0;
    stats.objTotalTime = 0;
    stats.timingSum = emptyTiming();

    for k = 1:numel(x)
        hk = h * max(1, abs(x(k)));

        xp = x;
        xm = x;
        xp(k) = xp(k) + hk;
        xm(k) = xm(k) - hk;

        tObj = tic;
        [Jp, detailsP] = objFun(xp);
        dt = toc(tObj);
        stats.numObjCalls = stats.numObjCalls + 1;
        stats.objTotalTime = stats.objTotalTime + dt;
        stats.timingSum = addTiming(stats.timingSum, getTiming(detailsP));

        tObj = tic;
        [Jm, detailsM] = objFun(xm);
        dt = toc(tObj);
        stats.numObjCalls = stats.numObjCalls + 1;
        stats.objTotalTime = stats.objTotalTime + dt;
        stats.timingSum = addTiming(stats.timingSum, getTiming(detailsM));

        G(k) = (Jp - Jm) / (2 * hk);
    end
end

function timing = emptyTiming()
    timing = struct();
    timing.envelope = 0;
    timing.obstacle = 0;
    timing.regularization = 0;
    timing.curvature = 0;
end

function timing = getTiming(details)
    timing = emptyTiming();

    if isfield(details, 'timing')
        f = fieldnames(timing);
        for i = 1:numel(f)
            if isfield(details.timing, f{i})
                timing.(f{i}) = details.timing.(f{i});
            end
        end
    end
end

function out = addTiming(a, b)
    out = emptyTiming();
    f = fieldnames(out);
    for i = 1:numel(f)
        out.(f{i}) = a.(f{i}) + b.(f{i});
    end
end

function printTimingDebug(info, iter, params)
    win = min(params.timingPrintWindow, iter);
    idx = iter - win + 1 : iter;

    tTotal = mean(info.timing.totalIter(idx));
    tMainObj = mean(info.timing.mainObjective(idx));
    tGrad = mean(info.timing.gradient(idx));
    tUpdate = mean(info.timing.update(idx));

    objCalls = mean(info.timing.objCalls(idx));
    objAll = mean(info.timing.objAll(idx));
    objAvg = mean(info.timing.objAvg(idx));

    tEnvelope = mean(info.timing.envelope(idx));
    tObstacle = mean(info.timing.obstacle(idx));
    tReg = mean(info.timing.regularization(idx));
    tCurv = mean(info.timing.curvature(idx));
    tOther = mean(info.timing.objectiveOther(idx));

    safeTotal = max(tTotal, 1e-12);
    safeObjAll = max(objAll, 1e-12);

    fprintf('\n[TIMING] recent %d iters, up to iter %d\n', win, iter);
    fprintf('  total / iter              : %.3f ms\n', tTotal * 1000);
    fprintf('  main objective            : %.3f ms  (%5.1f%% total)\n', ...
        tMainObj * 1000, 100 * tMainObj / safeTotal);
    fprintf('  finite-diff gradient      : %.3f ms  (%5.1f%% total)\n', ...
        tGrad * 1000, 100 * tGrad / safeTotal);
    fprintf('  Adam update               : %.3f ms  (%5.1f%% total)\n', ...
        tUpdate * 1000, 100 * tUpdate / safeTotal);

    fprintf('  objective calls / iter    : %.1f\n', objCalls);
    fprintf('  avg objective call        : %.3f ms\n', objAvg * 1000);
    fprintf('  all objective time        : %.3f ms\n', objAll * 1000);

    fprintf('    fixedChordEnvelope      : %.3f ms  (%5.1f%% obj)\n', ...
        tEnvelope * 1000, 100 * tEnvelope / safeObjAll);
    fprintf('    obstacle clearance      : %.3f ms  (%5.1f%% obj)\n', ...
        tObstacle * 1000, 100 * tObstacle / safeObjAll);
    fprintf('    ref/smooth/length       : %.3f ms  (%5.1f%% obj)\n', ...
        tReg * 1000, 100 * tReg / safeObjAll);
    fprintf('    curvature               : %.3f ms  (%5.1f%% obj)\n', ...
        tCurv * 1000, 100 * tCurv / safeObjAll);
    fprintf('    other objective time    : %.3f ms  (%5.1f%% obj)\n', ...
        tOther * 1000, 100 * tOther / safeObjAll);

    if tGrad > 0.7 * tTotal
        fprintf('  [hint] finite-difference gradient dominates. Consider analytic/semi-analytic gradient.\n');
    elseif tEnvelope > 0.5 * objAll
        fprintf('  [hint] fixedChordEnvelope dominates. Reduce envOpts.nU/nVGrid or add warm start.\n');
    elseif tObstacle > 0.5 * objAll
        fprintf('  [hint] obstacle clearance dominates. Use active obstacles or faster min-u search.\n');
    end
end

function params = setDefaultParamsLocal(params)
    if ~isfield(params, 'numIter'); params.numIter = 120; end
    if ~isfield(params, 'lr'); params.lr = 0.03; end
    if ~isfield(params, 'beta1'); params.beta1 = 0.9; end
    if ~isfield(params, 'beta2'); params.beta2 = 0.999; end
    if ~isfield(params, 'epsAdam'); params.epsAdam = 1e-8; end
    if ~isfield(params, 'fdStep'); params.fdStep = 1e-4; end
    if ~isfield(params, 'gradClip'); params.gradClip = 100.0; end
    if ~isfield(params, 'saveInterval'); params.saveInterval = 20; end
    if ~isfield(params, 'printInterval'); params.printInterval = 10; end

    if ~isfield(params, 'enableTimingDebug'); params.enableTimingDebug = true; end
    if ~isfield(params, 'timingPrintInterval'); params.timingPrintInterval = 10; end
    if ~isfield(params, 'timingPrintWindow'); params.timingPrintWindow = 5; end
end