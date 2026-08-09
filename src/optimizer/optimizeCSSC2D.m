function [Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params)
%OPTIMIZECSSC2D Unified global CSSC optimizer.
%
% Select gradient mode with:
%   params.solver.gradMode = 'semi-analytic';  % recommended
%   params.solver.gradMode = 'finite-diff';    % reference / debugging
%
% The optimizer uses Adam on all interior B-spline control points.

    params = setDefaultParams(params);

    Ptemplate = Pinit;
    Ptemplate(1,:) = Pref(1,:);
    Ptemplate(end,:) = Pref(end,:);

    x = packInterior(Ptemplate);
    M = zeros(size(x));
    V = zeros(size(x));

    info = struct();
    objectiveComponentFields = {'Jobs','Jclear','Jreg','Jref','Jsmooth','Jlen','Jtrust'};
    info.objectiveComponentFields = objectiveComponentFields;
    info.Jhist = nan(params.numIter,1);
    for k = 1:numel(objectiveComponentFields)
        info.([objectiveComponentFields{k} 'Hist']) = nan(params.numIter,1);
    end
    info.minClearHist = nan(params.numIter,1);
    info.gradNormHist = nan(params.numIter,1);
    info.stepNormHist = nan(params.numIter,1);
    info.relStepHist = nan(params.numIter,1);
    info.relImproveHist = nan(params.numIter,1);
    info.bestJHist = nan(params.numIter,1);
    info.bestSafeJHist = nan(params.numIter,1);
    info.noImproveCountHist = nan(params.numIter,1);
    info.snapshots = cell(params.numIter,1);
    info.gradMode = params.solver.gradMode;
    info.objectiveMode = params.solver.objectiveMode;
    info.returnPolicy = params.returnPolicy;
    info.timing = initTiming(params.numIter);
    info.stopParams = params.stop;
    info.converged = false;
    info.failed = false;
    info.stopIter = params.numIter;
    info.stopReason = 'maxIter';
    info.bestP = Ptemplate;
    info.bestJ = inf;
    info.bestIter = 0;
    info.bestDetails = [];
    info.bestSafeP = [];
    info.bestSafeJ = inf;
    info.bestSafeIter = 0;
    info.bestSafeDetails = [];
    info.hasSafeSolution = false;
    info.returnedSafe = false;
    info.returnedIter = 0;
    info.noImproveCount = 0;
    info.returnedBest = true;
    info.elapsedTimeSec = 0;

    fprintf('[optimizeCSSC2D] gradMode = %s | objectiveMode = %s\n', ...
        params.solver.gradMode, params.solver.objectiveMode);

    iterDone = 0;
    tOptimizeAll = tic;
    bestState = initBestState(Ptemplate);
    bestSafeState = initBestState([]);
    noImproveCount = 0;
    for iter = 1:params.numIter
        if toc(tOptimizeAll) >= params.stop.maxTimeSec
            info.stopIter = iterDone;
            info.stopReason = 'maxTimeSec';
            break;
        end
        tIter = tic;

        P = unpackInterior(x, Ptemplate);

        switch lower(params.solver.gradMode)
            case {'semi-analytic','semianalytic','semi'}
                tObj = tic;
                [J, details, gradP] = objectiveWithGradient( ...
                    P, Pref, obstacles, params);
                dtObj = toc(tObj);
                G = packInteriorGradient(gradP);
                gradStats = emptyGradStats();
                dtGrad = 0;
                objCalls = 1;
                objAll = dtObj;
                gradientComplete = true;

            case {'finite-diff','finitediff','fd'}
                objFunX = @(xx) objectiveFromVector(xx, Ptemplate, Pref, obstacles, params);
                tObj = tic;
                [J, details] = objFunX(x);
                dtObj = toc(tObj);

                tGrad = tic;
                remainingGradTime = max(0, params.stop.maxTimeSec - ...
                    toc(tOptimizeAll));
                [G, gradStats, gradientComplete] = ...
                    finiteDifferenceGradientTimed( ...
                    objFunX, x, params.fdStep, remainingGradTime);
                dtGrad = toc(tGrad);
                objCalls = 1 + gradStats.numObjCalls;
                objAll = dtObj + gradStats.objTotalTime;

            otherwise
                error('Unknown params.solver.gradMode: %s', params.solver.gradMode);
        end

        clearanceFailure = isClearanceEvaluationFailure(details);
        [bestState, significantBest] = updateBestState( ...
            bestState, P, J, details, iter, params.stop.tolBestRel);
        if isSafetySatisfied(details, params)
            bestSafeState = updateBestSafeState( ...
                bestSafeState, P, J, details, iter);
        end
        if significantBest
            noImproveCount = 0;
        else
            noImproveCount = noImproveCount + 1;
        end

        tUpdate = tic;
        if gradientComplete
            gradNormRaw = norm(G);
        else
            gradNormRaw = nan;
            G = zeros(size(x));
        end
        if gradNormRaw > params.gradClip
            G = G * params.gradClip / (gradNormRaw + 1e-12);
        end

        xPrev = x;
        if ~clearanceFailure && gradientComplete
            M = params.beta1 * M + (1 - params.beta1) * G;
            V = params.beta2 * V + (1 - params.beta2) * (G.^2);
            Mhat = M / (1 - params.beta1^iter);
            Vhat = V / (1 - params.beta2^iter);
            x = x - params.lr * Mhat ./ (sqrt(Vhat) + params.epsAdam);
        end
        stepNorm = norm(x - xPrev);
        relStep = stepNorm / max(1, norm(xPrev));
        dtUpdate = toc(tUpdate);

        dtIter = toc(tIter);
        iterDone = iter;

        info = storeObjectiveHistory(info, iter, J, details, objectiveComponentFields);
        info.minClearHist(iter) = details.minClear;
        info.gradNormHist(iter) = gradNormRaw;
        info.stepNormHist(iter) = stepNorm;
        info.relStepHist(iter) = relStep;
        info.relImproveHist(iter) = computeRelativeImprovement(info.Jhist, iter, params.stop.window);
        info.bestJHist(iter) = bestState.J;
        info.bestSafeJHist(iter) = bestSafeState.J;
        info.noImproveCountHist(iter) = noImproveCount;
        info.bestP = bestState.P;
        info.bestJ = bestState.J;
        info.bestIter = bestState.iter;
        info.bestDetails = bestState.details;
        info.bestSafeP = bestSafeState.P;
        info.bestSafeJ = bestSafeState.J;
        info.bestSafeIter = bestSafeState.iter;
        info.bestSafeDetails = bestSafeState.details;
        info.hasSafeSolution = isfinite(bestSafeState.J);
        info.noImproveCount = noImproveCount;

        if iter == 1 || mod(iter, params.saveInterval) == 0 || iter == params.numIter
            info.snapshots{iter} = unpackInterior(x, Ptemplate);
        end

        % Timing storage.
        mainTiming = getTiming(details);
        totalTiming = addTiming(mainTiming, gradStats.timingSum);

        info.timing.totalIter(iter) = dtIter;
        info.timing.mainObjective(iter) = dtObj;
        info.timing.gradient(iter) = dtGrad;
        info.timing.update(iter) = dtUpdate;
        info.timing.objCalls(iter) = objCalls;
        info.timing.objAll(iter) = objAll;
        info.timing.objAvg(iter) = objAll / max(1, objCalls);
        info.timing.evaluate(iter) = totalTiming.evaluate;
        info.timing.obstacleGrad(iter) = totalTiming.obstacleGrad;
        info.timing.regularization(iter) = totalTiming.regularization;
        info.timing.other(iter) = totalTiming.other;

        if iter == 1 || mod(iter, params.printInterval) == 0 || iter == params.numIter
            fprintf('iter %4d | J = %.6g | minClear = %.5f | gradNorm = %.3g\n', ...
                iter, J, details.minClear, gradNormRaw);
            if isfield(details, 'Jobs')
                fprintf('           Jobs=%.3g Jclear=%.3g Jreg=%.3g active=%d\n', ...
                    details.Jobs, details.Jclear, details.Jreg, numel(details.active.indices));
            end
        end

        if params.enableTimingDebug && ...
           (iter == 1 || mod(iter, params.timingPrintInterval) == 0 || iter == params.numIter)
            printTimingDebug(info, iter, params);
        end

        if ~gradientComplete
            info.stopIter = iter;
            info.stopReason = 'maxTimeSec';
            info.snapshots{iter} = unpackInterior(x, Ptemplate);
            break;
        end

        if toc(tOptimizeAll) >= params.stop.maxTimeSec
            info.stopIter = iter;
            info.stopReason = 'maxTimeSec';
            info.snapshots{iter} = unpackInterior(x, Ptemplate);
            break;
        end

        stopState = checkEarlyStop(info, iter, details, gradNormRaw, relStep, params);
        if stopState.shouldStop
            info.converged = ~stopState.isFailure;
            info.failed = stopState.isFailure;
            info.stopIter = iter;
            info.stopReason = stopState.reason;
            info.snapshots{iter} = unpackInterior(x, Ptemplate);
            if stopState.isFailure
                fprintf('optimization failed at iter %d | reason = %s | bestIter = %d | bestJ = %.6g\n', ...
                    iter, info.stopReason, bestState.iter, bestState.J);
            else
                fprintf('early stop at iter %d | reason = %s | relImprove = %.3g | gradNorm = %.3g | relStep = %.3g | noImprove = %d | bestIter = %d | bestJ = %.6g\n', ...
                    iter, info.stopReason, info.relImproveHist(iter), gradNormRaw, ...
                    relStep, noImproveCount, bestState.iter, bestState.J);
            end
            break;
        end
    end

    info.numIterActual = iterDone;
    info = trimInfoHistories(info, iterDone, objectiveComponentFields);

    finalIterateP = unpackInterior(x, Ptemplate);
    [finalIterateJ, finalIterateDetails] = objectiveValue( ...
        finalIterateP, Pref, obstacles, params);
    info.finalIterateP = finalIterateP;
    info.finalIterateJ = finalIterateJ;
    info.finalIterateDetails = finalIterateDetails;

    switch lower(char(params.returnPolicy))
        case {'best-safe','bestsafe'}
            if isfinite(bestSafeState.J)
                Popt = bestSafeState.P;
                info.returnedSafe = true;
                info.returnedIter = bestSafeState.iter;
            elseif isfinite(bestState.J)
                Popt = bestState.P;
                info.returnedIter = bestState.iter;
            else
                Popt = finalIterateP;
                info.returnedBest = false;
            end
        case {'best-objective','best'}
            if isfinite(bestState.J)
                Popt = bestState.P;
                info.returnedIter = bestState.iter;
            else
                Popt = finalIterateP;
                info.returnedBest = false;
            end
        case {'last-iterate','last'}
            Popt = finalIterateP;
            info.returnedBest = false;
            info.returnedIter = iterDone;
            info.returnedSafe = isSafetySatisfied( ...
                finalIterateDetails, params);
        otherwise
            error('Unknown params.returnPolicy: %s', params.returnPolicy);
    end
    info.bestP = bestState.P;
    info.bestJ = bestState.J;
    info.bestIter = bestState.iter;
    info.bestDetails = bestState.details;
    info.bestSafeP = bestSafeState.P;
    info.bestSafeJ = bestSafeState.J;
    info.bestSafeIter = bestSafeState.iter;
    info.bestSafeDetails = bestSafeState.details;
    info.hasSafeSolution = isfinite(bestSafeState.J);
    info.noImproveCount = noImproveCount;
    [info.finalJ, info.finalDetails] = objectiveValue( ...
        Popt, Pref, obstacles, params);
    info.finalMinClear = info.finalDetails.minClear;
    info.elapsedTimeSec = toc(tOptimizeAll);
end

%% Helpers

function [J, details] = objectiveFromVector(x, Ptemplate, Pref, obstacles, params)
    P = unpackInterior(x, Ptemplate);
    [J, details] = objectiveValue(P, Pref, obstacles, params);
end

function [J, details, gradP] = objectiveWithGradient( ...
        P, Pref, obstacles, params)
    switch normalizeObjectiveMode(params.solver.objectiveMode)
        case 'cssc-chord'
            [J, details, gradP] = objectiveCSSC2D_SemiGrad( ...
                P, Pref, obstacles, params);
        case 'centerline'
            [J, details, gradP] = objectiveCenterline2D_SemiGrad( ...
                P, Pref, obstacles, params);
    end
end

function [J, details] = objectiveValue(P, Pref, obstacles, params)
    switch normalizeObjectiveMode(params.solver.objectiveMode)
        case 'cssc-chord'
            [J, details] = objectiveCSSC2D_Value(P, Pref, obstacles, params);
        case 'centerline'
            [J, details] = objectiveCenterline2D_Value( ...
                P, Pref, obstacles, params);
    end
end

function mode = normalizeObjectiveMode(mode)
    mode = regexprep(lower(char(string(mode))), '[^a-z0-9]', '');
    switch mode
        case {'cssc','csscchord','chord','segment'}
            mode = 'cssc-chord';
        case {'centerline','point','pointsdf'}
            mode = 'centerline';
        otherwise
            error('Unknown objective mode: %s', char(string(mode)));
    end
end

function info = storeObjectiveHistory(info, iter, J, details, componentFields)
    info.Jhist(iter) = J;
    for k = 1:numel(componentFields)
        f = componentFields{k};
        if isfield(details, f) && isscalar(details.(f)) && isnumeric(details.(f))
            histName = [f 'Hist'];
            histValue = info.(histName);
            histValue(iter) = details.(f);
            info.(histName) = histValue;
        end
    end
end

function relImprove = computeRelativeImprovement(Jhist, iter, window)
    relImprove = nan;
    if iter <= 1
        return;
    end

    window = max(1, round(window));
    lookback = min(window, iter - 1);
    Jold = Jhist(iter - lookback);
    Jnew = Jhist(iter);
    if isfinite(Jold) && isfinite(Jnew)
        relImprove = abs(Jold - Jnew) / max(1, abs(Jold));
    end
end

function bestState = initBestState(Ptemplate)
    bestState = struct();
    bestState.P = Ptemplate;
    bestState.J = inf;
    bestState.iter = 0;
    bestState.details = [];
end

function [bestState, significantImprovement] = updateBestState( ...
        bestState, P, J, details, iter, tolBestRel)
    significantImprovement = false;
    if ~isfinite(J) || isClearanceEvaluationFailure(details)
        return;
    end

    if ~isfinite(bestState.J)
        shouldUpdate = true;
        significantImprovement = true;
    else
        deltaJ = bestState.J - J;
        shouldUpdate = deltaJ > 0;
        relImprovement = deltaJ / max(abs(bestState.J), eps);
        significantImprovement = shouldUpdate && ...
            relImprovement > tolBestRel;
    end

    if shouldUpdate
        bestState.P = P;
        bestState.J = J;
        bestState.iter = iter;
        bestState.details = details;
    end
end

function bestSafeState = updateBestSafeState( ...
        bestSafeState, P, J, details, iter)
    if ~isfinite(J) || isClearanceEvaluationFailure(details)
        return;
    end
    if ~isfinite(bestSafeState.J) || J < bestSafeState.J
        bestSafeState.P = P;
        bestSafeState.J = J;
        bestSafeState.iter = iter;
        bestSafeState.details = details;
    end
end

function stopState = checkEarlyStop(info, iter, details, gradNorm, relStep, params)
    stopState.shouldStop = false;
    stopState.reason = 'running';
    stopState.isFailure = false;

    if isClearanceEvaluationFailure(details)
        stopState.shouldStop = true;
        stopState.reason = 'invalid-clearance';
        stopState.isFailure = true;
        return;
    end

    if ~params.stop.enable || iter < params.stop.minIter
        return;
    end

    safeEnough = checkStopSafety(details, params);
    patienceReached = isscalar(params.stop.patience) && ...
        isfinite(params.stop.patience) && ...
        params.stop.patience > 0 && ...
        info.noImproveCount >= params.stop.patience;

    if patienceReached && safeEnough
        stopState.shouldStop = true;
        stopState.reason = 'best-patience';
        return;
    end

    relImprove = info.relImproveHist(iter);
    if ~isfinite(relImprove)
        return;
    end

    convergedJ = relImprove < params.stop.tolRelJ;
    convergedGrad = gradNorm < params.stop.tolGrad;
    convergedStep = relStep < params.stop.tolStep;

    if convergedJ && (convergedGrad || convergedStep) && safeEnough
        stopState.shouldStop = true;
        if convergedGrad && convergedStep
            stopState.reason = 'relJ+grad+step';
        elseif convergedGrad
            stopState.reason = 'relJ+grad';
        else
            stopState.reason = 'relJ+step';
        end
    end
end

function safeEnough = checkStopSafety(details, params)
    safeEnough = true;
    if ~params.stop.requireSafe
        return;
    end

    safeEnough = isSafetySatisfied(details, params);
end

function tf = isSafetySatisfied(details, params)
    tf = false;
    if isfield(params, 'dMin') && isstruct(details) && ...
            isfield(details, 'minClear') && isscalar(details.minClear) && ...
            isfinite(details.minClear)
        tf = details.minClear >= ...
            params.dMin + params.stop.clearanceMargin;
    end
end

function tf = isClearanceEvaluationFailure(details)
    tf = ~isstruct(details) || ...
        ~isfield(details, 'minClear') || ...
        ~isscalar(details.minClear) || ...
        isnan(details.minClear);
end

function info = trimInfoHistories(info, nIter, componentFields)
    info.Jhist = info.Jhist(1:nIter);
    for k = 1:numel(componentFields)
        f = [componentFields{k} 'Hist'];
        histValue = info.(f);
        info.(f) = histValue(1:nIter);
    end
    info.minClearHist = info.minClearHist(1:nIter);
    info.gradNormHist = info.gradNormHist(1:nIter);
    info.stepNormHist = info.stepNormHist(1:nIter);
    info.relStepHist = info.relStepHist(1:nIter);
    info.relImproveHist = info.relImproveHist(1:nIter);
    info.bestJHist = info.bestJHist(1:nIter);
    info.bestSafeJHist = info.bestSafeJHist(1:nIter);
    info.noImproveCountHist = info.noImproveCountHist(1:nIter);
    info.snapshots = info.snapshots(1:nIter);

    f = fieldnames(info.timing);
    for i = 1:numel(f)
        timingValue = info.timing.(f{i});
        info.timing.(f{i}) = timingValue(1:nIter);
    end
end

function x = packInterior(P)
    X = P(2:end-1,:);
    x = X(:);
end

function P = unpackInterior(x, Ptemplate)
    P = Ptemplate;
    X = reshape(x, [], 2);
    P(2:end-1,:) = X;
end

function g = packInteriorGradient(gradP)
    Gint = gradP(2:end-1,:);
    g = Gint(:);
end

function [G, stats, complete] = finiteDifferenceGradientTimed( ...
        objFun, x, h, maxTimeSec)
    G = zeros(size(x));
    stats = emptyGradStats();
    complete = true;
    tAll = tic;

    for k = 1:numel(x)
        if toc(tAll) >= maxTimeSec
            complete = false;
            return;
        end
        hk = h * max(1, abs(x(k)));
        xp = x;
        xm = x;
        xp(k) = xp(k) + hk;
        xm(k) = xm(k) - hk;

        t = tic;
        [Jp, detailsP] = objFun(xp);
        dt = toc(t);
        stats.numObjCalls = stats.numObjCalls + 1;
        stats.objTotalTime = stats.objTotalTime + dt;
        stats.timingSum = addTiming(stats.timingSum, getTiming(detailsP));

        if toc(tAll) >= maxTimeSec
            complete = false;
            return;
        end

        t = tic;
        [Jm, detailsM] = objFun(xm);
        dt = toc(t);
        stats.numObjCalls = stats.numObjCalls + 1;
        stats.objTotalTime = stats.objTotalTime + dt;
        stats.timingSum = addTiming(stats.timingSum, getTiming(detailsM));

        G(k) = (Jp - Jm) / (2 * hk);
    end
end

function stats = emptyGradStats()
    stats.numObjCalls = 0;
    stats.objTotalTime = 0;
    stats.timingSum = emptyTiming();
end

function timing = initTiming(nIter)
    timing.totalIter = zeros(nIter,1);
    timing.mainObjective = zeros(nIter,1);
    timing.gradient = zeros(nIter,1);
    timing.update = zeros(nIter,1);
    timing.objCalls = zeros(nIter,1);
    timing.objAll = zeros(nIter,1);
    timing.objAvg = zeros(nIter,1);
    timing.evaluate = zeros(nIter,1);
    timing.obstacleGrad = zeros(nIter,1);
    timing.regularization = zeros(nIter,1);
    timing.other = zeros(nIter,1);
end

function timing = emptyTiming()
    timing.evaluate = 0;
    timing.obstacleGrad = 0;
    timing.regularization = 0;
    timing.other = 0;
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
    idx = iter-win+1:iter;

    tTotal = mean(info.timing.totalIter(idx));
    tObj = mean(info.timing.mainObjective(idx));
    tGrad = mean(info.timing.gradient(idx));
    tUpdate = mean(info.timing.update(idx));
    objCalls = mean(info.timing.objCalls(idx));
    objAll = mean(info.timing.objAll(idx));
    objAvg = mean(info.timing.objAvg(idx));
    tEval = mean(info.timing.evaluate(idx));
    tObs = mean(info.timing.obstacleGrad(idx));
    tReg = mean(info.timing.regularization(idx));
    tOther = mean(info.timing.other(idx));

    safeTotal = max(tTotal, 1e-12);
    safeObjAll = max(objAll, 1e-12);

    fprintf('\n[TIMING] recent %d iters, up to iter %d\n', win, iter);
    fprintf('  total / iter              : %.3f ms\n', tTotal*1000);
    fprintf('  objective                 : %.3f ms  (%5.1f%% total)\n', tObj*1000, 100*tObj/safeTotal);
    fprintf('  gradient                  : %.3f ms  (%5.1f%% total)\n', tGrad*1000, 100*tGrad/safeTotal);
    fprintf('  Adam update               : %.3f ms  (%5.1f%% total)\n', tUpdate*1000, 100*tUpdate/safeTotal);
    fprintf('  objective calls / iter    : %.1f\n', objCalls);
    fprintf('  avg objective call        : %.3f ms\n', objAvg*1000);
    fprintf('  all objective time        : %.3f ms\n', objAll*1000);
    fprintf('    envelope/clearance eval : %.3f ms  (%5.1f%% obj)\n', tEval*1000, 100*tEval/safeObjAll);
    fprintf('    obstacle grad/cost      : %.3f ms  (%5.1f%% obj)\n', tObs*1000, 100*tObs/safeObjAll);
    fprintf('    regularization          : %.3f ms  (%5.1f%% obj)\n', tReg*1000, 100*tReg/safeObjAll);
    fprintf('    other                   : %.3f ms  (%5.1f%% obj)\n', tOther*1000, 100*tOther/safeObjAll);

    if strcmpi(info.gradMode, 'finite-diff') && tGrad > 0.7*tTotal
        fprintf('  [hint] finite-difference gradient dominates. Use semi-analytic mode.\n');
    elseif tEval > 0.7*objAll
        fprintf('  [hint] envelope/clearance evaluation dominates. Add warm-start for v(u).\n');
    end
end

function params = setDefaultParams(params)
    if ~isfield(params, 'solver'); params.solver = struct(); end
    if ~isfield(params.solver, 'gradMode'); params.solver.gradMode = 'semi-analytic'; end
    if ~isfield(params.solver, 'objectiveMode') || ...
            isempty(params.solver.objectiveMode)
        params.solver.objectiveMode = 'cssc-chord';
    end
    if ~isfield(params, 'returnPolicy') || isempty(params.returnPolicy)
        params.returnPolicy = 'best-safe';
    end

    if ~isfield(params, 'numIter'); params.numIter = 80; end
    if ~isfield(params, 'lr'); params.lr = 0.008; end
    if ~isfield(params, 'beta1'); params.beta1 = 0.9; end
    if ~isfield(params, 'beta2'); params.beta2 = 0.999; end
    if ~isfield(params, 'epsAdam'); params.epsAdam = 1e-8; end
    if ~isfield(params, 'fdStep'); params.fdStep = 1e-4; end
    if ~isfield(params, 'gradClip'); params.gradClip = 30; end
    if ~isfield(params, 'printInterval'); params.printInterval = 10; end
    if ~isfield(params, 'saveInterval'); params.saveInterval = 20; end

    if ~isfield(params, 'stop'); params.stop = struct(); end
    if ~isfield(params.stop, 'enable'); params.stop.enable = true; end
    if ~isfield(params.stop, 'minIter'); params.stop.minIter = 10; end
    if ~isfield(params.stop, 'window'); params.stop.window = 8; end
    if ~isfield(params.stop, 'tolRelJ'); params.stop.tolRelJ = 1e-2; end
    if ~isfield(params.stop, 'tolGrad'); params.stop.tolGrad = 1e-1; end
    if ~isfield(params.stop, 'tolStep'); params.stop.tolStep = 1e-3; end
    if ~isfield(params.stop, 'patience') || isempty(params.stop.patience) || ...
       ~isscalar(params.stop.patience) || ~isfinite(params.stop.patience)
        params.stop.patience = 10;
    end
    if ~isfield(params.stop, 'tolBestRel') || isempty(params.stop.tolBestRel) || ...
       ~isscalar(params.stop.tolBestRel) || ~isfinite(params.stop.tolBestRel)
        params.stop.tolBestRel = 1e-4;
    end
    if ~isfield(params.stop, 'requireSafe'); params.stop.requireSafe = true; end
    if ~isfield(params.stop, 'clearanceMargin'); params.stop.clearanceMargin = 0.0; end
    if ~isfield(params.stop, 'maxTimeSec') || isempty(params.stop.maxTimeSec)
        params.stop.maxTimeSec = inf;
    end
    if ~isfield(params, 'invalidClearancePenalty')
        params.invalidClearancePenalty = 1e6;
    end

    if ~isfield(params, 'enableTimingDebug'); params.enableTimingDebug = false; end
    if ~isfield(params, 'timingPrintInterval'); params.timingPrintInterval = 10; end
    if ~isfield(params, 'timingPrintWindow'); params.timingPrintWindow = 5; end

    if ~isfield(params, 'activeTopK'); params.activeTopK = 20; end
    if ~isfield(params, 'activeClearanceMargin'); params.activeClearanceMargin = 0.05; end
    if ~isfield(params, 'activeMode') || isempty(params.activeMode)
        params.activeMode = 'topk';
    end
end
