function [Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params)
%OPTIMIZECSSC2D Adam optimizer with finite-difference gradients and stage saving.
%
% Added result features:
%   - automatically creates results/<runName>/;
%   - saves progress.csv;
%   - saves snapshots/stage_iter_XXXXXX.mat every params.resultSaveInterval;
%   - saves final_result.mat and summary.txt after optimization;
%   - writes console messages to logs/optimization_log.txt.
%
% Required external function:
%   objectiveCSSC2D(P, Pref, obstacles, params)

    if nargin < 4
        params = struct();
    end
    params = setDefaultParamsLocal(params);

    [resultDir, snapshotDir, logDir] = prepareResultDirs(params);
    logFile = fullfile(logDir, 'optimization_log.txt');
    logFID = fopen(logFile, 'w');
    cleaner = onCleanup(@() closeLogFile(logFID)); %#ok<NASGU>

    progressCsv = fullfile(resultDir, 'progress.csv');
    writeProgressHeader(progressCsv);

    P = Pinit;
    P(1,:) = Pref(1,:);
    P(end,:) = Pref(end,:);

    x = packInterior(P);
    M = zeros(size(x));
    V = zeros(size(x));

    info = struct();
    info.resultDir = resultDir;
    info.snapshotDir = snapshotDir;
    info.logDir = logDir;
    info.logFile = logFile;
    info.progressCsv = progressCsv;

    info.Jhist = zeros(params.numIter, 1);
    info.minClearHist = zeros(params.numIter, 1);
    info.gradNormHist = zeros(params.numIter, 1);
    info.snapshots = cell(0,1);
    info.snapshotIters = [];

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

    logMsg(logFID, 'CSSC optimization started: %s\n', datestr(now, 31));
    logMsg(logFID, 'Result directory: %s\n', resultDir);

    if params.enableResultSave
        save(fullfile(resultDir, 'initial_problem.mat'), ...
            'Pinit', 'Pref', 'obstacles', 'params', params.saveMatFlag);
    end

    for iter = 1:params.numIter
        tIter = tic;

        tMainObj = tic;
        [J, details] = objFun(x);
        dtMainObj = toc(tMainObj);

        tGrad = tic;
        [G, gradStats] = finiteDifferenceGradientTimed(objFun, x, params.fdStep);
        dtGrad = toc(tGrad);

        tUpdate = tic;

        gnormRaw = norm(G);
        if gnormRaw > params.gradClip
            G = G * params.gradClip / (gnormRaw + 1e-12);
        end

        M = params.beta1 * M + (1 - params.beta1) * G;
        V = params.beta2 * V + (1 - params.beta2) * (G.^2);

        Mhat = M / (1 - params.beta1^iter);
        Vhat = V / (1 - params.beta2^iter);

        x = x - params.lr * Mhat ./ (sqrt(Vhat) + params.epsAdam);
        dtUpdate = toc(tUpdate);
        dtIter = toc(tIter);

        Pcur = unpackInterior(x, P);

        info.Jhist(iter) = J;
        info.minClearHist(iter) = safeGet(details, 'minClear', NaN);
        info.gradNormHist(iter) = gnormRaw;

        if shouldSaveSnapshot(iter, params)
            info.snapshots{end+1} = Pcur;
            info.snapshotIters(end+1) = iter;
        end

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

        appendProgressLine(progressCsv, iter, J, info.minClearHist(iter), ...
            gnormRaw, dtIter, dtMainObj, dtGrad, dtUpdate, totalObjCalls, totalObjTime);

        if mod(iter, params.printInterval) == 0 || iter == 1 || iter == params.numIter
            logMsg(logFID, 'iter %4d | J = %.6g | minClear = %.4f | gradNorm = %.3g\n', ...
                iter, J, info.minClearHist(iter), gnormRaw);
        end

        if params.enableTimingDebug && ...
           (mod(iter, params.timingPrintInterval) == 0 || iter == 1 || iter == params.numIter)
            printTimingDebug(logFID, info, iter, params);
        end

        if params.enableResultSave && shouldSaveResult(iter, params)
            stageInfo = trimInfoToIter(info, iter);
            stageFile = fullfile(snapshotDir, sprintf('stage_iter_%06d.mat', iter));
            save(stageFile, ...
                'iter', 'Pcur', 'Pinit', 'Pref', 'obstacles', 'params', ...
                'J', 'details', 'stageInfo', params.saveMatFlag);

            if params.verboseSave
                logMsg(logFID, '  [save] stage result saved: %s\n', stageFile);
            end
        end
    end

    Popt = unpackInterior(x, P);
    [info.finalJ, info.finalDetails] = objectiveCSSC2D(Popt, Pref, obstacles, params);
    info.finalMinClear = safeGet(info.finalDetails, 'minClear', NaN);
    info.finishedAt = datestr(now, 31);

    logMsg(logFID, '\nCSSC optimization finished: %s\n', info.finishedAt);
    logMsg(logFID, 'Final J        : %.8g\n', info.finalJ);
    logMsg(logFID, 'Final minClear : %.8g\n', info.finalMinClear);

    if params.enableResultSave
        finalFile = fullfile(resultDir, 'final_result.mat');
        save(finalFile, 'Popt', 'Pinit', 'Pref', 'obstacles', 'params', 'info', params.saveMatFlag);
        writeSummaryText(fullfile(resultDir, 'summary.txt'), info, params);
        logMsg(logFID, 'Final result saved: %s\n', finalFile);
    end
end

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
        xp = x; xm = x;
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

function [resultDir, snapshotDir, logDir] = prepareResultDirs(params)
    root = getProjectRootLocal(params);
    resultRoot = params.resultRoot;
    if ~isAbsolutePath(resultRoot)
        resultRoot = fullfile(root, resultRoot);
    end

    if isempty(params.runName)
        runName = ['run_', datestr(now, 'yyyymmdd_HHMMSS')];
    else
        runName = params.runName;
    end

    resultDir = fullfile(resultRoot, runName);
    snapshotDir = fullfile(resultDir, 'snapshots');
    logDir = fullfile(resultDir, 'logs');

    if ~exist(resultDir, 'dir'); mkdir(resultDir); end
    if ~exist(snapshotDir, 'dir'); mkdir(snapshotDir); end
    if ~exist(logDir, 'dir'); mkdir(logDir); end
end

function root = getProjectRootLocal(params)
    if isfield(params, 'projectRoot') && ~isempty(params.projectRoot)
        root = params.projectRoot;
        return;
    end

    try
        root = evalin('base', 'CSSC_PROJECT_ROOT');
        if ischar(root) || isstring(root)
            root = char(root);
            return;
        end
    catch
    end

    root = pwd;
end

function tf = isAbsolutePath(p)
    p = char(p);
    if ispc
        tf = numel(p) >= 2 && p(2) == ':';
    else
        tf = startsWith(p, filesep);
    end
end

function tf = shouldSaveSnapshot(iter, params)
    tf = mod(iter, params.saveInterval) == 0 || iter == 1 || iter == params.numIter;
end

function tf = shouldSaveResult(iter, params)
    tf = mod(iter, params.resultSaveInterval) == 0 || iter == 1 || iter == params.numIter;
end

function info2 = trimInfoToIter(info, iter)
    info2 = info;
    histFields = {'Jhist', 'minClearHist', 'gradNormHist'};
    for i = 1:numel(histFields)
        f = histFields{i};
        if isfield(info2, f)
            info2.(f) = info2.(f)(1:iter);
        end
    end

    if isfield(info2, 'timing')
        tf = fieldnames(info2.timing);
        for i = 1:numel(tf)
            f = tf{i};
            val = info2.timing.(f);
            if isnumeric(val) && numel(val) >= iter
                info2.timing.(f) = val(1:iter);
            end
        end
    end
end

function writeProgressHeader(csvFile)
    fid = fopen(csvFile, 'w');
    if fid < 0; return; end
    fprintf(fid, 'iter,J,minClear,gradNorm,totalIter,mainObjective,gradient,update,objCalls,objAll\n');
    fclose(fid);
end

function appendProgressLine(csvFile, iter, J, minClear, gradNorm, totalIter, mainObj, gradTime, updateTime, objCalls, objAll)
    fid = fopen(csvFile, 'a');
    if fid < 0; return; end
    fprintf(fid, '%d,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%d,%.15g\n', ...
        iter, J, minClear, gradNorm, totalIter, mainObj, gradTime, updateTime, objCalls, objAll);
    fclose(fid);
end

function writeSummaryText(summaryFile, info, params)
    fid = fopen(summaryFile, 'w');
    if fid < 0; return; end

    fprintf(fid, 'CSSC Optimization Summary\n');
    fprintf(fid, '=========================\n\n');
    fprintf(fid, 'Finished at        : %s\n', info.finishedAt);
    fprintf(fid, 'Result directory   : %s\n', info.resultDir);
    fprintf(fid, 'numIter            : %d\n', params.numIter);
    fprintf(fid, 'printInterval      : %d\n', params.printInterval);
    fprintf(fid, 'resultSaveInterval : %d\n', params.resultSaveInterval);
    fprintf(fid, 'Final J            : %.15g\n', info.finalJ);
    fprintf(fid, 'Final minClear     : %.15g\n', info.finalMinClear);

    if isfield(info, 'Jhist')
        [bestJ, bestIter] = min(info.Jhist);
        [bestClear, bestClearIter] = max(info.minClearHist);
        fprintf(fid, 'Best J             : %.15g at iter %d\n', bestJ, bestIter);
        fprintf(fid, 'Best minClear      : %.15g at iter %d\n', bestClear, bestClearIter);
    end

    fclose(fid);
end

function closeLogFile(fid)
    if fid > 2
        fclose(fid);
    end
end

function logMsg(fid, varargin)
    fprintf(varargin{:});
    if fid > 2
        fprintf(fid, varargin{:});
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

function printTimingDebug(logFID, info, iter, params)
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

    logMsg(logFID, '\n[TIMING] recent %d iters, up to iter %d\n', win, iter);
    logMsg(logFID, '  total / iter              : %.3f ms\n', tTotal * 1000);
    logMsg(logFID, '  main objective            : %.3f ms  (%5.1f%% total)\n', ...
        tMainObj * 1000, 100 * tMainObj / safeTotal);
    logMsg(logFID, '  finite-diff gradient      : %.3f ms  (%5.1f%% total)\n', ...
        tGrad * 1000, 100 * tGrad / safeTotal);
    logMsg(logFID, '  Adam update               : %.3f ms  (%5.1f%% total)\n', ...
        tUpdate * 1000, 100 * tUpdate / safeTotal);

    logMsg(logFID, '  objective calls / iter    : %.1f\n', objCalls);
    logMsg(logFID, '  avg objective call        : %.3f ms\n', objAvg * 1000);
    logMsg(logFID, '  all objective time        : %.3f ms\n', objAll * 1000);

    logMsg(logFID, '    fixedChordEnvelope      : %.3f ms  (%5.1f%% obj)\n', ...
        tEnvelope * 1000, 100 * tEnvelope / safeObjAll);
    logMsg(logFID, '    obstacle clearance      : %.3f ms  (%5.1f%% obj)\n', ...
        tObstacle * 1000, 100 * tObstacle / safeObjAll);
    logMsg(logFID, '    ref/smooth/length       : %.3f ms  (%5.1f%% obj)\n', ...
        tReg * 1000, 100 * tReg / safeObjAll);
    logMsg(logFID, '    curvature               : %.3f ms  (%5.1f%% obj)\n', ...
        tCurv * 1000, 100 * tCurv / safeObjAll);
    logMsg(logFID, '    other objective time    : %.3f ms  (%5.1f%% obj)\n', ...
        tOther * 1000, 100 * tOther / safeObjAll);
end

function v = safeGet(s, fieldName, defaultValue)
    if isstruct(s) && isfield(s, fieldName)
        v = s.(fieldName);
    else
        v = defaultValue;
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

    if ~isfield(params, 'enableResultSave'); params.enableResultSave = true; end
    if ~isfield(params, 'projectRoot'); params.projectRoot = ''; end
    if ~isfield(params, 'resultRoot'); params.resultRoot = 'results'; end
    if ~isfield(params, 'runName'); params.runName = ''; end
    if ~isfield(params, 'resultSaveInterval') || isempty(params.resultSaveInterval)
        params.resultSaveInterval = params.printInterval;
    end
    if ~isfield(params, 'verboseSave'); params.verboseSave = true; end
    if ~isfield(params, 'saveMatFlag'); params.saveMatFlag = '-v7'; end
end
