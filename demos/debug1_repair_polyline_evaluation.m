%% Repair degree-1 polyline fixed-chord evaluation in a saved baseline run
% This script does not rerun any planner or optimizer. It copies the source
% run to a new directory ending in "_evaluation_fixed", reevaluates every
% saved native polyline with the corrected chord solver, updates every
% result.mat and aggregate table, and regenerates the comparison figures.
% Saved cubic CSSC/RRTSC metrics are preserved unchanged.

clear; clc; close all;

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

sourceRunName = ...
    'run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_20260731_154814';
repairSuffix = '_evaluation_fixed';
sourceRunDir = fullfile(projectRoot, 'results', 'runs', sourceRunName);
targetRunName = [sourceRunName, repairSuffix];
targetRunDir = fullfile(projectRoot, 'results', 'runs', targetRunName);

if exist(sourceRunDir, 'dir') ~= 7
    error('Source run directory does not exist: %s', sourceRunDir);
end
if exist(targetRunDir, 'dir') == 7
    error(['Target directory already exists. Remove or rename it explicitly ' ...
        'before rerunning this repair: %s'], targetRunDir);
end

fprintf('[evaluation repair]\n  source: %s\n  target: %s\n', ...
    sourceRunDir, targetRunDir);
copyfile(sourceRunDir, targetRunDir);

batchPath = fullfile(targetRunDir, 'baseline_five_methods_results.mat');
if exist(batchPath, 'file') ~= 2
    error('Missing aggregate result file: %s', batchPath);
end
batch = load(batchPath);
requiredBatchFields = {'cfg','results','trialTable','summaryTable'};
for i = 1:numel(requiredBatchFields)
    if ~isfield(batch, requiredBatchFields{i})
        error('Aggregate result is missing: %s', requiredBatchFields{i});
    end
end

repairInfo = struct();
repairInfo.schemaVersion = 'polyline-evaluation-repair-v1';
repairInfo.createdAt = datestr(now, 'yyyy-mm-dd HH:MM:SS');
repairInfo.sourceRunName = sourceRunName;
repairInfo.targetRunName = targetRunName;
repairInfo.reason = ['Replace degree-1 B-spline Newton chord roots with ' ...
    'native polyline segment-circle first-forward roots.'];
repairInfo.plannersRerun = false;
repairInfo.optimizersRerun = false;
repairInfo.evaluator = 'evaluatePolylineHighPrecision2D';
repairInfo.chordSolver = ...
    'polyline-segment-quadratic-first-forward-root';

batch.cfg = updateOutputPaths(batch.cfg, targetRunDir, targetRunName);
caseFiles = dir(fullfile(targetRunDir, 'cases', 'case_*', 'result.mat'));
if isempty(caseFiles)
    error('No per-case result.mat files were found under the copied run.');
end

changeRows = repmat(emptyChangeRow(), numel(caseFiles), 1);
for i = 1:numel(caseFiles)
    resultPath = fullfile(caseFiles(i).folder, caseFiles(i).name);
    loaded = load(resultPath, 'result');
    if ~isfield(loaded, 'result')
        error('Missing result variable: %s', resultPath);
    end
    result = loaded.result;
    oldRow = result.summaryRow;
    oldMetrics = result.highPrecisionMetrics;

    evalOpts = findEvaluationOptions(result);
    [finalMetrics, finalRepresentation] = evaluateSavedFinalPath( ...
        result, evalOpts);
    [stageMetrics, stageTimes] = repairStageMetrics( ...
        result, evalOpts, finalMetrics);

    row = updateSummaryRow(oldRow, finalMetrics, stageMetrics, stageTimes);
    row.pathRepresentation = string(finalRepresentation);
    row.resultMatPath = string(resultPath);
    row = finalizeSolutionStatus(row);

    result.highPrecisionMetrics = finalMetrics;
    result.stageHighPrecisionMetrics = stageMetrics;
    result.summaryRow = row;
    result.successFlags = makeSuccessFlags(row);
    result.timing = updateTiming(result.timing, row, stageTimes);
    result.config = updateOutputPaths(result.config, targetRunDir, targetRunName);
    if ~isfield(result, 'provenance') || ~isstruct(result.provenance)
        result.provenance = struct();
    end
    result.provenance.evaluationRepair = repairInfo;
    result.evaluationRepair = repairInfo;

    caseId = row.caseId;
    if caseId < 1 || caseId > numel(batch.results)
        error('Invalid caseId %d in %s.', caseId, resultPath);
    end
    batch.results(caseId) = row;
    changeRows(i) = makeChangeRow(oldRow, row, oldMetrics, finalMetrics);

    saveResultAtomic(resultPath, result);
    if mod(i, 100) == 0 || i == numel(caseFiles)
        fprintf('  repaired %d / %d cases\n', i, numel(caseFiles));
    end
end

batch.trialTable = struct2table(batch.results);
batch.summaryTable = refreshEvaluationSummary( ...
    batch.summaryTable, batch.trialTable);
batch.cfg = updateOutputPaths(batch.cfg, targetRunDir, targetRunName);
batch.repairInfo = repairInfo;
if isfield(batch, 'runDefinition') && isstruct(batch.runDefinition)
    batch.runDefinition.evaluationRepair = repairInfo;
end
if isfield(batch, 'provenance')
    batch.provenance.evaluationRepair = repairInfo;
end

writeTableAtomic(batch.trialTable, ...
    fullfile(targetRunDir, 'trial_results.csv'));
writeTableAtomic(batch.summaryTable, ...
    fullfile(targetRunDir, 'summary_by_scene_difficulty_planner.csv'));
writeTableAtomic(batch.summaryTable, ...
    fullfile(targetRunDir, 'summary_mean_table.csv'));
changeTable = struct2table(changeRows);
writeTableAtomic(changeTable, ...
    fullfile(targetRunDir, 'evaluation_repair_changes.csv'));

saveStructAtomic(batchPath, batch);
repairCheckpoint(targetRunDir, batch.results, repairInfo);
repairProvenance(targetRunDir, repairInfo);
updateMetricDefinitions(targetRunDir, repairInfo);
writeRepairNotes(targetRunDir, repairInfo, changeTable);
regenerateFigures(batch.trialTable, batch.summaryTable, batch.cfg);

fprintf('\n[repair complete]\n  output: %s\n', targetRunDir);
fprintf('  changed numerical certification: %d cases\n', ...
    nnz(changeTable.oldFTLRealizable ~= changeTable.newFTLRealizable));
fprintf('  changed whole-body success      : %d cases\n', ...
    nnz(changeTable.oldWholeBodySuccess ~= changeTable.newWholeBodySuccess));

%% Repair helpers

function opts = findEvaluationOptions(result)
    opts = struct();
    candidates = {
        getNestedField(result, {'highPrecisionMetrics','paramsUsed'}, []), ...
        getNestedField(result, {'parameters','effective','evaluation'}, []), ...
        getNestedField(result, {'parameters','globalEvaluation'}, [])};
    for i = 1:numel(candidates)
        if isstruct(candidates{i}) && ~isempty(fieldnames(candidates{i}))
            opts = candidates{i};
            return;
        end
    end
end

function [metrics, representation] = evaluateSavedFinalPath(result, opts)
    planner = string(result.summaryRow.planner);
    if isPolylinePlanner(planner)
        path = firstNonemptyPath(result.paths, ...
            {'pathOptimized','Popt','pathFrontend','pathRRT'});
        metrics = evaluatePolylineHighPrecision2D( ...
            path, result.obstacles, opts);
        representation = 'piecewise-linear-exact-chord';
    else
        % This repair targets only the degree-1 polyline mismatch. Preserve
        % saved cubic-path metrics bit for bit instead of mixing in unrelated
        % evaluator changes made after the original experiment.
        metrics = result.highPrecisionMetrics;
        representation = char(getFieldOrDefault( ...
            result.summaryRow, 'pathRepresentation', ...
            'cubic-bspline-degree-3'));
    end
end

function [stage, times] = repairStageMetrics(result, opts, finalMetrics)
    planner = string(result.summaryRow.planner);
    stage = result.stageHighPrecisionMetrics;
    if ~isstruct(stage)
        stage = struct();
    end
    stage.final = finalMetrics;
    times = struct( ...
        'frontend', getFieldOrDefault(result.summaryRow, ...
            'frontendEvalTimeSec', nan), ...
        'initial', getFieldOrDefault(result.summaryRow, ...
            'initialEvalTimeSec', nan), ...
        'final', getFieldOrDefault(result.summaryRow, ...
            'evalTimeSec', finalMetrics.evalTimeSec), ...
        'total', getFieldOrDefault(result.summaryRow, ...
            'totalEvaluationTimeSec', finalMetrics.evalTimeSec));

    if isPolylinePlanner(planner)
        times.final = finalMetrics.evalTimeSec;
        times.total = times.final;
    end

    if strcmpi(planner, 'CSSC')
        frontendPath = firstNonemptyPath(result.paths, ...
            {'pathFrontend','pathRRT'});
        stage.frontend = evaluatePolylineHighPrecision2D( ...
            frontendPath, result.obstacles, opts);
        times.frontend = stage.frontend.evalTimeSec;
        times.total = sum([times.frontend, times.initial, times.final], ...
            'omitnan');
    end
end

function row = updateSummaryRow(row, metrics, stage, times)
    oldTotalEval = getFieldOrDefault(row, 'totalEvaluationTimeSec', nan);
    row.pointSuccess = metrics.pointSuccess;
    row.envelopeSuccess = metrics.success;
    row.envelopeDMinSatisfied = metrics.dMinSatisfied;
    row.chordConstructionSuccess = metrics.chordConstructionSuccess;
    row.ftlGeometricRealizable = metrics.ftlGeometricRealizable;
    row.feasibilityNumericalFailure = metrics.feasibilityNumericalFailure;
    row.pointMinClear = metrics.pointMinClear;
    row.pointMinClearX = metrics.pointMinPoint(1);
    row.pointMinClearY = metrics.pointMinPoint(2);
    row.envelopeMinClear = metrics.minClear;
    row.envelopeClearMargin = metrics.minClear - metrics.dMin;
    row.maxChordLengthResidual = metrics.maxChordLengthResidual;
    row.pathLength = metrics.pathLength;
    row.turnAbsSum = metrics.turnAbsSum;
    row.turnSqSum = metrics.turnSqSum;
    row.meanAbsTurn = metrics.meanAbsTurn;
    row.evalTimeSec = times.final;
    row.totalEvaluationTimeSec = times.total;
    row.wholeBodySuccess = row.envelopeDMinSatisfied && ...
        row.ftlGeometricRealizable;

    if isfield(stage, 'frontend') && isstruct(stage.frontend) && ...
            isfield(stage.frontend, 'minClear')
        row.frontendPointMinClear = stage.frontend.pointMinClear;
        row.frontendEnvelopeMinClear = stage.frontend.minClear;
        row.frontendEnvelopeDMinSatisfied = stage.frontend.dMinSatisfied;
        row.frontendFTLGeometricRealizable = ...
            stage.frontend.ftlGeometricRealizable;
        row.frontendEvalTimeSec = times.frontend;
    end
    if isfield(stage, 'initial') && isstruct(stage.initial) && ...
            isfield(stage.initial, 'minClear')
        row.initialPointMinClear = stage.initial.pointMinClear;
        row.initialEnvelopeMinClear = stage.initial.minClear;
        row.initialEnvelopeDMinSatisfied = stage.initial.dMinSatisfied;
        row.initialFTLGeometricRealizable = ...
            stage.initial.ftlGeometricRealizable;
        row.initialPathLengthHP = stage.initial.pathLength;
        row.initialEvalTimeSec = times.initial;
        row.clearanceGainFromInitial = ...
            metrics.minClear - stage.initial.minClear;
        row.pathLengthChangeFromInitial = ...
            metrics.pathLength - stage.initial.pathLength;
    end
    if isfinite(row.totalTimeSec) && isfinite(oldTotalEval)
        row.totalTimeSec = max(0, row.totalTimeSec - oldTotalEval) + ...
            row.totalEvaluationTimeSec;
    end
end

function row = finalizeSolutionStatus(row)
    row.pathFound = row.plannerSuccess;
    row.centerlineCollisionFree = row.pointSuccess;
    row.wholeBodyCollisionFree = row.envelopeSuccess;
    row.dMinSatisfied = row.envelopeDMinSatisfied;
    row.numericallyCertified = row.ftlGeometricRealizable;
    row.strictAccepted = row.wholeBodySuccess;
    if ~row.pathFound
        row.solutionStatus = "no-path";
    elseif ~row.centerlineCollisionFree
        row.solutionStatus = "centerline-collision";
    elseif ~row.numericallyCertified
        row.solutionStatus = "numerically-uncertified";
    elseif row.strictAccepted
        row.solutionStatus = "strict-safe";
    elseif row.wholeBodyCollisionFree
        row.solutionStatus = "collision-free-below-dmin";
    else
        row.solutionStatus = "centerline-only";
    end
end

function flags = makeSuccessFlags(row)
    flags = struct( ...
        'plannerSuccess', row.plannerSuccess, ...
        'pathFound', row.pathFound, ...
        'highPrecisionSuccess', row.envelopeSuccess, ...
        'dMinSatisfied', row.envelopeDMinSatisfied, ...
        'ftlGeometricRealizable', row.ftlGeometricRealizable, ...
        'wholeBodySuccess', row.wholeBodySuccess, ...
        'fallbackReturned', row.fallbackReturned, ...
        'solutionStatus', row.solutionStatus, ...
        'pointSuccess', row.pointSuccess);
end

function timing = updateTiming(timing, row, stageTimes)
    if ~isstruct(timing); timing = struct(); end
    timing.evalTimeSec = row.evalTimeSec;
    timing.finalEvalTimeSec = row.evalTimeSec;
    timing.totalEvaluationTimeSec = stageTimes.total;
    if isfinite(stageTimes.frontend)
        timing.frontendEvalTimeSec = stageTimes.frontend;
    end
    if isfinite(stageTimes.initial)
        timing.initialEvalTimeSec = stageTimes.initial;
    end
end

function tf = isPolylinePlanner(planner)
    tf = any(strcmpi(planner, ["RRT", "RRT*", "Sp-RRT-2D"]));
end

function path = firstNonemptyPath(paths, names)
    path = zeros(0, 2);
    for i = 1:numel(names)
        if isstruct(paths) && isfield(paths, names{i}) && ...
                ~isempty(paths.(names{i}))
            path = paths.(names{i});
            return;
        end
    end
end

function summary = refreshEvaluationSummary(summary, T)
    if ~ismember('feasibilityNumericalFailureRate', summary.Properties.VariableNames)
        summary.feasibilityNumericalFailureRate = nan(height(summary), 1);
    end
    if ~ismember('chordConstructionSuccessRate', summary.Properties.VariableNames)
        summary.chordConstructionSuccessRate = nan(height(summary), 1);
    end
    if ~ismember('maxChordLengthResidualMean', summary.Properties.VariableNames)
        summary.maxChordLengthResidualMean = nan(height(summary), 1);
    end

    for i = 1:height(summary)
        mask = strcmp(T.sceneId, summary.sceneId(i)) & ...
            T.difficultyIndex == summary.difficultyIndex(i) & ...
            strcmp(T.planner, summary.planner(i));
        R = T(mask,:);
        n = height(R);
        summary.nValidEnvelope(i) = nnz(isfinite(R.envelopeMinClear));
        summary.nValidPathQuality(i) = nnz(isfinite(R.pathLength));
        summary.nValidFrontendHP(i) = nnz(isfinite(R.frontendEnvelopeMinClear));
        summary.nValidInitialHP(i) = nnz(isfinite(R.initialEnvelopeMinClear));
        summary.pointSuccessRate(i) = safeRate(R.pointSuccess, n);
        summary.envelopeSuccessRate(i) = safeRate(R.envelopeSuccess, n);
        summary.envelopeDMinRate(i) = safeRate(R.envelopeDMinSatisfied, n);
        summary.ftlGeometricRealizableRate(i) = ...
            safeRate(R.ftlGeometricRealizable, n);
        summary.wholeBodySuccessRate(i) = safeRate(R.wholeBodySuccess, n);
        summary.chordConstructionSuccessRate(i) = ...
            safeRate(R.chordConstructionSuccess, n);
        summary.feasibilityNumericalFailureRate(i) = ...
            safeRate(R.feasibilityNumericalFailure, n);
        summary.pointClearMean(i) = safeMean(R.pointMinClear);
        summary.envelopeClearMean(i) = safeMean(R.envelopeMinClear);
        summary.pointClearMin(i) = safeMin(R.pointMinClear);
        summary.envelopeClearMin(i) = safeMin(R.envelopeMinClear);
        summary.envelopeClearMarginMean(i) = safeMean(R.envelopeClearMargin);
        summary.envelopeClearMarginMin(i) = safeMin(R.envelopeClearMargin);
        summary.frontendDMinRate(i) = safeFiniteRate( ...
            R.frontendEnvelopeDMinSatisfied);
        summary.initialDMinRate(i) = safeFiniteRate( ...
            R.initialEnvelopeDMinSatisfied);
        summary.frontendEnvelopeClearMean(i) = ...
            safeMean(R.frontendEnvelopeMinClear);
        summary.initialEnvelopeClearMean(i) = ...
            safeMean(R.initialEnvelopeMinClear);
        summary.clearanceGainFromInitialMean(i) = ...
            safeMean(R.clearanceGainFromInitial);
        summary.pathLengthMean(i) = safeMean(R.pathLength);
        summary.pathLengthChangeFromInitialMean(i) = ...
            safeMean(R.pathLengthChangeFromInitial);
        summary.turnAbsMean(i) = safeMean(R.turnAbsSum);
        summary.turnSqMean(i) = safeMean(R.turnSqSum);
        summary.evalTimeMeanSec(i) = safeMean(R.evalTimeSec);
        summary.frontendEvalTimeMeanSec(i) = safeMean(R.frontendEvalTimeSec);
        summary.initialEvalTimeMeanSec(i) = safeMean(R.initialEvalTimeSec);
        summary.totalEvaluationTimeMeanSec(i) = ...
            safeMean(R.totalEvaluationTimeSec);
        summary.totalTimeMeanSec(i) = safeMean(R.totalTimeSec);
        summary.maxChordLengthResidualMean(i) = ...
            safeMean(R.maxChordLengthResidual);
    end
end

function row = emptyChangeRow()
    row = struct('caseId', 0, 'planner', "", 'sceneId', "", ...
        'difficultyId', "", 'trialIndex', 0, ...
        'oldMinClear', nan, 'newMinClear', nan, 'minClearChange', nan, ...
        'oldDMinSatisfied', false, 'newDMinSatisfied', false, ...
        'oldFTLRealizable', false, 'newFTLRealizable', false, ...
        'oldNumericalFailure', false, 'newNumericalFailure', false, ...
        'oldWholeBodySuccess', false, 'newWholeBodySuccess', false, ...
        'oldMaxChordResidual', nan, 'newMaxChordResidual', nan);
end

function row = makeChangeRow(oldRow, newRow, oldMetrics, newMetrics)
    row = emptyChangeRow();
    row.caseId = newRow.caseId;
    row.planner = newRow.planner;
    row.sceneId = newRow.sceneId;
    row.difficultyId = newRow.difficultyId;
    row.trialIndex = newRow.trialIndex;
    row.oldMinClear = getFieldOrDefault(oldRow, 'envelopeMinClear', nan);
    row.newMinClear = newRow.envelopeMinClear;
    row.minClearChange = row.newMinClear - row.oldMinClear;
    row.oldDMinSatisfied = logical(getFieldOrDefault( ...
        oldRow, 'envelopeDMinSatisfied', false));
    row.newDMinSatisfied = newRow.envelopeDMinSatisfied;
    row.oldFTLRealizable = logical(getFieldOrDefault( ...
        oldRow, 'ftlGeometricRealizable', false));
    row.newFTLRealizable = newRow.ftlGeometricRealizable;
    row.oldNumericalFailure = logical(getFieldOrDefault( ...
        oldMetrics, 'feasibilityNumericalFailure', false));
    row.newNumericalFailure = newMetrics.feasibilityNumericalFailure;
    row.oldWholeBodySuccess = logical(getFieldOrDefault( ...
        oldRow, 'wholeBodySuccess', false));
    row.newWholeBodySuccess = newRow.wholeBodySuccess;
    row.oldMaxChordResidual = getFieldOrDefault( ...
        oldMetrics, 'maxChordLengthResidual', nan);
    row.newMaxChordResidual = newMetrics.maxChordLengthResidual;
end

function cfg = updateOutputPaths(cfg, targetDir, targetName)
    if ~isstruct(cfg); return; end
    cfg.runName = targetName;
    cfg.outDir = targetDir;
    cfg.figDir = fullfile(targetDir, 'figures');
    cfg.caseDir = fullfile(targetDir, 'cases');
    cfg.checkpointPath = fullfile(targetDir, 'batch_checkpoint.mat');
    cfg.provenancePath = fullfile(targetDir, 'experiment_provenance.mat');
    cfg.provenanceTextPath = fullfile(targetDir, 'experiment_provenance.txt');
end

function repairCheckpoint(targetDir, results, repairInfo)
    path = fullfile(targetDir, 'batch_checkpoint.mat');
    if exist(path, 'file') ~= 2; return; end
    loaded = load(path, 'checkpoint');
    if ~isfield(loaded, 'checkpoint'); return; end
    checkpoint = loaded.checkpoint;
    checkpoint.results = results;
    checkpoint.repairInfo = repairInfo;
    checkpoint.savedAt = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
    tmp = [tempname(targetDir), '.mat'];
    save(tmp, 'checkpoint', '-v7.3');
    movefile(tmp, path, 'f');
end

function repairProvenance(targetDir, repairInfo)
    path = fullfile(targetDir, 'experiment_provenance.mat');
    if exist(path, 'file') == 2
        loaded = load(path, 'provenance');
        if isfield(loaded, 'provenance')
            provenance = loaded.provenance;
            provenance.evaluationRepair = repairInfo;
            save(path, 'provenance');
        end
    end
    textPath = fullfile(targetDir, 'experiment_provenance.txt');
    fid = fopen(textPath, 'a');
    if fid >= 0
        fprintf(fid, '\nOffline evaluation repair\n');
        fprintf(fid, 'createdAt: %s\n', repairInfo.createdAt);
        fprintf(fid, 'sourceRunName: %s\n', repairInfo.sourceRunName);
        fprintf(fid, 'chordSolver: %s\n', repairInfo.chordSolver);
        fprintf(fid, 'planners/optimizers rerun: false\n');
        fclose(fid);
    end
end

function writeRepairNotes(targetDir, repairInfo, changes)
    path = fullfile(targetDir, 'evaluation_repair_notes.txt');
    fid = fopen(path, 'w');
    if fid < 0; error('Could not write: %s', path); end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'Polyline fixed-chord evaluation repair\n\n');
    fprintf(fid, 'Source run: %s\n', repairInfo.sourceRunName);
    fprintf(fid, 'Created: %s\n', repairInfo.createdAt);
    fprintf(fid, 'No planner or optimizer was rerun.\n');
    fprintf(fid, ['RRT, RRT*, Sp-RRT-2D, and the CSSC frontend are now ' ...
        'evaluated on their native polylines using exact segment-circle ' ...
        'first-forward chord roots.\n']);
    fprintf(fid, ['RRTSC-2D and final CSSC paths retain the cubic B-spline ' ...
        'high-precision evaluator.\n']);
    fprintf(fid, 'Changed FTL certification: %d\n', ...
        nnz(changes.oldFTLRealizable ~= changes.newFTLRealizable));
    fprintf(fid, 'Changed whole-body success: %d\n', ...
        nnz(changes.oldWholeBodySuccess ~= changes.newWholeBodySuccess));
end

function updateMetricDefinitions(targetDir, repairInfo)
    path = fullfile(targetDir, 'metric_definitions.txt');
    if exist(path, 'file') == 2
        oldText = fileread(path);
    else
        oldText = '';
    end
    fid = fopen(path, 'w');
    if fid < 0; error('Could not write: %s', path); end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, ['EVALUATION REPAIR NOTICE (%s)\n' ...
        'This notice supersedes any degree-1 B-spline/Newton wording below.\n' ...
        'RRT, RRT*, Sp-RRT-2D, and the CSSC frontend retain native ' ...
        'polylines and use exact segment-circle first-forward chord roots.\n' ...
        'CSSC and RRTSC-2D cubic-path metrics were copied unchanged.\n' ...
        'No planner or optimizer was rerun.\n\n'], repairInfo.createdAt);
    fprintf(fid, '%s', oldText);
end

function regenerateFigures(T, S, cfg)
    if ~exist(cfg.figDir, 'dir'); mkdir(cfg.figDir); end
    specs = {
        'envelopeClearMargin','Safety-clearance margin','minClear - dMin','envelope_clearance_boxplot.jpg'
        'pathLength','Path length','length','path_length_boxplot.jpg'
        'turnAbsSum','Path turning','turn sum','turning_boxplot.jpg'
        'planTimeSec','Planning runtime','time (s)','runtime_boxplot.jpg'};
    for i = 1:size(specs,1)
        plotMetric(T, cfg, specs{i,1}, specs{i,2}, specs{i,3}, specs{i,4});
    end
    plotOverview(S, cfg);
    plotRRTSCStages(S, cfg);
end

function plotMetric(T, cfg, metricName, figTitle, yLabelText, fileName)
    sceneIds = unique(T.sceneId, 'stable');
    difficultyIndices = unique(T.difficultyIndex, 'stable');
    planners = string(cfg.plannerNames);
    fig = figure('Color','w','Name',figTitle,'Position',[60 40 1500 920]);
    tiledlayout(numel(difficultyIndices), numel(sceneIds), ...
        'Padding','compact','TileSpacing','compact');
    for d = 1:numel(difficultyIndices)
        for i = 1:numel(sceneIds)
            nexttile;
            rows = T(strcmp(T.sceneId,sceneIds(i)) & ...
                T.difficultyIndex == difficultyIndices(d),:);
            y = rows.(metricName);
            group = nan(height(rows),1);
            nSuccess = zeros(1,numel(planners));
            nTotal = zeros(1,numel(planners));
            for j = 1:numel(planners)
                mask = strcmp(rows.planner,planners(j));
                group(mask) = j;
                nTotal(j) = nnz(mask);
                nSuccess(j) = nnz(mask & rows.wholeBodySuccess);
            end
            valid = isfinite(y);
            if any(valid)
                boxplot(y(valid),group(valid),'Positions', ...
                    unique(group(valid),'stable'),'Symbol','k.');
            else
                text(0.5,0.5,'no valid data','HorizontalAlignment','center');
            end
            xlim([0.5,numel(planners)+0.5]);
            set(gca,'XTick',1:numel(planners),'XTickLabel',cellstr(planners), ...
                'TickLabelInterpreter','none');
            if strcmp(metricName,'envelopeClearMargin')
                yline(0,'--','Color',[0.75 0.15 0.15]);
            elseif strcmp(metricName,'planTimeSec') && any(valid) && all(y(valid)>0)
                set(gca,'YScale','log');
            end
            grid on;
            for j = 1:numel(planners)
                text((j-0.5)/numel(planners),0.02, ...
                    sprintf('%d/%d',nSuccess(j),nTotal(j)), ...
                    'Units','normalized','HorizontalAlignment','center', ...
                    'VerticalAlignment','bottom','FontSize',7, ...
                    'Color',[0.25 0.25 0.25],'BackgroundColor','white');
            end
            title(sprintf('%s | %s',char(sceneIds(i)), ...
                char(cfg.difficultyNames(difficultyIndices(d)))), ...
                'Interpreter','none');
            if i == 1; ylabel(yLabelText); end
        end
    end
    sgtitle(figTitle,'Interpreter','none');
    exportgraphics(fig,fullfile(cfg.figDir,fileName), ...
        'Resolution',cfg.figureResolution,'BackgroundColor','white');
    savefig(fig,fullfile(cfg.figDir,strrep(fileName,'.jpg','.fig')));
    close(fig);
end

function plotOverview(S, cfg)
    sceneIds = unique(S.sceneId,'stable');
    difficulties = unique(S.difficultyIndex,'stable');
    planners = string(cfg.plannerNames);
    fig = figure('Color','w','Position',[60 60 1500 720]);
    tiledlayout(2,numel(sceneIds),'Padding','compact','TileSpacing','compact');
    for i = 1:numel(sceneIds)
        nexttile;
        colororder(cfg.plannerColors);
        bar(summaryMatrix(S,sceneIds(i),difficulties,planners, ...
            'wholeBodySuccessRate'));
        ylim([0 1]); grid on; ylabel('whole-body success rate');
        title(char(sceneIds(i)),'Interpreter','none');
        set(gca,'XTick',1:numel(difficulties),'XTickLabel', ...
            cellstr(cfg.difficultyNames(difficulties)));
        nexttile(numel(sceneIds)+i);
        colororder(cfg.plannerColors);
        bar(summaryMatrix(S,sceneIds(i),difficulties,planners, ...
            'planTimeMeanSec'));
        set(gca,'YScale','log','XTick',1:numel(difficulties), ...
            'XTickLabel',cellstr(cfg.difficultyNames(difficulties)));
        grid on; ylabel('mean planning time (s)');
        if i == numel(sceneIds)
            legend(cellstr(planners),'Location','southoutside', ...
                'Orientation','horizontal');
        end
    end
    sgtitle(sprintf('Mean results over %d trials',cfg.numSeedsPerScene));
    exportgraphics(fig,fullfile(cfg.figDir,'summary_mean_overview.jpg'), ...
        'Resolution',cfg.figureResolution,'BackgroundColor','white');
    savefig(fig,fullfile(cfg.figDir,'summary_mean_overview.fig'));
    close(fig);
end

function plotRRTSCStages(S, cfg)
    sceneIds = unique(S.sceneId,'stable');
    difficulties = unique(S.difficultyIndex,'stable');
    fig = figure('Color','w','Position',[80 80 1400 420]);
    tiledlayout(1,numel(sceneIds),'Padding','compact','TileSpacing','compact');
    for i = 1:numel(sceneIds)
        values = nan(numel(difficulties),3);
        for d = 1:numel(difficulties)
            row = S(strcmp(S.sceneId,sceneIds(i)) & ...
                S.difficultyIndex == difficulties(d) & ...
                strcmpi(S.planner,'RRTSC-2D'),:);
            if ~isempty(row)
                values(d,:) = [row.rrtscC1Rate(1), ...
                    row.rrtscC2GivenC1Rate(1),row.rrtscC3GivenC2Rate(1)];
            end
        end
        nexttile; bar(values); ylim([0 1]); grid on;
        title(char(sceneIds(i)),'Interpreter','none');
        set(gca,'XTick',1:numel(difficulties),'XTickLabel', ...
            cellstr(cfg.difficultyNames(difficulties)));
        if i == numel(sceneIds)
            legend({'Collision 1','Collision 2 | C1','Collision 3 | C2'}, ...
                'Location','southoutside','Orientation','horizontal');
        end
    end
    sgtitle('RRTSC-2D attempt-level stage pass rates');
    exportgraphics(fig,fullfile(cfg.figDir,'rrtsc_stage_pass_rates.jpg'), ...
        'Resolution',cfg.figureResolution,'BackgroundColor','white');
    savefig(fig,fullfile(cfg.figDir,'rrtsc_stage_pass_rates.fig'));
    close(fig);
end

function data = summaryMatrix(S, sceneId, difficulties, planners, fieldName)
    data = nan(numel(difficulties),numel(planners));
    for d = 1:numel(difficulties)
        for j = 1:numel(planners)
            row = S(strcmp(S.sceneId,sceneId) & ...
                S.difficultyIndex == difficulties(d) & ...
                strcmp(S.planner,planners(j)),:);
            if ~isempty(row)
                values = row.(fieldName);
                data(d,j) = values(1);
            end
        end
    end
end

function value = getNestedField(s, names, defaultValue)
    value = s;
    for i = 1:numel(names)
        if ~isstruct(value) || ~isfield(value,names{i})
            value = defaultValue;
            return;
        end
        value = value.(names{i});
    end
end

function value = getFieldOrDefault(s, name, defaultValue)
    if isstruct(s) && isfield(s,name); value = s.(name); else; value = defaultValue; end
end

function value = safeMean(x)
    x = x(isfinite(x)); if isempty(x); value = nan; else; value = mean(x); end
end

function value = safeMin(x)
    x = x(isfinite(x)); if isempty(x); value = nan; else; value = min(x); end
end

function value = safeRate(x, n)
    if n <= 0; value = nan; else; value = nnz(logical(x)) / n; end
end

function value = safeFiniteRate(x)
    valid = isfinite(double(x));
    if ~any(valid); value = nan; else; value = mean(double(x(valid))); end
end

function saveResultAtomic(path, result)
    folder = fileparts(path);
    tmp = [tempname(folder), '.mat'];
    save(tmp, 'result', '-v7.3');
    movefile(tmp, path, 'f');
end

function saveStructAtomic(path, data)
    folder = fileparts(path);
    tmp = [tempname(folder), '.mat'];
    save(tmp, '-struct', 'data', '-v7.3');
    movefile(tmp, path, 'f');
end

function writeTableAtomic(T, path)
    folder = fileparts(path);
    [~,~,extension] = fileparts(path);
    tmp = [tempname(folder), extension];
    writetable(T,tmp);
    movefile(tmp,path,'f');
end
