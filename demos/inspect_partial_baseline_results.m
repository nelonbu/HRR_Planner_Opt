clear; clc;

%% Inspect saved cases from an interrupted or still-running baseline batch
% This script is read-only with respect to case result.mat files. It prints
% progress, recent cases, and saved-case summaries. Completed cases are
% normally saved when saveCaseResults=true; an interrupted in-flight case
% or an unexpected exception can still be absent. Rates reported here are
% therefore explicitly "saved cases only", with missing IDs listed.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

runName = ...
    'run_baseline_rrt_rrtstar_rrtconnect_rrtsc_adaptive_sprrt_cssc_20260728_221310';
runDir = fullfile(projectRoot, 'results', 'runs', runName);
recentCount = 20;
writePartialCsv = true;
makePlots = true;

caseRoot = fullfile(runDir, 'cases');
if ~isfolder(caseRoot)
    error('Case directory does not exist: %s', caseRoot);
end

files = dir(fullfile(caseRoot, 'case_*', 'result.mat'));
if isempty(files)
    error('No per-case result.mat files were found under: %s', caseRoot);
end

fprintf('\n[partial baseline result inspection]\n');
fprintf('  run dir          : %s\n', runDir);
fprintf('  result.mat files : %d\n', numel(files));

rowCells = cell(numel(files), 1);
sourcePaths = strings(numel(files), 1);
savedAt = NaT(numel(files), 1);
validFile = false(numel(files), 1);
badFiles = strings(0, 1);
firstConfig = struct();

for i = 1:numel(files)
    filePath = fullfile(files(i).folder, files(i).name);
    try
        loaded = load(filePath, 'result');
        if ~isfield(loaded, 'result') || ...
                ~isfield(loaded.result, 'summaryRow')
            error('Missing result.summaryRow.');
        end

        rowCells{i} = loaded.result.summaryRow;
        sourcePaths(i) = string(filePath);
        savedAt(i) = datetime(files(i).datenum, ...
            'ConvertFrom', 'datenum');
        validFile(i) = true;

        if isempty(fieldnames(firstConfig)) && ...
                isfield(loaded.result, 'config')
            firstConfig = loaded.result.config;
        end
    catch ME
        badFiles(end+1, 1) = string(filePath) + " | " + ...
            string(ME.message); %#ok<SAGROW>
    end

    if mod(i, 250) == 0 || i == numel(files)
        fprintf('  loaded           : %d / %d\n', i, numel(files));
    end
end

if ~any(validFile)
    error('No readable result.summaryRow records were found.');
end

rows = vertcat(rowCells{validFile});
trialTable = struct2table(rows);
trialTable.sourceMatPath = sourcePaths(validFile);
trialTable.fileSavedAt = savedAt(validFile);
trialTable = sortrows(trialTable, 'caseId');

caseIds = double(trialTable.caseId);
maxCaseId = max(caseIds);
missingCaseIds = setdiff((1:maxCaseId).', caseIds);

[expectedTotal, nSceneGroups, nTrials, plannerNames] = ...
    inferBatchDimensions(firstConfig);

fprintf('\n[progress]\n');
fprintf('  readable files   : %d\n', height(trialTable));
fprintf('  last saved case  : %d\n', maxCaseId);
if isfinite(expectedTotal)
    fprintf('  expected cases   : %d\n', expectedTotal);
    fprintf('  traversal        : %.2f %%\n', ...
        100 * maxCaseId / expectedTotal);
end
fprintf('  missing IDs <= last saved case: %d\n', numel(missingCaseIds));
fprintf('  unreadable files : %d\n', numel(badFiles));

lastRow = trialTable(end, :);
fprintf('\n[last saved result]\n');
fprintf('  case / trial     : %d / %d\n', ...
    lastRow.caseId, lastRow.trialIndex);
fprintf('  scene / level    : %s / %s\n', ...
    char(lastRow.sceneId), char(lastRow.difficultyId));
fprintf('  planner          : %s\n', char(lastRow.planner));
fprintf('  planner success  : %d\n', lastRow.plannerSuccess);
fprintf('  planning time    : %.3f s\n', lastRow.planTimeSec);
fprintf('  saved at         : %s\n', ...
    char(lastRow.fileSavedAt));

if isfinite(expectedTotal) && maxCaseId < expectedTotal
    nextCaseId = maxCaseId + 1;
    casesPerGroup = nTrials * numel(plannerNames);
    nextGroup = floor((nextCaseId - 1) / casesPerGroup) + 1;
    withinGroup = mod(nextCaseId - 1, casesPerGroup);
    nextTrial = floor(withinGroup / numel(plannerNames)) + 1;
    nextPlannerIndex = mod(withinGroup, numel(plannerNames)) + 1;

    fprintf('\n[next expected or currently running]\n');
    fprintf('  case             : %d\n', nextCaseId);
    fprintf('  scene group      : %d / %d\n', nextGroup, nSceneGroups);
    fprintf('  trial            : %d / %d\n', nextTrial, nTrials);
    fprintf('  planner          : %s\n', ...
        char(plannerNames(nextPlannerIndex)));
end

if ~isempty(missingCaseIds)
    nShow = min(30, numel(missingCaseIds));
    fprintf('\n[first missing case IDs]\n  ');
    fprintf('%d ', missingCaseIds(1:nShow));
    fprintf('\n');
    fprintf(['  Note: missing IDs usually indicate an interrupted in-flight ' ...
        'case, an exception, or a run made without per-case saving.\n']);
end

if ~isempty(badFiles)
    fprintf('\n[unreadable files]\n');
    disp(badFiles);
end

recentColumns = intersect( ...
    {'caseId', 'trialIndex', 'sceneId', 'difficultyId', 'planner', ...
     'plannerSuccess', 'envelopeDMinSatisfied', 'planTimeSec', ...
     'numIter', 'numNodes'}, ...
    trialTable.Properties.VariableNames, 'stable');
fprintf('\n[most recent %d saved cases]\n', ...
    min(recentCount, height(trialTable)));
disp(trialTable(max(1, height(trialTable)-recentCount+1):end, ...
    recentColumns));

summaryTable = summarizeSavedCases(trialTable);
fprintf('\n[all readable saved cases by planner]\n');
disp(summaryTable);

currentMask = strcmp(string(trialTable.sceneId), ...
        string(lastRow.sceneId)) & ...
    trialTable.difficultyIndex == lastRow.difficultyIndex;
currentSummary = summarizeSavedCases(trialTable(currentMask, :));
fprintf('\n[current scene and difficulty by planner]\n');
disp(currentSummary);

fprintf(['\nImportant: the summaries above use readable saved result.mat ' ...
    'files only.\nUse the final batch CSV for formal rates after the run ' ...
    'completes.\n']);

if writePartialCsv
    trialCsv = fullfile(runDir, 'partial_trial_results_saved_only.csv');
    summaryCsv = fullfile(runDir, 'partial_summary_saved_only.csv');
    writetable(trialTable, trialCsv);
    writetable(summaryTable, summaryCsv);
    fprintf('\n[partial CSV output]\n');
    fprintf('  %s\n', trialCsv);
    fprintf('  %s\n', summaryCsv);
end

if makePlots
    plotCfg = makePartialPlotConfig(firstConfig, runDir);
    groupedSummary = summarizeSavedCasesByGroup(trialTable, plotCfg);
    plotPartialBaselineFigures( ...
        trialTable, groupedSummary, plotCfg, expectedTotal);
    fprintf('\n[partial figure output]\n');
    fprintf('  %s\n', plotCfg.figDir);
end

%% Local helpers

function [expectedTotal, nSceneGroups, nTrials, plannerNames] = ...
        inferBatchDimensions(cfg)
    expectedTotal = nan;
    nSceneGroups = nan;
    nTrials = nan;
    plannerNames = strings(0, 1);

    required = {'numSceneFamilies', 'numDifficultyLevels', ...
        'numSeedsPerScene', 'plannerNames'};
    if ~isstruct(cfg) || ~all(isfield(cfg, required))
        return;
    end

    nSceneGroups = cfg.numSceneFamilies * cfg.numDifficultyLevels;
    nTrials = cfg.numSeedsPerScene;
    plannerNames = string(cfg.plannerNames(:));
    expectedTotal = nSceneGroups * nTrials * numel(plannerNames);
end

function summary = summarizeSavedCases(T)
    planners = unique(string(T.planner), 'stable');
    template = struct( ...
        'planner', "", ...
        'nSaved', 0, ...
        'nPlannerSuccess', 0, ...
        'nDMinSuccess', 0, ...
        'plannerSuccessRateSaved', nan, ...
        'dMinRateSaved', nan, ...
        'planTimeMeanSec', nan, ...
        'planTimeMedianSec', nan, ...
        'planTimeMaxSec', nan, ...
        'envelopeClearMean', nan, ...
        'numIterMean', nan, ...
        'numNodesMean', nan, ...
        'candidatesPerExpandMean', nan, ...
        'abandonedCandidatesMean', nan, ...
        'collisionRejectMean', nan, ...
        'angleRejectMean', nan);
    rows = repmat(template, numel(planners), 1);

    for i = 1:numel(planners)
        mask = strcmp(string(T.planner), planners(i));
        Tp = T(mask, :);
        rows(i).planner = planners(i);
        rows(i).nSaved = height(Tp);
        rows(i).nPlannerSuccess = sum(Tp.plannerSuccess);
        rows(i).nDMinSuccess = sum(Tp.envelopeDMinSatisfied);
        rows(i).plannerSuccessRateSaved = ...
            rows(i).nPlannerSuccess / max(1, rows(i).nSaved);
        rows(i).dMinRateSaved = ...
            rows(i).nDMinSuccess / max(1, rows(i).nSaved);
        rows(i).planTimeMeanSec = finiteMean(Tp.planTimeSec);
        rows(i).planTimeMedianSec = finiteMedian(Tp.planTimeSec);
        rows(i).planTimeMaxSec = finiteMax(Tp.planTimeSec);
        rows(i).envelopeClearMean = finiteMean(Tp.envelopeMinClear);
        rows(i).numIterMean = finiteMean(Tp.numIter);
        rows(i).numNodesMean = finiteMean(Tp.numNodes);
        rows(i).candidatesPerExpandMean = ...
            finiteMean(Tp.averageCandidatesPerExpand);
        rows(i).abandonedCandidatesMean = ...
            finiteMean(Tp.abandonedCandidateCount);
        rows(i).collisionRejectMean = ...
            finiteMean(Tp.collisionRejectCount);
        rows(i).angleRejectMean = finiteMean(Tp.angleRejectCount);
    end

    summary = struct2table(rows);
end

function cfg = makePartialPlotConfig(sourceCfg, runDir)
    cfg = sourceCfg;
    cfg.figDir = fullfile(runDir, 'figures', 'partial');
    cfg.sceneIds = [ ...
        "double_slit", ...
        "s_channel", ...
        "staggered_baffles", ...
        "random_mixed"];

    if ~isfield(cfg, 'plannerNames') || isempty(cfg.plannerNames)
        cfg.plannerNames = {'RRT', 'RRT*', 'RRT-Connect', 'RRTSC-2D', ...
            'Adaptive Sp-RRT-2D', 'CSSC (RRT)', ...
            'CSSC (RRT-Connect)'};
    end
    if ~isfield(cfg, 'plannerColors') || ...
            size(cfg.plannerColors, 1) ~= numel(cfg.plannerNames)
        cfg.plannerColors = [
            0.20, 0.45, 0.75
            0.30, 0.65, 0.40
            0.85, 0.65, 0.15
            0.85, 0.45, 0.18
            0.10, 0.62, 0.66
            0.55, 0.25, 0.65
            0.75, 0.25, 0.50
        ];
    end
    if ~isfield(cfg, 'difficultyNames') || isempty(cfg.difficultyNames)
        cfg.difficultyNames = ["Easy", "Normal", "Hard"];
    end
    if ~isfield(cfg, 'numSeedsPerScene') || ...
            isempty(cfg.numSeedsPerScene)
        cfg.numSeedsPerScene = 50;
    end
    if ~isfield(cfg, 'figureResolution') || isempty(cfg.figureResolution)
        cfg.figureResolution = 300;
    end
end

function summary = summarizeSavedCasesByGroup(T, cfg)
    planners = string(cfg.plannerNames);
    difficultyIndices = 1:numel(cfg.difficultyNames);
    template = struct( ...
        'sceneId', "", ...
        'difficultyIndex', 0, ...
        'difficultyId', "", ...
        'planner', "", ...
        'nSaved', 0, ...
        'envelopeDMinRate', nan, ...
        'planTimeMeanSec', nan, ...
        'rrtscC1Rate', nan, ...
        'rrtscC2GivenC1Rate', nan, ...
        'rrtscC3GivenC2Rate', nan);
    rows = repmat(template, ...
        numel(cfg.sceneIds) * numel(difficultyIndices) * ...
        numel(planners), 1);
    k = 0;

    for i = 1:numel(cfg.sceneIds)
        for d = difficultyIndices
            for j = 1:numel(planners)
                mask = strcmp(string(T.sceneId), cfg.sceneIds(i)) & ...
                    T.difficultyIndex == d & ...
                    strcmp(string(T.planner), planners(j));
                Tp = T(mask, :);

                k = k + 1;
                rows(k).sceneId = cfg.sceneIds(i);
                rows(k).difficultyIndex = d;
                rows(k).planner = planners(j);
                rows(k).nSaved = height(Tp);
                if isempty(Tp)
                    continue;
                end

                rows(k).difficultyId = string(Tp.difficultyId(1));
                rows(k).envelopeDMinRate = ...
                    sum(Tp.envelopeDMinSatisfied) / height(Tp);
                rows(k).planTimeMeanSec = finiteMean(Tp.planTimeSec);

                if strcmpi(planners(j), "RRTSC-2D")
                    nAttempts = finiteSum(Tp.attempts);
                    nC1 = finiteSum(Tp.rrtscRRTSuccessCount);
                    nC2 = finiteSum(Tp.rrtscCenterlinePassCount);
                    nC3 = finiteSum(Tp.rrtscChordPassCount);
                    rows(k).rrtscC1Rate = safeRatio(nC1, nAttempts);
                    rows(k).rrtscC2GivenC1Rate = safeRatio(nC2, nC1);
                    rows(k).rrtscC3GivenC2Rate = safeRatio(nC3, nC2);
                end
            end
        end
    end

    summary = struct2table(rows(1:k));
end

function plotPartialBaselineFigures(T, summaryTable, cfg, expectedTotal)
    if ~isfolder(cfg.figDir)
        mkdir(cfg.figDir);
    end

    metricSpecs = {
        'envelopeClearMargin', 'Safety-clearance margin', ...
            'minClear - dMin', 'envelope_clearance_boxplot.jpg'
        'pathLength', 'Path length', ...
            'length', 'path_length_boxplot.jpg'
        'turnAbsSum', 'Path turning', ...
            'turn sum', 'turning_boxplot.jpg'
        'planTimeSec', 'Planning runtime', ...
            'time (s)', 'runtime_boxplot.jpg'
    };

    for i = 1:size(metricSpecs, 1)
        plotPartialMetricBoxplot(T, cfg, ...
            metricSpecs{i, 1}, metricSpecs{i, 2}, ...
            metricSpecs{i, 3}, metricSpecs{i, 4});
    end

    plotPartialMeanOverview(summaryTable, cfg, height(T), expectedTotal);
    plotPartialRRTSCStageRates(summaryTable, cfg);
end

function plotPartialMetricBoxplot( ...
        T, cfg, metricName, figTitle, yLabelText, fileName)
    sceneIds = cfg.sceneIds;
    difficultyIndices = 1:numel(cfg.difficultyNames);
    planners = string(cfg.plannerNames);
    fig = figure('Color', 'w', 'Name', figTitle, ...
        'Position', [60 40 1500 920]);
    tiledlayout(numel(difficultyIndices), numel(sceneIds), ...
        'Padding', 'compact', 'TileSpacing', 'compact');

    for d = difficultyIndices
        for i = 1:numel(sceneIds)
            nexttile((d - 1) * numel(sceneIds) + i);
            rows = T(strcmp(string(T.sceneId), sceneIds(i)) & ...
                T.difficultyIndex == d, :);
            y = rows.(metricName);
            valid = isfinite(y);
            groupIndex = nan(height(rows), 1);
            nSuccess = zeros(1, numel(planners));
            nSaved = zeros(1, numel(planners));

            for j = 1:numel(planners)
                methodMask = strcmp(string(rows.planner), planners(j));
                groupIndex(methodMask) = j;
                nSaved(j) = sum(methodMask);
                nSuccess(j) = sum( ...
                    methodMask & rows.envelopeDMinSatisfied);
            end

            if any(valid)
                present = unique(groupIndex(valid), 'stable');
                boxplot(y(valid), groupIndex(valid), ...
                    'Positions', present, 'Symbol', 'k.');
            else
                drawNoDataPlaceholder();
            end

            xlim([0.5, numel(planners) + 0.5]);
            set(gca, 'XTick', 1:numel(planners), ...
                'XTickLabel', cellstr(planners), ...
                'TickLabelInterpreter', 'none');
            if strcmp(metricName, 'envelopeClearMargin')
                yline(0, '--', 'Color', [0.75, 0.15, 0.15]);
            elseif strcmp(metricName, 'planTimeSec') && ...
                    any(valid) && all(y(valid) > 0)
                set(gca, 'YScale', 'log');
            end
            grid on;

            for j = 1:numel(planners)
                xNormalized = (j - 0.5) / numel(planners);
                text(xNormalized, 0.02, ...
                    sprintf('%d/%d', nSuccess(j), nSaved(j)), ...
                    'Units', 'normalized', ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'bottom', ...
                    'FontSize', 7, 'Color', [0.25, 0.25, 0.25], ...
                    'BackgroundColor', 'white', 'Margin', 0.5);
            end

            title(sprintf('%s | %s', ...
                char(sceneIds(i)), char(cfg.difficultyNames(d))), ...
                'Interpreter', 'none');
            if i == 1
                ylabel(yLabelText);
            end
        end
    end

    sgtitle([figTitle ' (partial saved cases)'], 'Interpreter', 'none');
    exportPartialFigure(fig, cfg, fileName);
end

function plotPartialMeanOverview(S, cfg, nSaved, expectedTotal)
    sceneIds = cfg.sceneIds;
    difficultyIndices = 1:numel(cfg.difficultyNames);
    planners = string(cfg.plannerNames);
    fig = figure('Color', 'w', 'Name', 'Mean baseline summary', ...
        'Position', [60 60 1500 720]);
    tiledlayout(2, numel(sceneIds), ...
        'Padding', 'compact', 'TileSpacing', 'compact');

    for i = 1:numel(sceneIds)
        successData = partialSummaryMatrix(S, sceneIds(i), ...
            difficultyIndices, planners, 'envelopeDMinRate');
        timeData = partialSummaryMatrix(S, sceneIds(i), ...
            difficultyIndices, planners, 'planTimeMeanSec');

        nexttile(i);
        colororder(cfg.plannerColors);
        bar(successData);
        if ~any(isfinite(successData), 'all')
            drawNoDataPlaceholder();
        end
        title(char(sceneIds(i)), 'Interpreter', 'none');
        ylabel('dMin satisfaction rate');
        ylim([0, 1]);
        set(gca, 'XTick', difficultyIndices, ...
            'XTickLabel', cellstr(cfg.difficultyNames), ...
            'TickLabelInterpreter', 'none');
        grid on;

        nexttile(numel(sceneIds) + i);
        colororder(cfg.plannerColors);
        bar(timeData);
        if any(isfinite(timeData) & timeData > 0, 'all')
            set(gca, 'YScale', 'log');
        else
            drawNoDataPlaceholder();
        end
        ylabel('mean planning time (s)');
        set(gca, 'XTick', difficultyIndices, ...
            'XTickLabel', cellstr(cfg.difficultyNames), ...
            'TickLabelInterpreter', 'none');
        grid on;
        if i == numel(sceneIds)
            legend(cellstr(planners), 'Location', 'southoutside', ...
                'Orientation', 'horizontal');
        end
    end

    if isfinite(expectedTotal)
        titleText = sprintf( ...
            'Partial mean results: %d saved files / %d expected cases', ...
            nSaved, expectedTotal);
    else
        titleText = sprintf( ...
            'Partial mean results: %d saved files', nSaved);
    end
    sgtitle(titleText);
    exportPartialFigure(fig, cfg, 'summary_mean_overview.jpg');
end

function plotPartialRRTSCStageRates(S, cfg)
    sceneIds = cfg.sceneIds;
    difficultyIndices = 1:numel(cfg.difficultyNames);
    fig = figure('Color', 'w', 'Name', ...
        'RRTSC-2D stage pass rates', ...
        'Position', [80 80 1400 420]);
    tiledlayout(1, numel(sceneIds), ...
        'Padding', 'compact', 'TileSpacing', 'compact');

    for i = 1:numel(sceneIds)
        stageRates = nan(numel(difficultyIndices), 3);
        for d = difficultyIndices
            row = S(strcmp(string(S.sceneId), sceneIds(i)) & ...
                S.difficultyIndex == d & ...
                strcmpi(string(S.planner), "RRTSC-2D"), :);
            if ~isempty(row)
                stageRates(d,:) = [
                    row.rrtscC1Rate(1), ...
                    row.rrtscC2GivenC1Rate(1), ...
                    row.rrtscC3GivenC2Rate(1)
                ];
            end
        end

        nexttile;
        bar(stageRates);
        if ~any(isfinite(stageRates), 'all')
            drawNoDataPlaceholder();
        end
        title(char(sceneIds(i)), 'Interpreter', 'none');
        ylabel('conditional pass rate');
        ylim([0, 1]);
        set(gca, 'XTick', difficultyIndices, ...
            'XTickLabel', cellstr(cfg.difficultyNames), ...
            'TickLabelInterpreter', 'none');
        grid on;
        if i == numel(sceneIds)
            legend({'Collision 1', 'Collision 2 | C1', ...
                'Collision 3 | C2'}, ...
                'Location', 'southoutside', ...
                'Orientation', 'horizontal');
        end
    end

    sgtitle('RRTSC-2D attempt-level stage pass rates (partial)');
    exportPartialFigure(fig, cfg, 'rrtsc_stage_pass_rates.jpg');
end

function data = partialSummaryMatrix( ...
        S, sceneId, difficultyIndices, planners, fieldName)
    data = nan(numel(difficultyIndices), numel(planners));
    for d = difficultyIndices
        for j = 1:numel(planners)
            row = S(strcmp(string(S.sceneId), sceneId) & ...
                S.difficultyIndex == d & ...
                strcmp(string(S.planner), planners(j)), :);
            if ~isempty(row)
                values = row.(fieldName);
                data(d,j) = values(1);
            end
        end
    end
end

function drawNoDataPlaceholder()
    text(0.5, 0.5, 'No data', ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', ...
        'Color', [0.55, 0.55, 0.55]);
end

function exportPartialFigure(fig, cfg, fileName)
    exportgraphics(fig, fullfile(cfg.figDir, fileName), ...
        'Resolution', cfg.figureResolution, ...
        'BackgroundColor', 'white');
    savefig(fig, fullfile(cfg.figDir, ...
        strrep(fileName, '.jpg', '.fig')));
    close(fig);
end

function value = finiteSum(x)
    x = x(isfinite(x));
    if isempty(x)
        value = 0;
    else
        value = sum(x);
    end
end

function value = safeRatio(numerator, denominator)
    if ~isfinite(denominator) || denominator <= 0
        value = nan;
    else
        value = numerator / denominator;
    end
end

function value = finiteMean(x)
    x = x(isfinite(x));
    if isempty(x)
        value = nan;
    else
        value = mean(x);
    end
end

function value = finiteMedian(x)
    x = x(isfinite(x));
    if isempty(x)
        value = nan;
    else
        value = median(x);
    end
end

function value = finiteMax(x)
    x = x(isfinite(x));
    if isempty(x)
        value = nan;
    else
        value = max(x);
    end
end
