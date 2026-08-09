clear; clc; close all;

%% FIG_BASELINE_COMPARE Publication figures for the formal simu1 run
% Reads the saved formal run
%   results/runs/run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_20260731_154814_evaluation_fixed
% (trial_results.csv + summary_by_scene_difficulty_planner.csv) and exports
% two publication figures:
%   fig_main_comparison_statistics.pdf/jpg
%       (a1)-(a4) whole-body dMin satisfaction rate by scene and difficulty
%       (b)      safety-clearance margin boxen plot, with n/N annotation
%       (c)      planning time boxen plot (log scale)
%       (d)      path length boxen plot
%       (e)      turning boxen plot
%   fig_efficiency_endtoend.pdf/jpg
%       (a) convergence of representative CSSC cases (objective J and
%           whole-body minimum clearance on dual axes)
%       (b) CSSC stage time breakdown (RRT frontend / init / optimization)
%
% Statistical conventions follow simu1_baseline_compare.m:
% success uses dMinSatisfied, corresponding to R^{WB}_{dMin}; boxplots
% only use trials with finite metrics; planning time excludes high-precision
% evaluation. Output is written to results/figures/baseline_compare/.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeDisplayConfig(projectRoot);
if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

fprintf('\n[fig_baseline_compare]\n');
fprintf('  run dir : %s\n', cfg.runDir);
fprintf('  out dir : %s\n', cfg.outDir);

[T, S] = loadRunTables(cfg);
stats = computeStats(T, S, cfg);

figMain = plotMainComparison(T, S, cfg);
exportPublicationFigure(figMain, cfg, 'fig_main_comparison_statistics');

figEff = plotEfficiency(T, S, cfg);
exportPublicationFigure(figEff, cfg, 'fig_efficiency_endtoend');

save(fullfile(cfg.outDir, 'baseline_compare_stats.mat'), ...
    'cfg', 'T', 'S', 'stats');
fprintf('\n[done] figures saved to %s\n', cfg.outDir);

%% Configuration

function cfg = makeDisplayConfig(projectRoot)
    % Shared geometry values and difficulty names:
    % demos/getCSSCDemoConfig2D.m
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ...
        'run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_20260731_154814_evaluation_fixed';
    cfg.runDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.caseDir = fullfile(cfg.runDir, 'cases');
    cfg.trialCsvPath = fullfile(cfg.runDir, 'trial_results.csv');
    cfg.summaryCsvPath = fullfile( ...
        cfg.runDir, 'summary_by_scene_difficulty_planner.csv');
    cfg.outDir = fullfile(projectRoot, 'results', 'figures', 'baseline_compare');

    cfg.figureResolution = 400;
    cfg.figureVisible = 'on';
    cfg.fontName = 'Times New Roman';
    cfg.fontSize = 12;
    cfg.mainFontSize = 8;
    cfg.mainFigureSizeCm = [18.0, 8.5];
    cfg.efficiencyFigureSizeCm = [18.0, 7.5];

    % Representative CSSC cases for the convergence analysis (loaded from
    % their saved result.mat optimizerInfo).
    cfg.convergenceCases = [ ...
        "case_0800_CSSC_double_slit_normal", ...
        "case_2030_CSSC_s_channel_normal"];
    cfg.convergenceLabels = [ ...
        "Double slit (Normal)", ...
        "S channel (Normal)"];
    cfg.convergenceAbbrev = ["DS-N", "SC-N"];

    cfg.plannerCsvNames = ["RRT", "RRT*", "RRTSC-2D", "Sp-RRT-2D", "CSSC"];
    cfg.plannerNames = ["RRT", "RRT*", "RRTSC", "Sp-RRT", "CSSC"];
    cfg.plannerColors = [
        0.20, 0.45, 0.75
        0.30, 0.65, 0.40
        0.85, 0.45, 0.18
        0.10, 0.62, 0.66
        0.85, 0.12, 0.14
    ];
    cfg.sceneIds = [ ...
        "double_slit", "s_channel", ...
        "staggered_baffles", "random_mixed"];
    cfg.sceneNames = [ ...
        "Double slit", "S channel", ...
        "Staggered baffles", "Random mixed"];
    cfg.difficultyNames = ["Easy", "Normal", "Hard"];
end

%% Data loading and statistics

function [T, S] = loadRunTables(cfg)
    if ~exist(cfg.trialCsvPath, 'file') || ...
            ~exist(cfg.summaryCsvPath, 'file')
        error('Run tables not found under %s', cfg.runDir);
    end
    T = readtable(cfg.trialCsvPath, 'TextType', 'string');
    S = readtable(cfg.summaryCsvPath, 'TextType', 'string');
    T.planner = strip(T.planner);
    S.planner = strip(S.planner);
end

function stats = computeStats(T, S, cfg)
    stats = struct();
    stats.nTrialsPerMethod = zeros(1, 5);
    stats.successRate = zeros(1, 5);
    stats.timeoutRate = zeros(1, 5);

    metricFields = {'envelopeClearMargin', 'pathLength', ...
        'turnAbsSum', 'planTimeSec'};
    for f = 1:numel(metricFields)
        fn = metricFields{f};
        med = nan(1, 5);
        q1 = nan(1, 5);
        q3 = nan(1, 5);
        meanVal = nan(1, 5);
        nValid = zeros(1, 5);
        for j = 1:5
            mask = T.planner == cfg.plannerCsvNames(j);
            stats.nTrialsPerMethod(j) = sum(mask);
            stats.successRate(j) = mean(T.dMinSatisfied(mask));
            stats.timeoutRate(j) = mean(T.timedOut(mask));
            v = T.(fn)(mask & isfinite(T.(fn)));
            nValid(j) = numel(v);
            if ~isempty(v)
                med(j) = median(v);
                q1(j) = prctile(v, 25);
                q3(j) = prctile(v, 75);
                meanVal(j) = mean(v);
            end
        end
        stats.(fn) = struct( ...
            'nValid', nValid, 'median', med, ...
            'q1', q1, 'q3', q3, 'mean', meanVal);
    end

    stats.successBySceneDiff = nan(4, 3, 5);
    for s = 1:4
        for d = 1:3
            for j = 1:5
                row = S(S.sceneId == cfg.sceneIds(s) & ...
                    S.difficultyIndex == d & ...
                    S.planner == cfg.plannerCsvNames(j), :);
                if ~isempty(row)
                    stats.successBySceneDiff(s, d, j) = ...
                        row.envelopeDMinRate(1);
                end
            end
        end
    end

    rows = S(S.planner == "CSSC", :);
    stats.csscStageMeans = [ ...
        rows.frontendTimeMeanSec, rows.initTimeMeanSec, ...
        rows.optTimeMeanSec, rows.planTimeMeanSec];
end

%% Figure 1: main comparison statistics

function fig = plotMainComparison(T, S, cfg)
    fig = figure( ...
        'Color', 'w', ...
        'Visible', cfg.figureVisible, ...
        'Units', 'centimeters', ...
        'Position', [2, 2, cfg.mainFigureSizeCm], ...
        'PaperUnits', 'centimeters', ...
        'PaperPosition', [0, 0, cfg.mainFigureSizeCm], ...
        'InvertHardcopy', 'off');

    layout = tiledlayout(fig, 2, 4, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    % ---- (a1)-(a4) whole-body dMin satisfaction rate by scene ----
    barHandles = gobjects(1, 5);
    for s = 1:4
        ax = nexttile(layout, s);
        mat = nan(3, 5);
        for d = 1:3
            for j = 1:5
                row = S(S.sceneId == cfg.sceneIds(s) & ...
                    S.difficultyIndex == d & ...
                    S.planner == cfg.plannerCsvNames(j), :);
                if ~isempty(row)
                    mat(d, j) = row.envelopeDMinRate(1);
                end
            end
        end
        bh = bar(ax, mat, 'BarLayout', 'grouped');
        for j = 1:5
            bh(j).FaceColor = cfg.plannerColors(j, :);
            barHandles(j) = bh(j);
        end
        set(ax, 'XTick', 1:3, 'XTickLabel', cfg.difficultyNames, ...
            'FontName', cfg.fontName, 'FontSize', cfg.mainFontSize);
        ylim(ax, [0, 1.08]);
        grid(ax, 'on');
        title(ax, sprintf('(a%d) %s', s, cfg.sceneNames(s)), ...
            'FontName', cfg.fontName, ...
            'FontSize', cfg.mainFontSize, ...
            'FontWeight', 'normal');
        if s == 1
            ylabel(ax, '$R^{\mathrm{WB}}_{d_{\min}}$', ...
                'Interpreter', 'latex');
        end
    end

    lgd = legend(ax, barHandles, cellstr(cfg.plannerNames), ...
        'Orientation', 'horizontal', ...
        'NumColumns', 5, ...
        'FontName', cfg.fontName, ...
        'FontSize', cfg.mainFontSize, ...
        'Box', 'off');
    lgd.Layout.Tile = 'south';
    % Anchor the legend at the bottom-right corner of the figure.
    drawnow;
    lgd.Units = 'normalized';
    lgd.Position(1) = 1 - lgd.Position(3) - 0.005;

    % ---- second row: (b) margin, (c) time, (d) length, (e) turning ----
    axb = nexttile(layout, 5);
    drawBoxenPanel(axb, T, cfg, ...
        'envelopeClearMargin', 'Safety margin', ... %(minClear - dMin)
        false, true, '(b)');

    axc = nexttile(layout, 6);
    drawBoxenPanel(axc, T, cfg, ...
        'planTimeSec', 'Planning time (s)', ...
        true, false, '(c)');

    axd = nexttile(layout, 7);
    drawBoxenPanel(axd, T, cfg, ...
        'pathLength', 'Path length', false, false, '(d)');

    axe = nexttile(layout, 8);
    drawBoxenPanel(axe, T, cfg, ...
        'turnAbsSum', 'Turning', false, false, '(e)'); % (turn sum)
end

function drawBoxenPanel(ax, T, cfg, fieldName, yLabelText, ...
        logScale, zeroLine, panelLabel)
    valid = isfinite(T.(fieldName));
    if logScale
        valid = valid & T.(fieldName) > 0;
    end
    y = T.(fieldName)(valid);
    hold(ax, 'on');
    for j = 1:5
        yj = sort(y(T.planner(valid) == cfg.plannerCsvNames(j)));
        if numel(yj) < 2
            continue;
        end
        drawLetterValueBox(ax, j, yj, cfg.plannerColors(j, :));
    end

    set(ax, 'XTick', 1:5, 'XTickLabel', cfg.plannerNames, ...
        'XTickLabelRotation', 45);
    if logScale
        set(ax, 'YScale', 'log');
    end
    if zeroLine
        yline(ax, 0, '--', 'Color', [0.75, 0.15, 0.15], 'LineWidth', 1.0);
    end
    grid(ax, 'on');
    set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.mainFontSize);
    ylabel(ax, yLabelText);
    title(ax, panelLabel, ...
        'FontName', cfg.fontName, ...
        'FontSize', cfg.mainFontSize, ...
        'FontWeight', 'normal');

end

function drawLetterValueBox(ax, xc, ys, color)
    n = numel(ys);
    maxLevel = min(5, max(1, floor(log2(n)) - 1));
    halfW = 0.30;

    % Nested letter-value boxes: outer levels lighter, inner levels darker.
    for k = maxLevel:-1:1
        p = 1 - 2^(-k);
        qLo = quantile(ys, 1 - p);
        qHi = quantile(ys, p);
        alpha = 0.40 - 0.30 * (k - 1) / max(1, maxLevel - 1);
        patch(ax, xc + [-halfW, halfW, halfW, -halfW], ...
            [qLo, qLo, qHi, qHi], color, ...
            'FaceAlpha', alpha, 'EdgeColor', 'none');
    end

    % Median line.
    qMed = quantile(ys, 0.5);
    plot(ax, xc + [-halfW, halfW], [qMed, qMed], '-', ...
        'Color', color, 'LineWidth', 1.6);

    % Thin whiskers from the outermost box to the data extremes.
    pOut = 1 - 2^(-maxLevel);
    qOutLo = quantile(ys, 1 - pOut);
    qOutHi = quantile(ys, pOut);
    plot(ax, [xc, xc], [ys(1), qOutLo], '-', ...
        'Color', color, 'LineWidth', 0.6);
    plot(ax, [xc, xc], [qOutHi, ys(end)], '-', ...
        'Color', color, 'LineWidth', 0.6);
end

%% Figure 2: efficiency

function fig = plotEfficiency(T, S, cfg)
    fig = figure( ...
        'Color', 'w', ...
        'Visible', cfg.figureVisible, ...
        'Units', 'centimeters', ...
        'Position', [2, 2, cfg.efficiencyFigureSizeCm], ...
        'PaperUnits', 'centimeters', ...
        'PaperPosition', [0, 0, cfg.efficiencyFigureSizeCm], ...
        'InvertHardcopy', 'off');

    layout = tiledlayout(fig, 1, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    ax1 = nexttile(layout, 1);
    drawConvergencePanel(ax1, cfg, '(a) Convergence');

    ax2 = nexttile(layout, 2);
    drawStageBreakdownPanel(ax2, S, cfg, ...
        '(b) CSSC stage time');
end

function data = loadConvergenceData(cfg)
    n = numel(cfg.convergenceCases);
    data = struct( ...
        'caseName', cell(1, n), ...
        'label', cell(1, n), ...
        'abbrev', cell(1, n), ...
        'Jhist', cell(1, n), ...
        'minClearHist', cell(1, n), ...
        'numIter', cell(1, n), ...
        'stopReason', cell(1, n), ...
        'returnedIter', cell(1, n));
    for i = 1:n
        caseName = char(cfg.convergenceCases(i));
        matPath = fullfile(cfg.caseDir, caseName, 'result.mat');
        if ~exist(matPath, 'file')
            error('Convergence case not found: %s', matPath);
        end
        S = load(matPath);
        info = S.result.optimizerInfo;
        data(i).caseName = caseName;
        data(i).label = char(cfg.convergenceLabels(i));
        data(i).abbrev = char(cfg.convergenceAbbrev(i));
        data(i).Jhist = info.Jhist(:);
        data(i).minClearHist = info.minClearHist(:);
        data(i).numIter = numel(data(i).Jhist);
        data(i).stopReason = char(info.stopReason);
        if isfield(info, 'returnedIter')
            data(i).returnedIter = double(info.returnedIter);
        else
            data(i).returnedIter = nan;
        end
    end
end

function drawConvergencePanel(ax, cfg, panelLabel)
    data = loadConvergenceData(cfg);
    blue = [0.05, 0.15, 0.55];   % left axis: objective J
    red = [0.65, 0.05, 0.05];    % right axis: whole-body clearance
    lineStyle = {'-', '--'};      % line style encodes the case

    yyaxis(ax, 'left');
    hold(ax, 'on');
    ax.YAxis(1).Color = blue;
    hJ = gobjects(1, numel(data));
    for i = 1:numel(data)
        J = data(i).Jhist;
        it = (1:numel(J)).';
        valid = isfinite(J) & J > 0;
        hJ(i) = plot(ax, it(valid), J(valid), lineStyle{i}, ...
            'Color', blue, 'LineWidth', 1.6);
    end
    ylabel(ax, 'Objective J');

    yyaxis(ax, 'right');
    hold(ax, 'on');
    ax.YAxis(2).Color = red;
    hC = gobjects(1, numel(data));
    for i = 1:numel(data)
        mc = data(i).minClearHist;
        it = (1:numel(mc)).';
        hC(i) = plot(ax, it, mc, lineStyle{i}, ...
            'Color', red, 'LineWidth', 1.6);
    end
    yline(ax, cfg.dMin, '-.', ...
        'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.3);
    ylabel(ax, '$c^{\mathrm{WB}}_{\min}$', 'Interpreter', 'latex');

    % Fit the shared x-axis to the actual data length.
    maxIter = max([data.numIter]);
    xlim(ax, [1, maxIter]);
    xlabel(ax, 'Iteration');
    grid(ax, 'on');
    set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize);
    title(ax, panelLabel, ...
        'FontName', cfg.fontName, ...
        'FontSize', cfg.fontSize, ...
        'FontWeight', 'normal');
    % Legend: at most two entries; color-neutral handles so color is read
    % from the axis and line style from the case.
    hDummy = gobjects(1, numel(data));
    for i = 1:numel(data)
        hDummy(i) = plot(ax, nan, nan, lineStyle{i}, ...
            'Color', [0.20, 0.20, 0.20], 'LineWidth', 1.6);
    end
    legend(ax, hDummy, convergenceLegendLabels(data), ...
        'Location', 'southeast', ...
        'FontSize', cfg.fontSize, ...
        'Box', 'off');
end

function labels = convergenceLegendLabels(data)
    labels = cell(1, numel(data));
    for i = 1:numel(data)
        labels{i} = sprintf('Case%d (%s)', ...
            i, data(i).abbrev);
    end
end

function drawStageBreakdownPanel(ax, S, cfg, panelLabel)
    rows = S(S.planner == "CSSC", :);
    if isempty(rows)
        error('No CSSC summary rows found.');
    end
    x = (1:height(rows)).';
    front = rows.frontendTimeMeanSec;
    init = rows.initTimeMeanSec;
    opt = rows.optTimeMeanSec;
    total = rows.planTimeMeanSec;

    bh = bar(ax, x, [front, init, opt], 'stacked');
    stageColors = [
        0.20, 0.45, 0.75
        0.30, 0.65, 0.40
        0.85, 0.45, 0.18
    ];
    for j = 1:3
        bh(j).FaceColor = stageColors(j, :);
    end
    hold(ax, 'on');
    plot(ax, x, total, 'ko-', 'LineWidth', 1.2, ...
        'MarkerFaceColor', 'k', 'MarkerSize', 5);

    set(ax, 'XTick', 1:numel(x), ...
        'XTickLabel', stageLabels(rows, cfg), ...
        'FontName', cfg.fontName, 'FontSize', cfg.fontSize);
    xtickangle(ax, 45);
    ylabel(ax, 'Mean time (s)');
    grid(ax, 'on');
    title(ax, panelLabel, ...
        'FontName', cfg.fontName, ...
        'FontSize', cfg.fontSize, ...
        'FontWeight', 'normal');
    legend(ax, {'RRT frontend', 'Shortcut+B-spline init', ...
        'CSSC optimization', 'Total planning'}, ...
        'Location', 'northwest', ...
        'FontSize', cfg.fontSize, ...
        'Box', 'off');
end

function labels = stageLabels(rows, cfg)
    n = height(rows);
    labels = strings(n, 1);
    sceneShort = containers.Map(cellstr(cfg.sceneIds), ...
        {'DS', 'SC', 'SB', 'RM'});
    for k = 1:n
        sid = char(rows.sceneId(k));
        if isKey(sceneShort, sid)
            ss = sceneShort(sid);
        else
            ss = sid;
        end
        dName = char(rows.difficultyName(k));
        labels(k) = sprintf('%s-%c', ss, dName(1));
    end
end

%% Export

function exportPublicationFigure(fig, cfg, baseName)
    pdfPath = fullfile(cfg.outDir, [baseName, '.pdf']);
    jpgPath = fullfile(cfg.outDir, [baseName, '.jpg']);
    figPath = fullfile(cfg.outDir, [baseName, '.fig']);
    exportgraphics(fig, pdfPath, ...
        'ContentType', 'vector', ...
        'BackgroundColor', 'white');
    exportgraphics(fig, jpgPath, ...
        'Resolution', cfg.figureResolution, ...
        'BackgroundColor', 'white');
    savefig(fig, figPath);
    close(fig);
    fprintf('  %s.pdf / .jpg / .fig\n', baseName);
end
