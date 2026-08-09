clear; clc; close all;

%% Saved Sp-RRT-2D, RRTSC-2D, and CSSC paths from one baseline run
% Loads matched per-case result.mat files. cfg.numPanels is an upper limit;
% smaller pilot runs automatically use all available complete trials.
% Failed methods remain visible whenever a degraded path was returned.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

%% Configuration
cfg.runName = 'run_pilot_s_channel_normal_20260731_120951';
cfg.sceneId = "s_channel";%"staggered_baffles";
cfg.difficultyId = "normal";
cfg.numPanels = 16; % Maximum number of panels; fewer saved trials are allowed.
cfg.trialIndices = []; % Set explicit trial indices here, or leave empty.

cfg.figureSizeCm = [24, 19];
cfg.resolutionPPI = 400;
cfg.visible = 'on';
cfg.fontName = 'Times New Roman';
cfg.fontSize = 8;

cfg.rrtscColor = [0.88, 0.35, 0.12];
cfg.sprrtColor = [0.58, 0.24, 0.68];
cfg.csscColor = [0.05, 0.55, 0.28];
cfg.rrtInitColor = [0.45, 0.48, 0.52];
cfg.bsplineInitColor = [0.18, 0.38, 0.76];
cfg.obstacleFaceColor = [0.82, 0.83, 0.84];
cfg.obstacleEdgeColor = [0.16, 0.17, 0.18];
cfg.dMinBoundaryColor = [0.92, 0.52, 0.10];
cfg.dMinBoundaryLineWidth = 0.75;
cfg.startColor = [0.08, 0.58, 0.55];
cfg.goalColor = [0.93, 0.64, 0.12];

runDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
caseDir = fullfile(runDir, 'cases');
csvPath = fullfile(runDir, 'trial_results.csv');
if ~isfile(csvPath) || ~isfolder(caseDir)
    error('Saved baseline run is incomplete: %s', runDir);
end

%% Find paired saved results
importOpts = detectImportOptions(csvPath, 'TextType', 'string');
trialTable = readtable(csvPath, importOpts);

baseMask = trialTable.sceneId == cfg.sceneId & ...
    trialTable.difficultyId == cfg.difficultyId;
rrtscRows = trialTable(baseMask & trialTable.planner == "RRTSC-2D", :);
sprrtRows = trialTable(baseMask & trialTable.planner == "Sp-RRT-2D", :);
csscRows = trialTable(baseMask & trialTable.planner == "CSSC", :);

if isempty(cfg.trialIndices)
    candidateTrials = unique(trialTable.trialIndex(baseMask), 'stable');
else
    candidateTrials = cfg.trialIndices(:);
end
candidateTrials = unique(candidateTrials, 'stable');

targetPanels = min(cfg.numPanels, numel(candidateTrials));
if targetPanels < 1
    error('No saved trials match scene=%s and difficulty=%s.', ...
        cfg.sceneId, cfg.difficultyId);
end
panels = repmat(emptyPanel(), targetPanels, 1);
nPanel = 0;
for i = 1:numel(candidateTrials)
    trialIndex = candidateTrials(i);
    rrRow = rrtscRows(rrtscRows.trialIndex == trialIndex, :);
    spRow = sprrtRows(sprrtRows.trialIndex == trialIndex, :);
    csRow = csscRows(csscRows.trialIndex == trialIndex, :);
    if height(rrRow) ~= 1 || height(spRow) ~= 1 || height(csRow) ~= 1
        continue;
    end

    rrResult = loadCaseResult(caseDir, rrRow.caseId);
    spResult = loadCaseResult(caseDir, spRow.caseId);
    csResult = loadCaseResult(caseDir, csRow.caseId);
    if isempty(rrResult) || isempty(spResult) || isempty(csResult)
        continue;
    end

    pathRRTSC = extractSavedPath(rrResult);
    pathSpRRT = extractSavedPath(spResult);
    pathCSSC = extractSavedPath(csResult);
    pathCSSCRRT = getNestedField(csResult, {'paths','pathRRT'}, []);
    pathCSSCInit = extractControlPointPath(csResult, 'Pinit');

    nPanel = nPanel + 1;
    panels(nPanel).trialIndex = trialIndex;
    panels(nPanel).envSeed = rrRow.envSeed;
    panels(nPanel).obstacles = rrResult.obstacles;
    panels(nPanel).bounds = rrResult.config.bounds;
    panels(nPanel).startPt = rrResult.config.startPt;
    panels(nPanel).goalPt = rrResult.config.goalPt;
    panels(nPanel).dMin = getNestedField( ...
        rrResult, {'config','dMin'}, 0);
    panels(nPanel).pathRRTSC = pathRRTSC;
    panels(nPanel).pathSpRRT = pathSpRRT;
    panels(nPanel).pathCSSC = pathCSSC;
    panels(nPanel).pathCSSCRRT = pathCSSCRRT;
    panels(nPanel).pathCSSCInit = pathCSSCInit;
    panels(nPanel).clearRRTSC = rrRow.envelopeMinClear;
    panels(nPanel).clearSpRRT = spRow.envelopeMinClear;
    panels(nPanel).clearCSSC = csRow.envelopeMinClear;
    panels(nPanel).successRRTSC = logical(rrRow.wholeBodySuccess);
    panels(nPanel).successSpRRT = logical(spRow.wholeBodySuccess);
    panels(nPanel).successCSSC = logical(csRow.wholeBodySuccess);

    if nPanel >= targetPanels
        break;
    end
end

if nPanel < 1
    error('No complete matched Sp-RRT/RRTSC/CSSC trials were found.');
end
panels = panels(1:nPanel);

%% Draw and export
fig = figure( ...
    'Color', 'w', ...
    'Visible', cfg.visible, ...
    'Units', 'centimeters', ...
    'Position', [2, 2, cfg.figureSizeCm], ...
    'PaperUnits', 'centimeters', ...
    'PaperPosition', [0, 0, cfg.figureSizeCm]);
nCols = min(4, ceil(sqrt(nPanel)));
nRows = ceil(nPanel / nCols);
layout = tiledlayout(fig, nRows, nCols, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
legendHandles = gobjects(1, 6);

for i = 1:numel(panels)
    ax = nexttile(layout, i);
    [hRRT, hInit, hSpRRT, hRRTSC, hCSSC, hDMin] = ...
        drawPanel(ax, panels(i), cfg);
    if i == 1
        legendHandles = [hRRT, hInit, hSpRRT, hRRTSC, hCSSC, hDMin];
    end
    title(ax, sprintf('Trial %d, seed %d', ...
        panels(i).trialIndex, panels(i).envSeed), ...
        'FontName', cfg.fontName, 'FontSize', cfg.fontSize, ...
        'FontWeight', 'normal');
    drawResultLabel(ax, panels(i), cfg);
end

figureTitle = sprintf('%s | %s: Sp-RRT-2D, RRTSC-2D, and CSSC', ...
    strrep(char(cfg.sceneId), '_', ' '), char(cfg.difficultyId));
sgtitle(layout, figureTitle, ...
    'FontName', cfg.fontName, 'FontWeight', 'normal');
lgd = legend(legendHandles, ...
    {'CSSC RRT initial', 'CSSC B-spline initial', ...
    'Sp-RRT-2D', 'RRTSC-2D', 'CSSC optimized', 'dMin boundary'}, ...
    'Orientation', 'horizontal', ...
    'FontName', cfg.fontName, 'FontSize', cfg.fontSize);
lgd.Layout.Tile = 'south';

outTag = regexprep(sprintf('%s_%s', ...
    char(cfg.sceneId), char(cfg.difficultyId)), '[^A-Za-z0-9_-]', '_');
outBase = fullfile(runDir, 'figures', ...
    sprintf('%s_sprrt_rrtsc_cssc_paths', outTag));
exportgraphics(fig, [outBase '.jpg'], ...
    'Resolution', cfg.resolutionPPI, 'BackgroundColor', 'white');
exportgraphics(fig, [outBase '.pdf'], ...
    'ContentType', 'vector', 'BackgroundColor', 'white');
savefig(fig, [outBase '.fig']);
save([outBase '_data.mat'], 'cfg', 'panels');

fprintf('[saved-result comparison]\n');
fprintf('  selected trials: %s\n', mat2str([panels.trialIndex]));
fprintf('  JPG: %s.jpg\n', outBase);
fprintf('  PDF: %s.pdf\n', outBase);

%% Local helpers
function panel = emptyPanel()
    panel = struct();
    panel.trialIndex = nan;
    panel.envSeed = nan;
    panel.obstacles = [];
    panel.bounds = nan(2,2);
    panel.startPt = [nan, nan];
    panel.goalPt = [nan, nan];
    panel.dMin = 0;
    panel.pathRRTSC = zeros(0,2);
    panel.pathSpRRT = zeros(0,2);
    panel.pathCSSC = zeros(0,2);
    panel.pathCSSCRRT = zeros(0,2);
    panel.pathCSSCInit = zeros(0,2);
    panel.clearRRTSC = nan;
    panel.clearSpRRT = nan;
    panel.clearCSSC = nan;
    panel.successRRTSC = false;
    panel.successSpRRT = false;
    panel.successCSSC = false;
end

function result = loadCaseResult(caseDir, caseId)
    result = [];
    pattern = fullfile(caseDir, sprintf('case_%04d_*', caseId), ...
        'result.mat');
    files = dir(pattern);
    if numel(files) ~= 1
        return;
    end
    data = load(fullfile(files(1).folder, files(1).name), 'result');
    if isfield(data, 'result')
        result = data.result;
    end
end

function path = extractSavedPath(result)
    path = zeros(0,2);
    candidates = {
        getNestedField(result, {'paths','pathOptimized'}, [])
        getNestedField(result, {'highPrecisionMetrics','pathSample'}, [])
    };

    for i = 1:numel(candidates)
        value = candidates{i};
        if isnumeric(value) && size(value,2) == 2 && ...
                size(value,1) >= 2 && all(isfinite(value(:)))
            path = value;
            return;
        end
    end

    path = extractControlPointPath(result, 'Popt');
end

function path = extractControlPointPath(result, controlPointField)
    path = zeros(0,2);
    P = getNestedField(result, {'paths',controlPointField}, []);
    params = getNestedField(result, ...
        {'highPrecisionMetrics','paramsUsed'}, struct());
    if isnumeric(P) && size(P,2) == 2 && size(P,1) >= 4
        degree = getFieldOr(params, 'degree', 3);
        knot = getFieldOr(params, 'knot', ...
            makeClampedUniformKnot(size(P,1), degree));
        u = linspace(0, 1, 900).';
        path = evalBSplinePath2D(P, u, degree, knot);
        if any(~isfinite(path(:)))
            path = zeros(0,2);
        end
    end
end

function [hRRT, hInit, hSpRRT, hRRTSC, hCSSC, hDMin] = ...
        drawPanel(ax, panel, cfg)
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    drawObstacles(ax, panel.obstacles, cfg);
    hDMin = drawDMinBoundaries(ax, panel.obstacles, panel.dMin, cfg);
    drawBoundary(ax, panel.bounds);

    hRRT = plot(ax, panel.pathCSSCRRT(:,1), panel.pathCSSCRRT(:,2), ':', ...
        'Color', cfg.rrtInitColor, 'LineWidth', 0.85);
    hInit = plot(ax, panel.pathCSSCInit(:,1), panel.pathCSSCInit(:,2), '--', ...
        'Color', cfg.bsplineInitColor, 'LineWidth', 1.05);
    hSpRRT = plot(ax, panel.pathSpRRT(:,1), panel.pathSpRRT(:,2), '-o', ...
        'Color', cfg.sprrtColor, 'LineWidth', 1.05, ...
        'MarkerSize', 2.2, 'MarkerFaceColor', cfg.sprrtColor);
    hRRTSC = plot(ax, panel.pathRRTSC(:,1), panel.pathRRTSC(:,2), '-.', ...
        'Color', cfg.rrtscColor, 'LineWidth', 1.25);
    hCSSC = plot(ax, panel.pathCSSC(:,1), panel.pathCSSC(:,2), '-', ...
        'Color', cfg.csscColor, 'LineWidth', 1.45);
    drawStartGoal(ax, panel.startPt, panel.goalPt, cfg);

    spanX = diff(panel.bounds(1,:));
    spanY = diff(panel.bounds(2,:));
    xlim(ax, panel.bounds(1,:) + [-1, 1] * 0.012 * spanX);
    ylim(ax, panel.bounds(2,:) + [-1, 1] * 0.012 * spanY);
end

function hLegend = drawDMinBoundaries(ax, obstacles, dMin, cfg)
    hLegend = plot(ax, nan, nan, '--', ...
        'Color', cfg.dMinBoundaryColor, ...
        'LineWidth', cfg.dMinBoundaryLineWidth);
    if ~isfinite(dMin) || dMin <= 0
        return;
    end

    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 140).';
                vertices = obs.center + (obs.radius + dMin) * ...
                    [cos(theta), sin(theta)];

            case {'rect','rectangle','box'}
                vertices = rectangleVertices( ...
                    obs.center, obs.halfSize, obs.yaw);
                vertices = offsetOutline(vertices, dMin);

            case 'polygon'
                vertices = offsetOutline(obs.vertices, dMin);

            otherwise
                continue;
        end

        plot(ax, [vertices(:,1); vertices(1,1)], ...
            [vertices(:,2); vertices(1,2)], '--', ...
            'Color', cfg.dMinBoundaryColor, ...
            'LineWidth', cfg.dMinBoundaryLineWidth, ...
            'HandleVisibility', 'off');
    end
end

function vertices = rectangleVertices(center, halfSize, yaw)
    local = [
        -halfSize(1), -halfSize(2)
         halfSize(1), -halfSize(2)
         halfSize(1),  halfSize(2)
        -halfSize(1),  halfSize(2)
    ];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    vertices = (R * local.').' + center;
end

function offsetVertices = offsetOutline(vertices, distance)
    if size(vertices, 1) > 1 && ...
            norm(vertices(1,:) - vertices(end,:)) < 1e-12
        vertices = vertices(1:end-1,:);
    end
    next = vertices([2:end, 1], :);
    signedArea = 0.5 * sum(vertices(:,1) .* next(:,2) - ...
        next(:,1) .* vertices(:,2));
    if signedArea < 0
        vertices = flipud(vertices);
    end

    n = size(vertices, 1);
    offsetVertices = zeros(n, 2);
    for i = 1:n
        iPrev = mod(i - 2, n) + 1;
        iNext = mod(i, n) + 1;
        edgePrev = vertices(i,:) - vertices(iPrev,:);
        edgeNext = vertices(iNext,:) - vertices(i,:);
        edgePrev = edgePrev / max(norm(edgePrev), eps);
        edgeNext = edgeNext / max(norm(edgeNext), eps);
        normalPrev = [edgePrev(2), -edgePrev(1)];
        normalNext = [edgeNext(2), -edgeNext(1)];
        pointPrev = vertices(i,:) + distance * normalPrev;
        pointNext = vertices(i,:) + distance * normalNext;
        A = [edgePrev(:), -edgeNext(:)];

        if abs(det(A)) < 1e-10
            normal = normalPrev + normalNext;
            normal = normal / max(norm(normal), eps);
            offsetVertices(i,:) = vertices(i,:) + distance * normal;
        else
            parameter = A \ (pointNext - pointPrev).';
            offsetVertices(i,:) = pointPrev + parameter(1) * edgePrev;
        end
    end
end

function drawResultLabel(ax, panel, cfg)
    label = sprintf('Sp %s | RRTSC %s | CSSC %s', ...
        formatMethodResult(panel.clearSpRRT, panel.successSpRRT), ...
        formatMethodResult(panel.clearRRTSC, panel.successRRTSC), ...
        formatMethodResult(panel.clearCSSC, panel.successCSSC));
    text(ax, 0.5, -0.035, label, ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'top', ...
        'Clipping', 'off', ...
        'FontName', cfg.fontName, ...
        'FontSize', max(6, cfg.fontSize - 1), ...
        'Color', [0.12, 0.12, 0.12]);
end

function textOut = formatMethodResult(minClear, success)
    if isfinite(minClear)
        clearText = sprintf('%.3f', minClear);
    else
        clearText = '--';
    end

    if success
        statusText = 'OK';
    else
        statusText = 'FAIL';
    end
    textOut = sprintf('%s %s', clearText, statusText);
end

function drawObstacles(ax, obstacles, cfg)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 100);
                x = obs.center(1) + obs.radius * cos(theta);
                y = obs.center(2) + obs.radius * sin(theta);

            case {'rect','rectangle','box'}
                local = [
                    -obs.halfSize(1), -obs.halfSize(2)
                     obs.halfSize(1), -obs.halfSize(2)
                     obs.halfSize(1),  obs.halfSize(2)
                    -obs.halfSize(1),  obs.halfSize(2)
                ];
                R = [cos(obs.yaw), -sin(obs.yaw); ...
                    sin(obs.yaw), cos(obs.yaw)];
                world = (R * local.').' + obs.center;
                x = world(:,1);
                y = world(:,2);

            case 'polygon'
                x = obs.vertices(:,1);
                y = obs.vertices(:,2);

            otherwise
                continue;
        end

        patch(ax, x, y, cfg.obstacleFaceColor, ...
            'FaceAlpha', 0.92, ...
            'EdgeColor', cfg.obstacleEdgeColor, ...
            'LineWidth', 0.75);
    end
end

function drawBoundary(ax, bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot(ax, [x(1), x(2), x(2), x(1), x(1)], ...
        [y(1), y(1), y(2), y(2), y(1)], ...
        'k-', 'LineWidth', 1.0, 'Clipping', 'off');
end

function drawStartGoal(ax, startPt, goalPt, cfg)
    plot(ax, startPt(1), startPt(2), 'o', ...
        'MarkerSize', 4.5, ...
        'MarkerFaceColor', cfg.startColor, ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'LineWidth', 0.7);
    plot(ax, goalPt(1), goalPt(2), 's', ...
        'MarkerSize', 4.5, ...
        'MarkerFaceColor', cfg.goalColor, ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'LineWidth', 0.7);
end

function value = getNestedField(s, names, defaultValue)
    value = s;
    for i = 1:numel(names)
        if ~isstruct(value) || ~isfield(value, names{i})
            value = defaultValue;
            return;
        end
        value = value.(names{i});
    end
end

function value = getFieldOr(s, name, defaultValue)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = defaultValue;
    end
end
