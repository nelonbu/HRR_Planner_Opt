clear; clc; close all;

%% FIG_QUALITATIVE_RESULT (data from a saved formal simu1 run)
% Exports the 2-by-2 qualitative figure directly from the saved baseline
% run produced by demos/simu1_baseline_compare.m:
%   results/runs/run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_20260731_154814_evaluation_fixed
%
% No RRT, shortcut, B-spline initialization or CSSC optimization is rerun.
% For each of the four scene families, one case folder is loaded directly
% from cfg.qualCaseNames (randomly chosen trial numbers at the Normal
% difficulty), without any seed selection or scoring. The panel drawing
% logic is identical to the previous script, archived under demos/legacy,
% except that the raw frontend polyline is no longer drawn.

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

fprintf('\n[qualitative result figure (from saved run)]\n');
fprintf('  run dir  : %s\n', cfg.runDir);
fprintf('  case dir : %s\n', cfg.caseDir);

results = loadSpecifiedCases(cfg);

for i = 1:numel(results)
    fprintf('\n[%d/%d] %s\n', i, numel(results), cfg.qualSceneNames(i));
    fprintf('  case                  : %s\n', results{i}.caseName);
    fprintf('  env seed              : %d\n', results{i}.envSeed);
    fprintf('  difficulty            : %s\n', results{i}.difficultyId);
    fprintf('  centerline clearance  : %.5f\n', ...
        results{i}.initMetrics.pointMinClear);
    fprintf('  initial chord clear   : %.5f\n', ...
        results{i}.initMetrics.minClear);
    fprintf('  CSSC chord clear      : %.5f\n', ...
        results{i}.optMetrics.minClear);
end

jpgPath = fullfile(cfg.outDir, 'fig_qualitative_result.jpg');
pdfPath = fullfile(cfg.outDir, 'fig_qualitative_result.pdf');
matPath = fullfile(cfg.outDir, 'fig_qualitative_result_data.mat');
exportQualitativeFigure(results, cfg, jpgPath, pdfPath);
save(matPath, 'cfg', 'results');

fprintf('\n[done]\n');
fprintf('  JPG : %s\n', jpgPath);
fprintf('  PDF : %s\n', pdfPath);
fprintf('  data: %s\n', matPath);

%% Configuration

function cfg = makeDisplayConfig(projectRoot)
    % Shared geometry values (dMin, L, bounds, difficulty names):
    % demos/getCSSCDemoConfig2D.m
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ...
        'run_baseline_rrt_rrtstar_rrtsc_sprrt_cssc_20260731_154814_evaluation_fixed';
    cfg.runDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.caseDir = fullfile(cfg.runDir, 'cases');
    cfg.outDir = fullfile(projectRoot, 'results', 'figures', 'qualitative_result');

    % Keep zero for the current geometry-only project definition. Set a
    % positive value only when explicitly modelling finite body thickness.
    cfg.bodyRadius = 0.0;
    cfg.safetyInflation = cfg.bodyRadius + cfg.dMin;

    cfg.figureWidthCm = 14.0;
    cfg.figureHeightCm = 12;
    cfg.resolutionPPI = 400;
    cfg.figureVisible = 'on';
    cfg.fontName = 'Times New Roman';

    cfg.startColor = [0.08, 0.58, 0.55];
    cfg.goalColor = [0.93, 0.64, 0.12];
    cfg.initPathColor = [0.18, 0.38, 0.76];
    cfg.optPathColor = [0.05, 0.55, 0.28];
    cfg.initMarkerColor = [0.12, 0.28, 0.78];
    cfg.optMarkerColor = [0.72, 0.16, 0.62];
    cfg.safetyBoundaryColor = [0.92, 0.52, 0.10];
    cfg.chordColor = [0.30, 0.30, 0.30];
    cfg.collisionColor = [0.92, 0.08, 0.06];

    cfg.initPathLineWidth = 1.35;
    cfg.optPathLineWidth = 1.70;
    cfg.chordLineWidth = 0.72;
    cfg.chordAlpha = 0.3;
    % Draw one fixed-length chord every sample to keep the swept lines
    % dense (evaluation nU = 240).
    cfg.chordStride = 1;
    % Draw the swept-envelope patch between every consecutive valid chord.
    cfg.sweepStride = 1;
    cfg.safeSweepAlpha = 0.2;
    cfg.collisionChordAlpha = 0.55;

    % Cases to load directly from the saved run. Trial numbers are random
    % picks at the Normal difficulty and can be edited freely.
    cfg.qualSceneIds = [ ...
        "double_slit", "s_channel", ...
        "staggered_baffles", "random_mixed"];
    cfg.qualSceneNames = [ ...
        "Double slit", "S channel", ...
        "Staggered baffles", "Random mixed obstacles"];
    cfg.qualCaseNames = [ ...
        "case_0685_CSSC_double_slit_normal", ...
        "case_2060_CSSC_s_channel_normal", ...
        "case_3940_CSSC_staggered_baffles_normal", ...
        "case_5265_CSSC_random_mixed_normal"];
end

%% Direct case loading from the saved run

function results = loadSpecifiedCases(cfg)
    results = cell(numel(cfg.qualCaseNames), 1);
    for i = 1:numel(cfg.qualCaseNames)
        caseName = char(cfg.qualCaseNames(i));
        matPath = fullfile(cfg.caseDir, caseName, 'result.mat');
        if ~exist(matPath, 'file')
            error('Specified case does not exist: %s', matPath);
        end
        S = load(matPath);
        results{i} = buildDisplayResult(S.result, cfg, caseName);
    end
end

function [sceneId, difficultyId] = parseCaseName(name, cfg)
    sceneId = '';
    difficultyId = '';
    parts = strsplit(name, '_');
    if numel(parts) < 6
        return;
    end
    if ~strcmp(parts{3}, 'CSSC')
        return;
    end
    difficultyId = parts{end};
    sceneId = strjoin(parts(4:end-1), '_');
end

function r = buildDisplayResult(res, cfg, caseName)
    initM = res.stageHighPrecisionMetrics.initial;
    finalM = res.stageHighPrecisionMetrics.final;
    [sceneId, difficultyId] = parseCaseName(caseName, cfg);

    r = struct();
    r.caseName = caseName;
    r.sceneId = sceneId;
    r.difficultyId = difficultyId;
    r.envSeed = res.seed.envSeed;
    r.obstacles = res.obstacles;
    r.envInfo = struct();
    r.envInfo.bounds = res.config.bounds;
    r.envInfo.startPt = res.config.startPt(:).';
    r.envInfo.goalPt = res.config.goalPt(:).';
    r.initState = initM.state;
    r.optState = finalM.state;
    r.initMetrics = initM;
    r.optMetrics = finalM;
end

%% Figure export

function exportQualitativeFigure(results, cfg, jpgPath, pdfPath)
    fig = figure( ...
        'Color', 'w', ...
        'Visible', cfg.figureVisible, ...
        'Units', 'centimeters', ...
        'Position', [2, 2, cfg.figureWidthCm, cfg.figureHeightCm], ...
        'PaperUnits', 'centimeters', ...
        'PaperPosition', [0, 0, cfg.figureWidthCm, cfg.figureHeightCm], ...
        'InvertHardcopy', 'off');

    layout = tiledlayout(fig, 2, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');
    panelLabels = {'(a) Double slit', '(b) S channel', ...
        '(c) Staggered baffles', '(d) Random mixed obstacles'};
    axesList = gobjects(4, 1);

    for i = 1:4
        axesList(i) = nexttile(layout, i);
        drawScenePanel(axesList(i), results{i}, cfg);
        text(axesList(i), 0.5, -0.035, panelLabels{i}, ...
            'Units', 'normalized', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', ...
            'FontName', cfg.fontName, ...
            'FontSize', 11.5, ...
            'FontWeight', 'normal', ...
            'Clipping', 'off');
    end

    [handles, labels] = createSharedLegendHandles(axesList(1), cfg);
    lgd = legend(axesList(1), handles, labels, ...
        'NumColumns', 2, ...
        'Location', 'southoutside', ...
        'Interpreter', 'none', ...
        'FontName', cfg.fontName, ...
        'FontSize', 10.0, ...
        'Box', 'off');
    lgd.Layout.Tile = 'south';

    exportgraphics(fig, jpgPath, ...
        'Resolution', cfg.resolutionPPI, ...
        'BackgroundColor', 'white');
    exportgraphics(fig, pdfPath, ...
        'ContentType', 'vector', ...
        'BackgroundColor', 'white');
end

function drawScenePanel(ax, result, cfg)
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    set(ax, 'FontName', cfg.fontName, 'FontSize', 10.0);

    bounds = result.envInfo.bounds;
    spanX = diff(bounds(1,:));
    spanY = diff(bounds(2,:));
    xlim(ax, bounds(1,:) + [-1, 1] * 0.012 * spanX);
    ylim(ax, bounds(2,:) + [-1, 1] * 0.012 * spanY);

    drawObstacles(ax, result.obstacles);
    drawSafetyBoundaries(ax, result.obstacles, cfg.safetyInflation, cfg);
    drawSweptChordPatches(ax, result.initState, cfg);
    drawSparseChords(ax, result.optState, cfg);
    drawBoundary(ax, bounds);

    plot(ax, result.initState.pathSample(:,1), result.initState.pathSample(:,2), ...
        '--', 'Color', cfg.initPathColor, 'LineWidth', cfg.initPathLineWidth);
    plot(ax, result.optState.pathSample(:,1), result.optState.pathSample(:,2), ...
        '-', 'Color', cfg.optPathColor, 'LineWidth', cfg.optPathLineWidth);

    drawClearanceMarkers(ax, result, cfg);
    drawStartGoal(ax, result.envInfo.startPt, result.envInfo.goalPt, cfg);
end

function drawBoundary(ax, bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot(ax, [x(1), x(2), x(2), x(1), x(1)], ...
        [y(1), y(1), y(2), y(2), y(1)], ...
        'k-', 'LineWidth', 1.15, 'Clipping', 'off');
end

function drawObstacles(ax, obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 100);
                vertices = obs.center + obs.radius * [cos(theta(:)), sin(theta(:))];
            case {'rect', 'rectangle', 'box'}
                vertices = rectangleVertices(obs.center, obs.halfSize, obs.yaw);
            otherwise
                continue;
        end

        patch(ax, vertices(:,1), vertices(:,2), [0.76, 0.78, 0.80], ...
            'FaceAlpha', 0.90, ...
            'EdgeColor', [0.16, 0.17, 0.18], ...
            'LineWidth', 0.75);
    end
end

function drawSafetyBoundaries(ax, obstacles, inflation, cfg)
    if inflation <= 0
        return;
    end

    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 120).';
                vertices = obs.center + (obs.radius + inflation) * ...
                    [cos(theta), sin(theta)];
            case {'rect', 'rectangle', 'box'}
                vertices = rectangleVertices(obs.center, obs.halfSize, obs.yaw);
                vertices = offsetRectangleOutline(vertices, inflation);
            otherwise
                continue;
        end

        plot(ax, [vertices(:,1); vertices(1,1)], ...
            [vertices(:,2); vertices(1,2)], '--', ...
            'Color', cfg.safetyBoundaryColor, ...
            'LineWidth', 0.72);
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

function offsetVertices = offsetRectangleOutline(vertices, distance)
    vertices = removeRepeatedEndVertex(vertices);
    if signedOutlineArea(vertices) < 0
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

function vertices = removeRepeatedEndVertex(vertices)
    if size(vertices, 1) > 1 && norm(vertices(1,:) - vertices(end,:)) < 1e-12
        vertices = vertices(1:end-1,:);
    end
end

function area = signedOutlineArea(vertices)
    next = vertices([2:end, 1], :);
    area = 0.5 * sum(vertices(:,1) .* next(:,2) - next(:,1) .* vertices(:,2));
end

function drawSweptChordPatches(ax, state, cfg)
    [valid, clearance] = chordStateData(state);
    validIdx = find(valid);
    if numel(validIdx) < 2
        return;
    end

    selected = validIdx(1:cfg.sweepStride:end);
    if selected(end) ~= validIdx(end)
        selected(end+1) = validIdx(end); %#ok<AGROW>
    end

    for k = 1:numel(selected)-1
        i = selected(k);
        j = selected(k+1);
        if any(~valid(i:j))
            continue;
        end

        vertices = [state.M(i,:); state.N(i,:); state.N(j,:); state.M(j,:)];
        colliding = any(isfinite(clearance(i:j)) & clearance(i:j) < cfg.dMin);
        if ~colliding
            patch(ax, vertices(:,1), vertices(:,2), [0.42, 0.42, 0.42], ...
                'FaceAlpha', cfg.safeSweepAlpha, ...
                'EdgeColor', 'none');
        end
    end

    collisionMask = valid & isfinite(clearance) & clearance < cfg.dMin;
    drawChordMask(ax, state.M, state.N, collisionMask, ...
        cfg.collisionColor, cfg.collisionChordAlpha, ...
        1.6 * cfg.chordLineWidth);
end

function drawSparseChords(ax, state, cfg)
    [valid, clearance] = chordStateData(state);
    idx = find(valid);
    idx = idx(1:cfg.chordStride:end);

    for k = 1:numel(idx)
        i = idx(k);
        if isfinite(clearance(i)) && clearance(i) < cfg.dMin
            color = cfg.collisionColor;
            lineWidth = 1.6 * cfg.chordLineWidth;
        else
            color = cfg.chordColor;
            lineWidth = cfg.chordLineWidth;
        end

        patch(ax, ...
            'XData', [state.M(i,1), state.N(i,1)], ...
            'YData', [state.M(i,2), state.N(i,2)], ...
            'FaceColor', 'none', ...
            'EdgeColor', color, ...
            'EdgeAlpha', cfg.chordAlpha, ...
            'LineWidth', lineWidth);
    end
end

function drawChordMask(ax, M, N, mask, color, alphaValue, lineWidth)
    idx = find(mask);
    for k = 1:numel(idx)
        i = idx(k);
        patch(ax, ...
            'XData', [M(i,1), N(i,1)], ...
            'YData', [M(i,2), N(i,2)], ...
            'FaceColor', 'none', ...
            'EdgeColor', color, ...
            'EdgeAlpha', alphaValue, ...
            'LineWidth', lineWidth);
    end
end

function [valid, clearance] = chordStateData(state)
    valid = state.validLine(:) & ...
        all(isfinite(state.M), 2) & all(isfinite(state.N), 2);

    clearance = nan(size(valid));
    if isfield(state, 'clearance') && numel(state.clearance) == numel(valid)
        clearance = state.clearance(:);
    elseif isfield(state, 'clearanceSegment') && ...
            numel(state.clearanceSegment) == numel(valid)
        clearance = state.clearanceSegment(:);
    end
end

function drawClearanceMarkers(ax, result, cfg)
    % Initial minimum is taken from the swept fixed-chord envelope (the
    % same quantity as the optimized marker), not the centerline SDF.
    pointMin = result.initState.minPoint;
    if all(isfinite(pointMin))
        plot(ax, pointMin(1), pointMin(2), 'x', ...
            'Color', cfg.initMarkerColor, ...
            'MarkerSize', 7.5, ...
            'LineWidth', 1.45);
    end

    optMin = result.optState.minPoint;
    if all(isfinite(optMin))
        plot(ax, optMin(1), optMin(2), 'p', ...
            'MarkerSize', 8.5, ...
            'MarkerFaceColor', cfg.optMarkerColor, ...
            'MarkerEdgeColor', [0.10, 0.10, 0.10], ...
            'LineWidth', 0.75);
    end
end

function drawStartGoal(ax, startPt, goalPt, cfg)
    plot(ax, startPt(1), startPt(2), 'o', ...
        'MarkerSize', 7.0, ...
        'MarkerFaceColor', cfg.startColor, ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'LineWidth', 0.75);
    plot(ax, goalPt(1), goalPt(2), 's', ...
        'MarkerSize', 7.0, ...
        'MarkerFaceColor', cfg.goalColor, ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'LineWidth', 0.75);

    text(ax, startPt(1) + 0.018, startPt(2) - 0.038, 'S', ...
        'FontName', cfg.fontName, 'FontSize', 9.5, 'FontWeight', 'bold');
    text(ax, goalPt(1) - 0.025, goalPt(2) - 0.038, 'G', ...
        'FontName', cfg.fontName, 'FontSize', 9.5, 'FontWeight', 'bold');
end

function [handles, labels] = createSharedLegendHandles(ax, cfg)
    handles = gobjects(6, 1);
    handles(1) = plot(ax, nan, nan, '--', 'Color', cfg.initPathColor, ...
        'LineWidth', cfg.initPathLineWidth);
    handles(2) = patch(ax, ...
        'XData', [nan, nan], 'YData', [nan, nan], ...
        'FaceColor', 'none', ...
        'EdgeColor', cfg.collisionColor, ...
        'EdgeAlpha', cfg.collisionChordAlpha, ...
        'LineWidth', 1.6 * cfg.chordLineWidth);
    handles(3) = plot(ax, nan, nan, 'x', ...
        'Color', cfg.initMarkerColor, 'MarkerSize', 7.5, 'LineWidth', 1.4);
    handles(4) = plot(ax, nan, nan, '-', 'Color', cfg.optPathColor, ...
        'LineWidth', cfg.optPathLineWidth);
    handles(5) = plot(ax, nan, nan, '-', 'Color', cfg.chordColor, ...
        'LineWidth', cfg.chordLineWidth);
    handles(6) = plot(ax, nan, nan, 'p', ...
        'MarkerFaceColor', cfg.optMarkerColor, ...
        'MarkerEdgeColor', [0.1, 0.1, 0.1], 'MarkerSize', 7.5);

    % MATLAB fills multi-column legends column by column.
    labels = {'Initial path', 'Below-dMin chords', ...
        'Initial minimum clearance', 'CSSC path', ...
        'Fixed-length chords', 'CSSC minimum clearance'};
end
