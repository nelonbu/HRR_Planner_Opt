clear; clc; close all;

%% FIG_SCENES_DISPLAY Four benchmark scene families at three difficulty levels.
% Rows: offset double slit, four-rectangle S channel, staggered baffles,
% and random mixed obstacles. Columns: low, medium, and high difficulty.
% The figure and its reproducibility data are saved under
% results/figures/benchmark_scenes.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

%% Shared scene and difficulty parameters
% Edit demos/getCSSCDemoConfig2D.m to update all common benchmark demos.
cfg = getCSSCDemoConfig2D();

% Figure output.
cfg.figureSizeCm = [11.0, 13.5];
cfg.resolutionPPI = 400;
cfg.visible = 'on';                 % Use 'off' for unattended batch export.
cfg.fontName = 'Times New Roman';
cfg.obstacleFaceColor = [0.76, 0.78, 0.80];
cfg.obstacleEdgeColor = [0.16, 0.17, 0.18];
cfg.startColor = [0.08, 0.58, 0.55];
cfg.goalColor = [0.93, 0.64, 0.12];
cfg.dimensionColor = [0.16, 0.32, 0.62];

%% Generate the 4-by-3 benchmark grid
nDifficulty = numel(cfg.difficultyNames);
if nDifficulty ~= 3
    error('cfg.difficultyNames must contain exactly three levels.');
end

scenes = cell(4, nDifficulty);
for level = 1:nDifficulty
    scenes{1,level} = makeDoubleSlitScene(cfg, level);
    scenes{2,level} = makeSChannelScene(cfg, level);
    scenes{3,level} = makeBaffleScene(cfg, level);
    scenes{4,level} = makeRandomMixedScene(cfg, level);
end

%% Draw and export
outDir = fullfile(projectRoot, 'results', 'figures', 'benchmark_scenes');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

fig = figure( ...
    'Color', 'w', ...
    'Visible', cfg.visible, ...
    'Units', 'centimeters', ...
    'Position', [2, 2, cfg.figureSizeCm], ...
    'PaperUnits', 'centimeters', ...
    'PaperPosition', [0, 0, cfg.figureSizeCm]);

layout = tiledlayout(fig, 4, 3, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

for row = 1:4
    for level = 1:nDifficulty
        ax = nexttile(layout, (row - 1) * nDifficulty + level);
        drawBenchmarkScene(ax, scenes{row,level}, cfg, row, level);
    end
end

% sgtitle(layout, sprintf('Benchmark scenes at three difficulty levels   ($d_{min}=%.3f$)', ...
%     cfg.dMin), 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');

jpgPath = fullfile(outDir, 'fig_scenes_display.jpg');
pdfPath = fullfile(outDir, 'fig_scenes_display.pdf');
matPath = fullfile(outDir, 'fig_scenes_display_data.mat');
exportgraphics(fig, jpgPath, 'Resolution', cfg.resolutionPPI, 'BackgroundColor', 'white');
exportgraphics(fig, pdfPath, 'ContentType', 'vector', 'BackgroundColor', 'white');
save(matPath, 'cfg', 'scenes');

fprintf('[fig_scenes_display]\n');
fprintf('  JPG : %s\n', jpgPath);
fprintf('  PDF : %s\n', pdfPath);
fprintf('  data: %s\n', matPath);

%% Local scene builders
function scene = makeDoubleSlitScene(cfg, level)
    opts = commonOpts(cfg);
    opts.xWalls = cfg.doubleSlit.xWalls;
    opts.gapHeight = cfg.doubleSlit.dGap(level);
    opts.wallThickness = cfg.doubleSlit.wallThickness;
    % Formal runs reuse the same per-family seed sequence across difficulty
    % levels; show the first trial of each level.
    opts.seed = cfg.envSeedBaseByFamily(1);
    opts.gapCentersY = sampleDoubleSlitCenters( ...
        cfg.doubleSlit.centerRange(level,:), cfg.bounds, ...
        opts.gapHeight, opts.seed);

    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
        'offsetDoubleSlit', opts);

    scene = packScene(obstacles, envInfo, 'Double slit');
    scene.metricName = 'd_{gap}';
    scene.metricValue = opts.gapHeight;
    scene.annotation = struct( ...
        'kind', 'verticalGap', ...
        'x', opts.xWalls(1), ...
        'yLow', opts.gapCentersY(1) - 0.5 * opts.gapHeight, ...
        'yHigh', opts.gapCentersY(1) + 0.5 * opts.gapHeight);
end

function centers = sampleDoubleSlitCenters(centerRange, bounds, gapHeight, seed)
    feasibleRange = [bounds(2,1), bounds(2,2)] + ...
        [0.5, -0.5] * gapHeight;
    sampleRange = [max(centerRange(1), feasibleRange(1)), ...
        min(centerRange(2), feasibleRange(2))];
    if sampleRange(1) >= 0 || sampleRange(2) <= 0
        error('Double-slit center range must span zero after clipping.');
    end

    stream = RandStream('mt19937ar', 'Seed', seed);
    centers = [ ...
        sampleRange(1) + (0 - sampleRange(1)) * rand(stream), ...
        0 + (sampleRange(2) - 0) * rand(stream)];
end

function scene = makeSChannelScene(cfg, level)
    opts = commonOpts(cfg);
    opts.dGap = cfg.sChannel.dGap(level);
    opts.W = cfg.sChannel.W;
    opts.H = cfg.sChannel.H;
    opts.seed = cfg.envSeedBaseByFamily(2);
    opts.yInOutRange = cfg.sChannel.yInOutRange;

    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
        'fourRectSChannel', opts);

    rect1 = obstacles(1);
    rect3 = obstacles(3);
    xLeft = rect1.center(1) - rect1.halfSize(1);
    scene = packScene(obstacles, envInfo, 'S channel');
    scene.metricName = 'd_{gap}';
    scene.metricValue = opts.dGap;
    scene.annotation = struct( ...
        'kind', 'verticalGap', ...
        'x', xLeft + 0.035, ...
        'yLow', rect3.center(2) + rect3.halfSize(2), ...
        'yHigh', rect1.center(2) - rect1.halfSize(2));
end

function scene = makeBaffleScene(cfg, level)
    opts = commonOpts(cfg);
    passageWidth = cfg.baffles.passageWidth(level);
    yMin = cfg.bounds(2,1);
    yMax = cfg.bounds(2,2);
    spanY = yMax - yMin;
    halfHeight = 0.5 * (spanY - passageWidth);

    if halfHeight <= 0
        error('Baffle passageWidth must be smaller than the workspace height.');
    end

    centers = zeros(3, 2);
    centers(:,1) = cfg.baffles.xCenters(:);
    for i = 1:3
        if cfg.baffles.pattern(i) > 0
            centers(i,2) = yMax - halfHeight;
        else
            centers(i,2) = yMin + halfHeight;
        end
    end

    opts.centers = centers;
    opts.halfSize = [0.5 * cfg.baffles.thickness, halfHeight];
    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D( ...
        'staggeredBaffles3', opts);

    firstTip = centers(1,2) - halfHeight;
    scene = packScene(obstacles, envInfo, 'Staggered baffles');
    scene.metricName = 'w_{pass}';
    scene.metricValue = passageWidth;
    scene.annotation = struct( ...
        'kind', 'verticalGap', ...
        'x', centers(1,1) + 0.042, ...
        'yLow', yMin, ...
        'yHigh', firstTip);
end

function scene = makeRandomMixedScene(cfg, level)
    opts = commonOpts(cfg);
    opts.seed = cfg.envSeedBaseByFamily(4);
    opts.nCircle = cfg.randomMixed.nCircle(level);
    opts.nRect = cfg.randomMixed.nRect(level);
    opts.minGap = cfg.randomMixed.minGap(level);
    opts.radiusRange = cfg.randomMixed.radiusRange;
    opts.halfSizeXRange = cfg.randomMixed.halfSizeXRange;
    opts.halfSizeYRange = cfg.randomMixed.halfSizeYRange;
    opts.yawRange = cfg.randomMixed.yawRange;
    opts.keepoutStart = cfg.randomMixed.keepoutStart;
    opts.keepoutGoal = cfg.randomMixed.keepoutGoal;
    opts.maxTry = cfg.randomMixed.maxTry;

    [obstacles, envInfo] = generateCSSCEnvironment2D('randomMixed', opts);
    requested = opts.nCircle + opts.nRect;
    if numel(obstacles) < requested
        warning('Random level %d generated %d/%d obstacles.', ...
            level, numel(obstacles), requested);
    end

    occupancy = obstacleArea(obstacles) / prod(diff(cfg.bounds, 1, 2));
    scene = packScene(obstacles, envInfo, 'Random mixed');
    scene.metricName = 'density';
    scene.metricValue = occupancy;
    scene.annotation = struct('kind', 'none');
    scene.numObstacles = numel(obstacles);
    scene.occupancy = occupancy;
end

function opts = commonOpts(cfg)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
end

function scene = packScene(obstacles, envInfo, rowName)
    scene = struct();
    scene.obstacles = obstacles;
    scene.envInfo = envInfo;
    scene.rowName = rowName;
    scene.metricName = '';
    scene.metricValue = nan;
    scene.annotation = struct('kind', 'none');
    scene.numObstacles = numel(obstacles);
    scene.occupancy = nan;
end

%% Local plotting helpers
function drawBenchmarkScene(ax, scene, cfg, row, level)
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');

    bounds = cfg.bounds;
    spanX = diff(bounds(1,:));
    spanY = diff(bounds(2,:));
    xlim(ax, bounds(1,:) + [-1, 1] * 0.015 * spanX);
    ylim(ax, bounds(2,:) + [-1, 1] * 0.015 * spanY);

    drawObstacles(ax, scene.obstacles, cfg);
    drawBoundary(ax, bounds);
    drawStartGoal(ax, cfg.startPt, cfg.goalPt, cfg);
    drawDifficultyAnnotation(ax, scene, cfg);

    if row == 4
        metricText = sprintf('$N_o=%d$', ...
            scene.numObstacles);
    else
        metricText = sprintf('$%s=%.3f$', scene.metricName, scene.metricValue);
    end
    title(ax, sprintf('%s: %s', ...
        char(cfg.difficultyNames(level)), metricText), ...
        'Interpreter', 'latex', 'FontSize', 9, 'FontWeight', 'normal');

    if level == 1
        text(ax, -0.15, 0.5, scene.rowName, ...
            'Units', 'normalized', ...
            'Rotation', 90, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontName','Times','FontSize', 10, 'FontWeight', 'normal', ...
            'Clipping', 'off');
    end
end

function drawObstacles(ax, obstacles, cfg)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 80);
                x = obs.center(1) + obs.radius * cos(theta);
                y = obs.center(2) + obs.radius * sin(theta);
            case {'rect', 'rectangle', 'box'}
                local = [
                    -obs.halfSize(1), -obs.halfSize(2)
                     obs.halfSize(1), -obs.halfSize(2)
                     obs.halfSize(1),  obs.halfSize(2)
                    -obs.halfSize(1),  obs.halfSize(2)
                ];
                R = [cos(obs.yaw), -sin(obs.yaw); sin(obs.yaw), cos(obs.yaw)];
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
            'FaceAlpha', 0.90, ...
            'EdgeColor', cfg.obstacleEdgeColor, ...
            'LineWidth', 0.75);
    end
end

function drawBoundary(ax, bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot(ax, [x(1), x(2), x(2), x(1), x(1)], ...
        [y(1), y(1), y(2), y(2), y(1)], ...
        'k-', 'LineWidth', 1.05, 'Clipping', 'off');
end

function drawStartGoal(ax, startPt, goalPt, cfg)
    plot(ax, startPt(1), startPt(2), 'o', ...
        'MarkerSize', 4.0, ...
        'MarkerFaceColor', cfg.startColor, ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'LineWidth', 0.75);
    plot(ax, goalPt(1), goalPt(2), 's', ...
        'MarkerSize', 4.0, ...
        'MarkerFaceColor', cfg.goalColor, ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'LineWidth', 0.75);

    text(ax, startPt(1) + 0.018, startPt(2) - 0.08, 'S', ...
        'FontName', cfg.fontName, 'FontSize', 6, 'FontWeight', 'bold');
    text(ax, goalPt(1) - 0.025, goalPt(2) - 0.08, 'G', ...
        'FontName', cfg.fontName, 'FontSize', 6, 'FontWeight', 'bold');
end

function drawDifficultyAnnotation(ax, scene, cfg)
    if ~isfield(scene.annotation, 'kind') || ...
            ~strcmpi(scene.annotation.kind, 'verticalGap')
        return;
    end

    ann = scene.annotation;
    spanX = diff(cfg.bounds(1,:));
    tickHalfWidth = 0.010 * spanX;
    x = ann.x;

    plot(ax, [x, x], [ann.yLow, ann.yHigh], '-', ...
        'Color', cfg.dimensionColor, 'LineWidth', 1.0);
    plot(ax, x + [-tickHalfWidth, tickHalfWidth], [ann.yLow, ann.yLow], '-', ...
        'Color', cfg.dimensionColor, 'LineWidth', 1.0);
    plot(ax, x + [-tickHalfWidth, tickHalfWidth], [ann.yHigh, ann.yHigh], '-', ...
        'Color', cfg.dimensionColor, 'LineWidth', 1.0);

    text(ax, x + 0.018 * spanX, 0.5 * (ann.yLow + ann.yHigh), ...
        sprintf('$%s$', scene.metricName), ...
        'Interpreter', 'latex', ...
        'Color', cfg.dimensionColor, ...
        'FontSize', 7.5, ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle');
end

function area = obstacleArea(obstacles)
    area = 0;
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                area = area + pi * obs.radius^2;
            case {'rect', 'rectangle', 'box'}
                area = area + 4 * prod(obs.halfSize);
            case 'polygon'
                area = area + polyarea(obs.vertices(:,1), obs.vertices(:,2));
        end
    end
end
