function visualizeCSSC2DOptimization(runInput, varargin)
%VISUALIZECSSC2DOPTIMIZATION Visualize saved CSSC optimization process.
%
% Usage:
%   visualizeCSSC2DOptimization('results/run_20260608_120000');
%   visualizeCSSC2DOptimization('results/run_xxx/final_result.mat');
%   visualizeCSSC2DOptimization('results/run_xxx', 'SaveFigures', true);
%
% It visualizes:
%   1) initial/reference/optimized paths;
%   2) intermediate snapshots;
%   3) obstacles if the obstacle format is recognized;
%   4) objective and clearance histories;
%   5) timing histories.

    p = inputParser;
    addRequired(p, 'runInput', @(x) ischar(x) || isstring(x));
    addParameter(p, 'SaveFigures', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ShowSnapshots', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'MaxSnapshots', 25, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'MakeAnimation', false, @(x) islogical(x) || isnumeric(x));
    parse(p, runInput, varargin{:});
    opt = p.Results;

    runInput = char(runInput);

    if isfolder(runInput)
        runDir = runInput;
        finalFile = fullfile(runDir, 'final_result.mat');
    else
        finalFile = runInput;
        runDir = fileparts(finalFile);
    end

    if ~exist(finalFile, 'file')
        error('Cannot find final_result.mat: %s', finalFile);
    end

    data = load(finalFile);

    Popt = data.Popt;
    Pinit = data.Pinit;
    Pref = data.Pref;
    obstacles = data.obstacles;
    info = data.info;

    figDir = fullfile(runDir, 'figures');
    if ~exist(figDir, 'dir')
        mkdir(figDir);
    end

    [snapshots, snapshotIters] = loadSnapshots(runDir, info);

    % ---------- Figure 1: path evolution ----------
    fig1 = figure('Color', 'w', 'Name', 'CSSC path evolution');
    hold on; grid on; axis equal;
    title('CSSC optimization path evolution');
    xlabel('x'); ylabel('y');

    plotObstacles2D(obstacles);

    plot(Pref(:,1), Pref(:,2), 'k--', ...
        'LineWidth', 1.5, 'DisplayName', 'Reference path');

    plot(Pinit(:,1), Pinit(:,2), '-', ...
        'Color', [0.60 0.60 0.60], ...
        'LineWidth', 1.8, 'DisplayName', 'Initial path');

    if opt.ShowSnapshots && ~isempty(snapshots)
        keepIdx = round(linspace(1, numel(snapshots), min(opt.MaxSnapshots, numel(snapshots))));
        cmap = parula(numel(keepIdx));
        for j = 1:numel(keepIdx)
            Pj = snapshots{keepIdx(j)};
            iterj = snapshotIters(keepIdx(j));
            plot(Pj(:,1), Pj(:,2), '-', ...
                'Color', cmap(j,:), ...
                'LineWidth', 0.8, ...
                'DisplayName', sprintf('Snapshot %d', iterj));
        end
    end

    plot(Popt(:,1), Popt(:,2), 'r-', ...
        'LineWidth', 2.8, 'DisplayName', 'Optimized path');

    scatter(Popt(1,1), Popt(1,2), 60, 'g', 'filled', ...
        'DisplayName', 'Start');
    scatter(Popt(end,1), Popt(end,2), 60, 'm', 'filled', ...
        'DisplayName', 'Goal');

    legend('Location', 'bestoutside');

    if opt.SaveFigures
        saveas(fig1, fullfile(figDir, 'path_evolution.png'));
        savefig(fig1, fullfile(figDir, 'path_evolution.fig'));
    end

    % ---------- Figure 2: objective and clearance ----------
    fig2 = figure('Color', 'w', 'Name', 'CSSC convergence');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile; hold on; grid on;
    Jhist = info.Jhist(:);
    validJ = isfinite(Jhist) & (Jhist ~= 0);
    plot(find(validJ), Jhist(validJ), 'LineWidth', 1.8);
    xlabel('Iteration'); ylabel('Objective J');
    title('Objective history');

    nexttile; hold on; grid on;
    minClearHist = info.minClearHist(:);
    validC = isfinite(minClearHist) & (minClearHist ~= 0);
    plot(find(validC), minClearHist(validC), 'LineWidth', 1.8);
    xlabel('Iteration'); ylabel('Minimum clearance');
    title('Minimum clearance history');

    if opt.SaveFigures
        saveas(fig2, fullfile(figDir, 'convergence.png'));
        savefig(fig2, fullfile(figDir, 'convergence.fig'));
    end

    % ---------- Figure 3: timing ----------
    if isfield(info, 'timing')
        fig3 = figure('Color', 'w', 'Name', 'CSSC timing');
        hold on; grid on;

        it = (1:numel(info.timing.totalIter)).';
        validT = info.timing.totalIter(:) > 0;

        plot(it(validT), info.timing.totalIter(validT) * 1000, ...
            'LineWidth', 1.8, 'DisplayName', 'Total / iter');
        plot(it(validT), info.timing.mainObjective(validT) * 1000, ...
            'LineWidth', 1.4, 'DisplayName', 'Main objective');
        plot(it(validT), info.timing.gradient(validT) * 1000, ...
            'LineWidth', 1.4, 'DisplayName', 'Finite-diff gradient');
        plot(it(validT), info.timing.update(validT) * 1000, ...
            'LineWidth', 1.4, 'DisplayName', 'Adam update');

        xlabel('Iteration'); ylabel('Time [ms]');
        title('Timing history');
        legend('Location', 'best');

        if opt.SaveFigures
            saveas(fig3, fullfile(figDir, 'timing.png'));
            savefig(fig3, fullfile(figDir, 'timing.fig'));
        end
    end

    if opt.MakeAnimation && ~isempty(snapshots)
        makePathAnimation(figDir, Pref, Pinit, Popt, obstacles, snapshots, snapshotIters);
    end

    fprintf('Visualization finished.\n');
    fprintf('Run directory : %s\n', runDir);
    fprintf('Figure folder : %s\n', figDir);
end

function [snapshots, snapshotIters] = loadSnapshots(runDir, info)
    snapshots = {};
    snapshotIters = [];

    if isfield(info, 'snapshots') && ~isempty(info.snapshots)
        snapshots = info.snapshots;
        snapshotIters = info.snapshotIters;
        return;
    end

    snapshotDir = fullfile(runDir, 'snapshots');
    if ~exist(snapshotDir, 'dir')
        return;
    end

    files = dir(fullfile(snapshotDir, 'stage_iter_*.mat'));
    if isempty(files)
        return;
    end

    [~, ord] = sort({files.name});
    files = files(ord);

    for i = 1:numel(files)
        f = fullfile(snapshotDir, files(i).name);
        s = load(f);
        if isfield(s, 'Pcur') && isfield(s, 'iter')
            snapshots{end+1} = s.Pcur; %#ok<AGROW>
            snapshotIters(end+1) = s.iter; %#ok<AGROW>
        end
    end
end

function plotObstacles2D(obstacles)
    % Supported formats:
    % 1) numeric Nx3: [x, y, radius]
    % 2) numeric Nx2: point obstacles
    % 3) struct with centers Nx2 and radii Nx1
    % 4) struct array with center and radius
    % 5) struct with x, y, r

    if isempty(obstacles)
        return;
    end

    if isnumeric(obstacles)
        if size(obstacles,2) >= 3
            for i = 1:size(obstacles,1)
                drawCircle(obstacles(i,1:2), obstacles(i,3));
            end
        elseif size(obstacles,2) == 2
            scatter(obstacles(:,1), obstacles(:,2), 20, 'k', 'filled', ...
                'DisplayName', 'Obstacle points');
        end
        return;
    end

    if isstruct(obstacles)
        if isfield(obstacles, 'centers') && isfield(obstacles, 'radii')
            centers = obstacles.centers;
            radii = obstacles.radii;
            for i = 1:size(centers,1)
                drawCircle(centers(i,:), radii(i));
            end
            return;
        end

        if numel(obstacles) > 1 && isfield(obstacles, 'center') && isfield(obstacles, 'radius')
            for i = 1:numel(obstacles)
                drawCircle(obstacles(i).center, obstacles(i).radius);
            end
            return;
        end

        if isfield(obstacles, 'x') && isfield(obstacles, 'y') && isfield(obstacles, 'r')
            for i = 1:numel(obstacles.x)
                drawCircle([obstacles.x(i), obstacles.y(i)], obstacles.r(i));
            end
            return;
        end
    end

    warning('Unknown obstacle format. Skip obstacle plotting.');
end

function drawCircle(c, r)
    th = linspace(0, 2*pi, 80);
    x = c(1) + r * cos(th);
    y = c(2) + r * sin(th);
    patch(x, y, [0.85 0.85 0.85], ...
        'EdgeColor', [0.25 0.25 0.25], ...
        'LineWidth', 0.8, ...
        'FaceAlpha', 0.55, ...
        'HandleVisibility', 'off');
end

function makePathAnimation(figDir, Pref, Pinit, Popt, obstacles, snapshots, snapshotIters)
    fig = figure('Color', 'w', 'Name', 'CSSC path animation');
    hold on; grid on; axis equal;
    title('CSSC optimization animation');
    xlabel('x'); ylabel('y');

    plotObstacles2D(obstacles);
    plot(Pref(:,1), Pref(:,2), 'k--', 'LineWidth', 1.2);
    plot(Pinit(:,1), Pinit(:,2), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    plot(Popt(:,1), Popt(:,2), 'r-', 'LineWidth', 2.4);

    hPath = plot(nan, nan, 'b-', 'LineWidth', 2.0);
    hTitle = title('');

    videoFile = fullfile(figDir, 'path_optimization_animation.mp4');
    vw = VideoWriter(videoFile, 'MPEG-4');
    vw.FrameRate = 10;
    open(vw);

    for i = 1:numel(snapshots)
        P = snapshots{i};
        iter = snapshotIters(i);

        set(hPath, 'XData', P(:,1), 'YData', P(:,2));
        set(hTitle, 'String', sprintf('Optimization snapshot, iter %d', iter));

        drawnow;
        frame = getframe(fig);
        writeVideo(vw, frame);
    end

    close(vw);
    savefig(fig, fullfile(figDir, 'path_optimization_animation.fig'));
    fprintf('Animation saved: %s\n', videoFile);
end
