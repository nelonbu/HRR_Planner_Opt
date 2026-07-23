function figs = plotCSSCOptimizationProcess2D(Pinit, Popt, obstacles, params, info)
%PLOTCSSCOPTIMIZATIONPROCESS2D Render optimization snapshots as image/video.

    params = setPlotDefaults(params);
    paramsPlot = params;
    paramsPlot.printEvalTiming = false;
    paramsPlot.enablePathSample = true;

    [Pseq, iterSeq] = collectSnapshotSequence(Pinit, Popt, info);
    if isempty(Pseq)
        figs = struct('process', []);
        return;
    end

    outputDir = params.plot.outputDir;
    figs = struct();

    if params.plot.saveProcessImage || params.plot.showProcessFigure
        imageIdx = selectImageIndices(numel(Pseq), params.plot.maxProcessCurves);
        figs.process = renderProcessImage(Pseq, iterSeq, imageIdx, obstacles, paramsPlot);

        if params.plot.saveProcessImage && ~isempty(outputDir)
            saveFigureCompat(figs.process, fullfile(outputDir, 'optimization_process.png'));
        end
    else
        figs.process = [];
    end

    if params.plot.saveProcessVideo && ~isempty(outputDir)
        videoPath = fullfile(outputDir, 'optimization_process.mp4');
        renderProcessVideo(Pseq, iterSeq, obstacles, paramsPlot, videoPath);
    end
end

function params = setPlotDefaults(params)
    if ~isfield(params, 'plot') || isempty(params.plot)
        params.plot = struct();
    end
    if ~isfield(params.plot, 'outputDir'); params.plot.outputDir = ''; end
    if ~isfield(params.plot, 'showProcessFigure'); params.plot.showProcessFigure = true; end
    if ~isfield(params.plot, 'saveProcessImage'); params.plot.saveProcessImage = false; end
    if ~isfield(params.plot, 'saveProcessVideo'); params.plot.saveProcessVideo = false; end
    if ~isfield(params.plot, 'maxProcessCurves'); params.plot.maxProcessCurves = 25; end
    if ~isfield(params.plot, 'videoFPS'); params.plot.videoFPS = 8; end
    if ~isfield(params.plot, 'videoQuality'); params.plot.videoQuality = 95; end
    if ~isfield(params.plot, 'videoDrawChords'); params.plot.videoDrawChords = true; end
    if ~isfield(params.plot, 'videoDrawMinClear'); params.plot.videoDrawMinClear = true; end
end

function [Pseq, iterSeq] = collectSnapshotSequence(Pinit, Popt, info)
    Pseq = {Pinit};
    iterSeq = 0;

    if isfield(info, 'snapshots') && iscell(info.snapshots)
        for k = 1:numel(info.snapshots)
            if ~isempty(info.snapshots{k})
                Pseq{end+1,1} = info.snapshots{k}; %#ok<AGROW>
                iterSeq(end+1,1) = k; %#ok<AGROW>
            end
        end
    end

    finalIter = numel(iterSeq);
    if isfield(info, 'numIterActual')
        finalIter = info.numIterActual;
    elseif isfield(info, 'Jhist')
        finalIter = numel(info.Jhist);
    end

    if isempty(Pseq) || ~isequal(size(Pseq{end}), size(Popt)) || norm(Pseq{end}(:) - Popt(:)) > 1e-12
        Pseq{end+1,1} = Popt;
        iterSeq(end+1,1) = finalIter;
    end
end

function idx = selectImageIndices(n, maxCurves)
    maxCurves = max(2, round(maxCurves));
    if n <= maxCurves
        idx = 1:n;
    else
        idx = unique(round(linspace(1, n, maxCurves)));
    end
end

function fig = renderProcessImage(Pseq, iterSeq, imageIdx, obstacles, params)
    fig = figure('Color','w','Name','CSSC optimization process','Position',[120 120 900 720]);
    hold on; grid on; axis equal;
    title('CSSC optimization process');
    xlabel('x'); ylabel('y');

    drawObstacles(obstacles, params);
    applyStableAxes(Pseq, obstacles, params);

    n = numel(imageIdx);
    cmap = parula(max(n, 2));

    for ii = 1:n
        idx = imageIdx(ii);
        P = Pseq{idx};
        state = evaluateCSSCGlobal(P, obstacles, params);

        if idx == 1
            plot(state.pathSample(:,1), state.pathSample(:,2), '--', ...
                'LineWidth', 1.5, 'Color', [0.45 0.45 0.45], ...
                'DisplayName', 'initial path');
        elseif idx == numel(Pseq)
            plot(state.pathSample(:,1), state.pathSample(:,2), 'k-', ...
                'LineWidth', 2.5, 'DisplayName', 'final path');
        else
            plot(state.pathSample(:,1), state.pathSample(:,2), '-', ...
                'LineWidth', 1.2, 'Color', cmap(ii,:), ...
                'HandleVisibility', 'off');
        end
    end

    legend('Location','bestoutside');
    text(0.02, 0.98, sprintf('%d snapshots, iter %d to %d', ...
        numel(Pseq), iterSeq(1), iterSeq(end)), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
end

function renderProcessVideo(Pseq, iterSeq, obstacles, params, videoPath)
    fig = figure('Color','w','Name','CSSC optimization video','Position',[120 120 900 720], ...
        'Visible', 'on');

    stateInit = evaluateCSSCGlobal(Pseq{1}, obstacles, params);

    writer = VideoWriter(videoPath, 'MPEG-4');
    writer.FrameRate = params.plot.videoFPS;
    writer.Quality = params.plot.videoQuality;
    open(writer);

    cleanupObj = onCleanup(@() closeVideo(writer));

    for idx = 1:numel(Pseq)
        clf(fig);
        hold on; grid on; axis equal;
        title(sprintf('iter %d', iterSeq(idx)));
        xlabel('x'); ylabel('y');

        drawObstacles(obstacles, params);
        applyStableAxes(Pseq, obstacles, params);

        state = evaluateCSSCGlobal(Pseq{idx}, obstacles, params);

        if idx > 1
            plot(stateInit.pathSample(:,1), stateInit.pathSample(:,2), '--', ...
                'LineWidth', 1.0, 'Color', [0.7 0.7 0.7]);
        end

        plot(state.pathSample(:,1), state.pathSample(:,2), 'k-', 'LineWidth', 2.4);
        plot(Pseq{idx}(:,1), Pseq{idx}(:,2), 'ro-', ...
            'LineWidth', 0.9, 'MarkerFaceColor', 'r', 'MarkerSize', 4);

        if params.plot.videoDrawChords
            drawSparseChords(state, 25);
        end

        if params.plot.videoDrawMinClear && all(isfinite(state.minPoint))
            plot(state.minPoint(1), state.minPoint(2), 'rp', ...
                'MarkerSize', 14, 'MarkerFaceColor', 'r');
        end

        label = sprintf('iter %d | minClear %.4f', iterSeq(idx), state.minClear);
        text(0.02, 0.98, label, 'Units', 'normalized', ...
            'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
            'BackgroundColor', 'w');

        drawnow;
        writeVideo(writer, getframe(fig));
    end

    close(writer);
    delete(cleanupObj);

    if ~params.plot.showProcessFigure
        close(fig);
    end
end

function closeVideo(writer)
    try
        close(writer);
    catch
    end
end

function drawObstacles(obstacles, params)
    for k = 1:numel(obstacles)
        obs = obstacles(k);

        switch lower(obs.type)
            case 'circle'
                drawCircle(obs.center, obs.radius, [0.2 0.2 0.2], 1.5, '-');
                drawCircle(obs.center, obs.radius + params.dMin, [0.85 0.2 0.2], 0.9, '--');

            case {'rect', 'rectangle', 'box'}
                drawRect(obs.center, obs.halfSize, obs.yaw, [0.2 0.2 0.2], 1.5, '-');
                drawRect(obs.center, obs.halfSize + params.dMin, obs.yaw, [0.85 0.2 0.2], 0.9, '--');

            case 'polygon'
                drawPolygon(obs.vertices, [0.2 0.2 0.2], 1.5, '-');
        end
    end
end

function drawSparseChords(state, nDraw)
    idx = find(state.validLine);
    if isempty(idx)
        return;
    end

    skip = max(1, floor(numel(idx) / nDraw));
    for ii = 1:skip:numel(idx)
        k = idx(ii);
        M = state.M(k,:);
        N = state.N(k,:);
        if all(isfinite(M)) && all(isfinite(N))
            plot([M(1), N(1)], [M(2), N(2)], '-', ...
                'Color', [0.78 0.78 0.78], 'HandleVisibility', 'off');
        end
    end
end

function drawCircle(c, r, color, lw, style)
    th = linspace(0, 2*pi, 160);
    plot(c(1) + r*cos(th), c(2) + r*sin(th), ...
        'Color', color, 'LineWidth', lw, 'LineStyle', style, ...
        'HandleVisibility','off');
end

function drawRect(c, h, yaw, color, lw, style)
    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];

    pts = [
        -h(1), -h(2)
         h(1), -h(2)
         h(1),  h(2)
        -h(1),  h(2)
        -h(1), -h(2)
    ];

    pts = (R * pts.').' + c;

    plot(pts(:,1), pts(:,2), ...
        'Color', color, 'LineWidth', lw, 'LineStyle', style, ...
        'HandleVisibility','off');
end

function drawPolygon(vertices, color, lw, style)
    pts = [vertices; vertices(1,:)];
    plot(pts(:,1), pts(:,2), ...
        'Color', color, 'LineWidth', lw, 'LineStyle', style, ...
        'HandleVisibility','off');
end

function applyStableAxes(Pseq, obstacles, params)
    pts = [];
    for k = 1:numel(Pseq)
        pts = [pts; Pseq{k}]; %#ok<AGROW>
    end

    for k = 1:numel(obstacles)
        obs = obstacles(k);
        switch lower(obs.type)
            case 'circle'
                r = obs.radius + params.dMin;
                pts = [pts; obs.center + [-r, -r]; obs.center + [r, r]]; %#ok<AGROW>
            case {'rect', 'rectangle', 'box'}
                h = obs.halfSize + params.dMin;
                pts = [pts; obs.center - h; obs.center + h]; %#ok<AGROW>
            case 'polygon'
                pts = [pts; obs.vertices]; %#ok<AGROW>
        end
    end

    lo = min(pts, [], 1);
    hi = max(pts, [], 1);
    span = max(hi - lo);
    if ~isfinite(span) || span <= 0
        span = 1;
    end
    pad = 0.08 * span;
    xlim([lo(1)-pad, hi(1)+pad]);
    ylim([lo(2)-pad, hi(2)+pad]);
end

function saveFigureCompat(fig, filePath)
    try
        exportgraphics(fig, filePath, 'Resolution', 180);
    catch
        saveas(fig, filePath);
    end
end
