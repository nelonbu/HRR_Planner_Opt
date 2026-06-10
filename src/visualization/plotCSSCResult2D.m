function fig = plotCSSCResult2D(Pinit, Popt, obstacles, params, info)
%PLOTCSSCRESULT2D Plot initial/final CSSC paths and objective components.

    if ~isfield(params, 'plot') || isempty(params.plot)
        params.plot = struct();
    end
    if ~isfield(params.plot, 'showProcess'); params.plot.showProcess = false; end
    if ~isfield(params.plot, 'saveFinalFigure'); params.plot.saveFinalFigure = false; end
    if ~isfield(params.plot, 'outputDir'); params.plot.outputDir = ''; end

    % 避免绘图时 evaluateCSSCGlobal 打印 timing
    paramsPlot = params;
    paramsPlot.printEvalTiming = false;
    paramsPlot.enablePathSample = true;

    stateInit = evaluateCSSCGlobal(Pinit, obstacles, paramsPlot);
    stateOpt  = evaluateCSSCGlobal(Popt,  obstacles, paramsPlot);

    fig = figure('Color','w','Name','CSSC result','Position',[100 100 1400 650]);

    %% ============================================================
    % 1. Geometry
    %% ============================================================
    subplot(1,2,1); hold on; grid on; axis equal;
    title('CSSC-FTL path and swept segments');
    xlabel('x'); ylabel('y');

    drawObstacles(obstacles, params);

    plot(stateInit.pathSample(:,1), stateInit.pathSample(:,2), '--', ...
        'LineWidth', 1.5, 'Color', [0.45 0.45 0.45], ...
        'DisplayName', 'initial path');

    plot(stateOpt.pathSample(:,1), stateOpt.pathSample(:,2), 'k-', ...
        'LineWidth', 2.4, 'DisplayName', 'optimized path');

    plot(Pinit(:,1), Pinit(:,2), 'o--', ...
        'Color', [0.5 0.5 0.5], ...
        'MarkerFaceColor', 'w', ...
        'DisplayName', 'initial ctrl');

    plot(Popt(:,1), Popt(:,2), 'ro-', ...
        'LineWidth', 1.0, ...
        'MarkerFaceColor', 'r', ...
        'DisplayName', 'optimized ctrl');

    drawSparseChords(stateOpt, 35);

    bad = stateOpt.clearance < params.dMin & isfinite(stateOpt.clearance);
    if any(bad)
        scatter(stateOpt.closestPoint(bad,1), stateOpt.closestPoint(bad,2), ...
            45, 'r', 'filled', 'DisplayName', 'unsafe closest pts');
    end

    if all(isfinite(stateOpt.minPoint))
        plot(stateOpt.minPoint(1), stateOpt.minPoint(2), 'rp', ...
            'MarkerSize', 16, ...
            'MarkerFaceColor', 'r', ...
            'DisplayName', 'min clearance');

        text(stateOpt.minPoint(1), stateOpt.minPoint(2), ...
            sprintf(' min %.3f', stateOpt.minClear), ...
            'Color', 'r', ...
            'FontWeight', 'bold');
    end

    legend('Location','bestoutside');

    %% ============================================================
    % 2. Objective components in the same coordinate system
    %% ============================================================
    subplot(1,2,2); hold on; grid on;
    title('Objective components');
    xlabel('iteration');
    ylabel('cost value');

    plotted = false;

    Jhist       = getHistory(info, 'J');
    JobsHist    = getHistory(info, 'Jobs');
    JclearHist  = getHistory(info, 'Jclear');
    JregHist    = getHistory(info, 'Jreg');
    JrefHist    = getHistory(info, 'Jref');
    JsmoothHist = getHistory(info, 'Jsmooth');
    JlenHist    = getHistory(info, 'Jlen');
    JtrustHist  = getHistory(info, 'Jtrust');

    if ~isempty(Jhist)
        plot(Jhist, 'k-', 'LineWidth', 2.4, 'DisplayName', 'J total');
        plotted = true;
    end

    if ~isempty(JobsHist)
        plot(JobsHist, '-', 'LineWidth', 1.8, 'DisplayName', 'J_{obs}');
        plotted = true;
    end

    if ~isempty(JclearHist)
        plot(JclearHist, '-', 'LineWidth', 1.8, 'DisplayName', 'J_{clear}');
        plotted = true;
    end

    if ~isempty(JregHist)
        plot(JregHist, '-', 'LineWidth', 1.8, 'DisplayName', 'J_{reg}');
        plotted = true;
    end

    if ~isempty(JrefHist)
        plot(JrefHist, '--', 'LineWidth', 1.4, 'DisplayName', 'J_{ref}');
        plotted = true;
    end

    if ~isempty(JsmoothHist)
        plot(JsmoothHist, '--', 'LineWidth', 1.4, 'DisplayName', 'J_{smooth}');
        plotted = true;
    end

    if ~isempty(JlenHist)
        plot(JlenHist, '--', 'LineWidth', 1.4, 'DisplayName', 'J_{len}');
        plotted = true;
    end

    if ~isempty(JtrustHist)
        plot(JtrustHist, '--', 'LineWidth', 1.4, 'DisplayName', 'J_{trust}');
        plotted = true;
    end

    if plotted
        legend('Location','best');
    else
        % 如果 info 没有分项历史，则退回画最终 clearance profile
        plot(stateOpt.u, stateOpt.clearance, 'b-', ...
            'LineWidth', 1.8, ...
            'DisplayName', 'final segment clearance');

        yline(params.dMin, 'r--', 'DisplayName', 'dMin');
        xlabel('u');
        ylabel('clearance');
        title('Final clearance profile');
        legend('Location','best');
    end

    fprintf('\n[Plot summary]\n');
    fprintf('  initial minClear = %.6f\n', stateInit.minClear);
    fprintf('  final   minClear = %.6f\n', stateOpt.minClear);

    if params.plot.saveFinalFigure && ~isempty(params.plot.outputDir)
        saveFigureCompat(fig, fullfile(params.plot.outputDir, 'final_result.png'));
    end

    if params.plot.showProcess
        plotCSSCOptimizationProcess2D(Pinit, Popt, obstacles, params, info);
    end
end

function saveFigureCompat(fig, filePath)
    try
        exportgraphics(fig, filePath, 'Resolution', 180);
    catch
        saveas(fig, filePath);
    end
end

%% ============================================================
% Helper: get history from info robustly
%% ============================================================

function h = getHistory(info, name)
%GETHISTORY Read optimization history robustly.
%
% Supports:
%   info.Jhist
%   info.JobsHist
%   info.JclearHist
%   ...
% and also:
%   info.detailsHist(k).J
%   info.detailsHist(k).Jobs
%   ...

    h = [];

    % Common direct field names
    candidates = {
        [name 'Hist']
        [name 'hist']
        [lower(name) 'Hist']
        [lower(name) 'hist']
    };

    % Special case: total objective usually saved as Jhist
    if strcmp(name, 'J')
        candidates = [{'Jhist'}; candidates(:)];
    end

    for i = 1:numel(candidates)
        f = candidates{i};
        if isfield(info, f)
            h = info.(f);
            h = h(:);
            return;
        end
    end

    % Try detailsHist / detailHist as struct array
    detailFields = {'detailsHist', 'detailHist', 'detailsHistory', 'detailHistory'};

    for k = 1:numel(detailFields)
        df = detailFields{k};
        if isfield(info, df)
            D = info.(df);
            if isstruct(D) && isfield(D, name)
                try
                    h = arrayfun(@(s) s.(name), D);
                    h = h(:);
                    return;
                catch
                    h = [];
                end
            end
        end
    end
end

%% ============================================================
% Drawing helpers
%% ============================================================

function drawObstacles(obstacles, params)
    for k = 1:numel(obstacles)
        obs = obstacles(k);

        switch lower(obs.type)
            case 'circle'
                drawCircle(obs.center, obs.radius, [0.2 0.2 0.2], 1.5, '-');
                drawCircle(obs.center, obs.radius + params.dMin, ...
                    [0.85 0.2 0.2], 0.9, '--');

            case {'rect', 'rectangle', 'box'}
                drawRect(obs.center, obs.halfSize, obs.yaw, [0.2 0.2 0.2], 1.5, '-');
                drawRect(obs.center, obs.halfSize + params.dMin, ...
                    obs.yaw, [0.85 0.2 0.2], 0.9, '--');
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
                'Color', [0.75 0.75 0.75], ...
                'HandleVisibility', 'off');
        end
    end
end

function drawCircle(c, r, color, lw, style)
    th = linspace(0, 2*pi, 160);
    plot(c(1) + r*cos(th), c(2) + r*sin(th), ...
        'Color', color, ...
        'LineWidth', lw, ...
        'LineStyle', style, ...
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
        'Color', color, ...
        'LineWidth', lw, ...
        'LineStyle', style, ...
        'HandleVisibility','off');
end
