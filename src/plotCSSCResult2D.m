function plotCSSCResult2D(Pinit, Popt, obstacles, params, info)
%PLOTCSSCRESULT2D Visualize initial/optimized path and fixed-chord envelope.

    if nargin < 5
        info = [];
    end

    pathOpts.degree = params.degree;

    rInit  = @(w) evalBSplinePath2D(Pinit, w, pathOpts);
    drInit = @(w) evalBSplineDerivOnly(Pinit, w, pathOpts);
    envInit = fixedChordEnvelope(rInit, drInit, params.L, params.envOpts);

    rOpt  = @(w) evalBSplinePath2D(Popt, w, pathOpts);
    drOpt = @(w) evalBSplineDerivOnly(Popt, w, pathOpts);
    envOpt = fixedChordEnvelope(rOpt, drOpt, params.L, params.envOpts);

    wPlot = linspace(0, 1, 600).';
    Cinit = evalBSplinePath2D(Pinit, wPlot, pathOpts);
    Copt = evalBSplinePath2D(Popt, wPlot, pathOpts);

    figure('Color','w'); hold on; grid on; axis equal;
    title('CSSC-FTL 2D MVP result');
    xlabel('x'); ylabel('y');

    % Obstacles
    for i = 1:size(obstacles,1)
        drawCircle(obstacles(i,1:2), obstacles(i,3), [0.85 0.85 0.85], [0.25 0.25 0.25]);
    end

    % Control polygons
    plot(Pinit(:,1), Pinit(:,2), 'k--o', 'LineWidth', 0.8, ...
        'MarkerSize', 4, 'DisplayName', 'Initial control polygon');
    plot(Popt(:,1), Popt(:,2), 'b--o', 'LineWidth', 1.0, ...
        'MarkerSize', 4, 'DisplayName', 'Optimized control polygon');

    % Curves
    plot(Cinit(:,1), Cinit(:,2), 'k-', 'LineWidth', 1.5, ...
        'DisplayName', 'Initial path');
    plot(Copt(:,1), Copt(:,2), 'b-', 'LineWidth', 2.5, ...
        'DisplayName', 'Optimized path');

    % Envelopes
    plotEnvelope(envInit, [0.7 0.7 0.7], 'Initial envelope');
    plotEnvelope(envOpt, [1 0 0], 'Optimized envelope');

    % Moving segments after optimization
    drawMovingSegments(envOpt, 35, [0.2 0.6 1.0]);

    legend('Location','bestoutside');

    % History figure
    if ~isempty(info) && isfield(info, 'Jhist')
        figure('Color','w');
        subplot(2,1,1); grid on; hold on;
        plot(info.Jhist, 'LineWidth', 1.5);
        xlabel('Iteration'); ylabel('J'); title('Objective history');

        subplot(2,1,2); grid on; hold on;
        plot(info.minClearHist, 'LineWidth', 1.5);
        yline(params.dMin, 'r--', 'dMin');
        xlabel('Iteration'); ylabel('Minimum clearance');
        title('Swept-segment clearance history');
    end
end

%% ============================================================
% Local helpers
%% ============================================================

function dr = evalBSplineDerivOnly(Pctrl, w, pathOpts)
    [~, dr] = evalBSplinePath2D(Pctrl, w, pathOpts);
end

function plotEnvelope(env, colorVal, nameStr)
    valid = env.validSegment & all(isfinite(env.G),2);
    if any(valid)
        plot(env.G(valid,1), env.G(valid,2), '-', ...
            'Color', colorVal, 'LineWidth', 2.0, 'DisplayName', nameStr);
    end
end

function drawMovingSegments(env, nSegToDraw, colorVal)
    validIdx = find(env.validLine & all(isfinite(env.M),2) & all(isfinite(env.N),2));
    if isempty(validIdx)
        return;
    end
    step = max(1, floor(numel(validIdx) / nSegToDraw));
    for ii = 1:step:numel(validIdx)
        k = validIdx(ii);
        plot([env.M(k,1), env.N(k,1)], [env.M(k,2), env.N(k,2)], '-', ...
            'Color', colorVal, 'LineWidth', 0.7, 'HandleVisibility','off');
    end
end

function drawCircle(c, r, faceColor, edgeColor)
    th = linspace(0, 2*pi, 80);
    x = c(1) + r * cos(th);
    y = c(2) + r * sin(th);
    patch(x, y, faceColor, 'EdgeColor', edgeColor, ...
        'FaceAlpha', 0.9, 'HandleVisibility','off');
end
