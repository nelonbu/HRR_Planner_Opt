clear; clc; close all;

%% ============================================================
% Test fixedChordEnvelope.m with a cubic Bezier curve
%% ============================================================

% Cubic Bezier control points
Pctrl = [
    0.0,  0.0;
    1.2,  2.0;
    3.0, -1.2;
    4.5,  1.0
];

% Fixed chord length
L = 1.20;

% Define curve function r(w) and derivative r'(w)
rFun  = @(w) bezierEval(Pctrl, w);
drFun = @(w) bezierDeriv(Pctrl, w);
% 
% rFun = @(w) [4*w, sin(2*pi*w)];
% drFun = @(w) [4, 2*pi*cos(2*pi*w)];

% Options
opts = struct();
opts.uRange = [0, 1];
opts.nU = 400;
opts.nVGrid = 600;
opts.epsV = 1e-8;
opts.tolDen = 1e-10;

% Compute envelope
env = fixedChordEnvelope(rFun, drFun, L, opts);

%% Plot result
figure('Color', 'w'); hold on; grid on; axis equal;
title('Fixed-length chord envelope on a cubic Bezier curve');
xlabel('x'); ylabel('y');

% Plot Bezier control polygon
plot(Pctrl(:,1), Pctrl(:,2), 'k--o', ...
    'LineWidth', 1.0, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Bezier control polygon');

% Plot Bezier curve
wPlot = linspace(0, 1, 1000);
Rplot = zeros(numel(wPlot), 2);
for i = 1:numel(wPlot)
    Rplot(i, :) = rFun(wPlot(i));
end

plot(Rplot(:,1), Rplot(:,2), 'k-', ...
    'LineWidth', 2.2, ...
    'DisplayName', 'Bezier curve r(w)');

% Draw some fixed-length chords
idxValid = find(env.validLine);
skip = max(1, floor(numel(idxValid) / 50));

for ii = 1:skip:numel(idxValid)
    k = idxValid(ii);
    plot([env.M(k,1), env.N(k,1)], ...
         [env.M(k,2), env.N(k,2)], ...
         '-', ...
         'Color', [0.75, 0.75, 0.75], ...
         'LineWidth', 0.8, ...
         'HandleVisibility', 'off');
end

% M trajectory
plot(env.M(env.validLine,1), env.M(env.validLine,2), ...
    'b-', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'M(u)=r(u)');

% N trajectory
plot(env.N(env.validLine,1), env.N(env.validLine,2), ...
    'g-', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'N(u)=r(v(u))');

% Envelope of extended line family
plot(env.G(env.validLine,1), env.G(env.validLine,2), ...
    'm--', ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Envelope of extended lines');

% True envelope points on finite segment
plot(env.G(env.validSegment,1), env.G(env.validSegment,2), ...
    'r-', ...
    'LineWidth', 2.6, ...
    'DisplayName', 'Envelope of finite segments');

% Points outside finite segment
badEnv = env.validLine & ~env.validSegment;
scatter(env.G(badEnv,1), env.G(badEnv,2), ...
    16, ...
    'm', ...
    'filled', ...
    'DisplayName', 'Line envelope outside segment');

legend('Location', 'bestoutside');

fprintf('Total u samples: %d\n', numel(env.u));
fprintf('Valid fixed-length chords: %d\n', sum(env.validLine));
fprintf('Envelope points on finite segments: %d\n', sum(env.validSegment));
fprintf('Mean chord residual: %.3e\n', mean(abs(env.residual(env.validLine)), 'omitnan'));
fprintf('Max chord residual : %.3e\n', max(abs(env.residual(env.validLine)), [], 'omitnan'));

%% Optional animation
doAnimation = true;

if doAnimation && ~isempty(idxValid)
    figure('Color', 'w'); hold on; grid on; axis equal;
    title('Moving fixed-length chord and envelope point');
    xlabel('x'); ylabel('y');

    plot(Pctrl(:,1), Pctrl(:,2), 'k--o', ...
        'LineWidth', 1.0, ...
        'MarkerFaceColor', 'w');

    plot(Rplot(:,1), Rplot(:,2), 'k-', ...
        'LineWidth', 2.2);

    plot(env.G(env.validSegment,1), env.G(env.validSegment,2), ...
        'r-', ...
        'LineWidth', 2.4);

    hSeg = plot(nan, nan, 'b-', 'LineWidth', 2.2);
    hM = plot(nan, nan, 'bo', 'MarkerFaceColor', 'b');
    hN = plot(nan, nan, 'go', 'MarkerFaceColor', 'g');
    hG = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);

    legend({'Control polygon', ...
            'Bezier curve', ...
            'Finite-segment envelope', ...
            'Moving chord MN', ...
            'M', ...
            'N', ...
            'Envelope contact'}, ...
            'Location', 'bestoutside');

    for ii = 1:3:numel(idxValid)
        k = idxValid(ii);

        set(hSeg, ...
            'XData', [env.M(k,1), env.N(k,1)], ...
            'YData', [env.M(k,2), env.N(k,2)]);

        set(hM, ...
            'XData', env.M(k,1), ...
            'YData', env.M(k,2));

        set(hN, ...
            'XData', env.N(k,1), ...
            'YData', env.N(k,2));

        if env.validSegment(k)
            set(hG, ...
                'XData', env.G(k,1), ...
                'YData', env.G(k,2));
        else
            set(hG, ...
                'XData', nan, ...
                'YData', nan);
        end

        drawnow;
        pause(0.015);
    end
end

%% ============================================================
% Local Bezier functions
%% ============================================================

function X = bezierEval(P, t)
    % Evaluate Bezier curve at scalar parameter t.
    % P is (n+1)-by-2.
    n = size(P, 1) - 1;
    X = zeros(1, 2);

    for i = 0:n
        B = nchoosek(n, i) * (1 - t)^(n - i) * t^i;
        X = X + B * P(i + 1, :);
    end
end

function Xd = bezierDeriv(P, t)
    % Evaluate derivative of Bezier curve at scalar parameter t.
    n = size(P, 1) - 1;
    Pd = n * (P(2:end, :) - P(1:end-1, :));
    Xd = bezierEval(Pd, t);
end