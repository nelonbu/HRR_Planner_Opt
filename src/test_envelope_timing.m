clear; clc; close all;

%% Test fixedChordEnvelope local-Newton version with a cubic Bezier curve

Pctrl = [
    0.0,  0.0;
    1.2,  2.0;
    3.0, -1.2;
    4.5,  1.0
];

L = 1.20;

rFun  = @(w) bezierEval(Pctrl, w);
drFun = @(w) bezierDeriv(Pctrl, w);

opts = struct();
opts.uRange = [0, 1];
opts.nU = 120;
opts.epsV = 1e-8;
opts.tolDen = 1e-10;
opts.enableTiming = true;

% Local Newton options.
opts.rootTol = 1e-6;
opts.newtonMaxIter = 10;
opts.newtonMaxStep = NaN;      % NaN -> fixedChordEnvelope uses 5*mean(du)
opts.newtonDamping = true;
opts.maxDampingIter = 6;
opts.dampingAcceptRatio = 0.9;

fprintf('\n===== Running fixedChordEnvelope local-Newton timing test =====\n');
env = fixedChordEnvelope(rFun, drFun, L, opts);

validLine = env.validLine;
validSeg = env.validSegment;

fprintf('\nMean chord residual: %.3e\n', ...
    mean(abs(env.residual(validLine)), 'omitnan'));

fprintf('Max chord residual : %.3e\n', ...
    max(abs(env.residual(validLine)), [], 'omitnan'));

fprintf('Valid line ratio   : %.1f%%\n', 100 * sum(validLine) / numel(validLine));
fprintf('Valid seg ratio    : %.1f%%\n', 100 * sum(validSeg) / numel(validSeg));

%% Plot result
figure('Color', 'w'); hold on; grid on; axis equal;
title('fixedChordEnvelope local Newton test');
xlabel('x'); ylabel('y');

wPlot = linspace(0, 1, 800);
Rplot = zeros(numel(wPlot), 2);
for i = 1:numel(wPlot)
    Rplot(i, :) = rFun(wPlot(i));
end

plot(Pctrl(:,1), Pctrl(:,2), 'k--o', ...
    'LineWidth', 1.0, ...
    'MarkerFaceColor', 'w', ...
    'DisplayName', 'Control polygon');

plot(Rplot(:,1), Rplot(:,2), 'k-', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Bezier curve');

% Draw some fixed-length chords.
idx = find(validLine);
skip = max(1, floor(numel(idx) / 40));

for ii = 1:skip:numel(idx)
    k = idx(ii);
    plot([env.M(k,1), env.N(k,1)], ...
         [env.M(k,2), env.N(k,2)], ...
         '-', ...
         'Color', [0.75 0.75 0.75], ...
         'LineWidth', 0.8, ...
         'HandleVisibility', 'off');
end

plot(env.M(validLine,1), env.M(validLine,2), ...
    'b-', ...
    'LineWidth', 1.2, ...
    'DisplayName', 'M(u)');

plot(env.N(validLine,1), env.N(validLine,2), ...
    'g-', ...
    'LineWidth', 1.2, ...
    'DisplayName', 'N(u)');

plot(env.G(validLine,1), env.G(validLine,2), ...
    'm--', ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Envelope of extended lines');

plot(env.G(validSeg,1), env.G(validSeg,2), ...
    'r-', ...
    'LineWidth', 2.4, ...
    'DisplayName', 'Envelope of finite segments');

% Mark failed samples on the original curve.
failed = ~validLine;
if any(failed)
    scatter(env.u(failed), zeros(sum(failed),1), 10, 'x', ...
        'DisplayName', 'Failed u samples shown in parameter axis proxy');
end

legend('Location', 'bestoutside');

%% ============================================================
% Local Bezier functions
%% ============================================================

function X = bezierEval(P, t)
    t = t(:);
    n = size(P, 1) - 1;

    X = zeros(numel(t), 2);
    for i = 0:n
        B = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i;
        X = X + B * P(i + 1, :);
    end

    if numel(t) == 1
        X = X(1, :);
    end
end

function Xd = bezierDeriv(P, t)
    n = size(P, 1) - 1;
    Pd = n * (P(2:end, :) - P(1:end-1, :));
    Xd = bezierEval(Pd, t);
end
