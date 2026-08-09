clear; clc; close all;

%% Debug: rectangle SDF field and segment clearance
% This small script visualizes the signed-distance field of one rotated
% rectangle and overlays the segment-clearance result used by
% querySegmentObstacleClearance2D.

try
    initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

%% 1. Rectangle obstacle and one test segment
obs = obsRect2D([0.0, 0.0], [0.28, 0.13], pi/8);
obstacles = obs;

M = [-0.55, -0.18];
N = [ 0.55,  -0.46];

segOut = querySegmentObstacleClearance2D(M, N, obstacles);

%% 2. Point SDF field
x = linspace(-0.70, 0.70, 220);
y = linspace(-0.48, 0.48, 180);
[X, Y] = meshgrid(x, y);
D = nan(size(X));

for i = 1:numel(X)
    D(i) = queryObstaclePointSDF2D([X(i), Y(i)], obstacles);
end

%% 3. Plot
fig = figure('Color','w', 'Name','Rectangle SDF and segment clearance');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
box(ax, 'on');

contourf(ax, X, Y, D, 36, 'LineStyle','none');
colormap(ax, turbo);
cb = colorbar(ax);
cb.Label.String = 'signed clearance / SDF';

contour(ax, X, Y, D, [0, 0], 'k-', 'LineWidth', 2.0);
contour(ax, X, Y, D, [-0.05, 0.05, 0.10, 0.20], ...
    'LineColor', [0.15, 0.15, 0.15], 'LineStyle','--', 'LineWidth', 0.8);

V = obs.vertices;
patch(ax, V(:,1), V(:,2), [0.20, 0.20, 0.20], ...
    'FaceAlpha', 0.22, 'EdgeColor','k', 'LineWidth', 1.5);

plot(ax, [M(1), N(1)], [M(2), N(2)], 'w-', 'LineWidth', 3.0);
plot(ax, [M(1), N(1)], [M(2), N(2)], 'b-', 'LineWidth', 1.4);
plot(ax, M(1), M(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize', 6);
plot(ax, N(1), N(2), 'bs', 'MarkerFaceColor','b', 'MarkerSize', 6);

q = segOut.closestPoint;
n = segOut.normal;
plot(ax, q(1), q(2), 'rp', 'MarkerFaceColor','r', ...
    'MarkerSize', 13, 'LineWidth', 1.5);
quiver(ax, q(1), q(2), 0.12*n(1), 0.12*n(2), 0, ...
    'Color','r', 'LineWidth', 1.6, 'MaxHeadSize', 1.8);

title(ax, sprintf('rect segment clearance = %.5f, alpha = %.3f', ...
    segOut.clearance, segOut.alpha));
xlabel(ax, 'x');
ylabel(ax, 'y');
xlim(ax, [min(x), max(x)]);
ylim(ax, [min(y), max(y)]);

fprintf('[rect SDF debug]\n');
fprintf('  segment clearance : %.6f\n', segOut.clearance);
fprintf('  alpha             : %.6f\n', segOut.alpha);
fprintf('  closest point     : [%.6f, %.6f]\n', q(1), q(2));
fprintf('  normal            : [%.6f, %.6f]\n', n(1), n(2));
