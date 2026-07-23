clear; clc; close all;

%% FIG_CHORD_GEOMETRY Fixed-length chord family and clearance geometry.
% This script creates a publication-style schematic for the relationship
% among a B-spline leader path, its fixed-length chord family, the analytic
% envelope candidate G(u), and segment-wise obstacle clearance.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

%% Figure and geometry parameters
cfg.figureSizeCm = [16.0, 10.0];
cfg.resolutionPPI = 600;
cfg.figureVisible = 'on';
cfg.fontName = 'Times New Roman';
cfg.fontSize = 11;

cfg.degree = 3;
cfg.L = 1.00;
cfg.bodyRadius = 0.04;
cfg.dMin = 0.14;
cfg.numFamilyChords = 40;

cfg.pathColor = [0.05, 0.24, 0.52];
cfg.familyChordColor = [0.68, 0.70, 0.73];
cfg.focusChordColor = [0.05, 0.07, 0.10];
cfg.envelopeColor = [0.78, 0.10, 0.10];
cfg.closestPointColor = [0.62, 0.08, 0.56];
cfg.normalColor = [0.00, 0.52, 0.43];
cfg.dMinColor = [0.92, 0.55, 0.05];
cfg.obstacleColor = [0.75, 0.77, 0.80];

cfg.pathLineWidth = 2.2;
cfg.familyChordLineWidth = 0.42;
cfg.focusChordLineWidth = 2.1;
cfg.envelopeLineWidth = 1.6;
cfg.boundaryLineWidth = 1.25;

%% B-spline leader path and fixed-length chord family
P = [
    0.45, 0.65
    0.9450, 0.5885
    1.4400, 0.8750
    1.8625, 1.4225
    2.5025, 1.6075
    3.1425, 1.4300
    3.7100, 1.5715
    4.1325, 2.0900
    4.6275, 2.4925
    5.05, 2.75
];
knot = makeClampedUniformKnot(size(P,1), cfg.degree);
rFun = @(u) evalBSplinePath2D(P, u, cfg.degree, knot);
drFun = @(u) evaluatePathDerivative(P, u, cfg.degree, knot);

envOpts = struct();
envOpts.uRange = [0, 1];
envOpts.vSearchRange = [0, 1];
envOpts.nU = 320;
envOpts.epsV = 1e-8;
envOpts.tolDen = 1e-10;
envOpts.lambdaTol = 1e-9;
envOpts.maxNewtonIter = 10;
envOpts.vAcceptTol = 1e-7;
envOpts.newtonTriggerTol = 1e-7;
envOpts.vStepTol = 1e-12;
envOpts.maxNewtonStep = 0.15;

env = fixedChordEnvelope(rFun, drFun, cfg.L, envOpts);
focusIdx = selectFocusChord(env);

M = env.M(focusIdx,:);
N = env.N(focusIdx,:);
G = env.G(focusIdx,:);
lambdaG = env.lambda(focusIdx);
chordVector = N - M;
chordLength = norm(chordVector);
chordTangent = chordVector / chordLength;
normalTowardObstacle = [chordTangent(2), -chordTangent(1)];

obstacleRadius = 0.58;
obstacleShift = obstacleRadius / sqrt(2) * [-3, 3];

% Compensate the tangential part of the requested obstacle shift so that
% the final q* remains well inside the chord and separated from G.
if lambdaG <= 0.5
    alphaTarget = min(0.82, lambdaG + 0.35);
else
    alphaTarget = max(0.18, lambdaG - 0.35);
end
shiftAlongChord = dot(obstacleShift, chordVector) / ...
    dot(chordVector, chordVector);
alphaDesign = alphaTarget - shiftAlongChord;
qDesign = M + alphaDesign * chordVector;

%% Circular obstacle with analytically controlled closest geometry
rawSegmentDistance = cfg.bodyRadius + 0.19;
baseObstacleCenter = qDesign + ...
    (rawSegmentDistance + 0.21) * normalTowardObstacle;
obstacleCenter = baseObstacleCenter + obstacleShift;

% Orthogonal projection of the moved circle center onto the focal segment.
alphaStar = dot(obstacleCenter - M, chordVector) / dot(chordVector, chordVector);
alphaStar = min(max(alphaStar, 0), 1);
qStar = M + alphaStar * chordVector;

theta = linspace(0, 2*pi, 480).';
obstacleBoundary = circleBoundary(obstacleCenter, obstacleRadius, theta);
dMinBoundary = circleBoundary(obstacleCenter, ...
    obstacleRadius + cfg.bodyRadius + cfg.dMin, theta);

centerToSegment = qStar - obstacleCenter;
qObstacle = obstacleCenter + ...
    obstacleRadius * centerToSegment / norm(centerToSegment);
nSDF = (qStar - qObstacle) / norm(qStar - qObstacle);
clearanceValue = norm(qStar - qObstacle) - cfg.bodyRadius;

%% Draw
fig = figure( ...
    'Color', 'w', ...
    'Visible', cfg.figureVisible, ...
    'Units', 'centimeters', ...
    'Position', [2, 2, cfg.figureSizeCm], ...
    'PaperUnits', 'centimeters', ...
    'PaperPosition', [0, 0, cfg.figureSizeCm], ...
    'InvertHardcopy', 'off');
ax = axes(fig, 'Position', [0.035, 0.055, 0.93, 0.90]);
hold(ax, 'on');
axis(ax, 'off');
set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize);
xlim(ax, [0.15, 5.35]);
ylim(ax, [-0.05, 3.45]);
axis(ax, 'equal');

% The safety boundary uses gold to remain distinct from the red envelope.
plotClosedCurve(ax, dMinBoundary, '-.', cfg.dMinColor, ...
    cfg.boundaryLineWidth);
patch(ax, obstacleBoundary(:,1), obstacleBoundary(:,2), cfg.obstacleColor, ...
    'FaceAlpha', 0.96, ...
    'EdgeColor', [0.05, 0.05, 0.05], ...
    'LineWidth', 1.25);

% Fixed-length chord family.
familyIdx = selectFamilyChords(env, focusIdx, cfg.numFamilyChords);
for k = 1:numel(familyIdx)
    i = familyIdx(k);
    plot(ax, [env.M(i,1), env.N(i,1)], [env.M(i,2), env.N(i,2)], ...
        '-', ...
        'Color', cfg.familyChordColor, ...
        'LineWidth', cfg.familyChordLineWidth);
end

% Analytic envelope candidate curve G(u).
GCurve = env.G;
validEnvelope = env.validSegment & all(isfinite(GCurve), 2);
GCurve(~validEnvelope,:) = nan;
GCurve = breakLargeCurveJumps(GCurve, 0.23);
plot(ax, GCurve(:,1), GCurve(:,2), '--', ...
    'Color', cfg.envelopeColor, ...
    'LineWidth', cfg.envelopeLineWidth);

% Leader path and representative chord.
uPlot = linspace(0, 1, 700).';
path = rFun(uPlot);
plot(ax, path(:,1), path(:,2), '-', ...
    'Color', cfg.pathColor, ...
    'LineWidth', cfg.pathLineWidth);
plot(ax, [M(1), N(1)], [M(2), N(2)], '-', ...
    'Color', cfg.focusChordColor, ...
    'LineWidth', cfg.focusChordLineWidth);

plot(ax, M(1), M(2), 'o', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', cfg.focusChordColor, ...
    'LineWidth', 1.2);
plot(ax, N(1), N(2), 'o', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', cfg.focusChordColor, ...
    'LineWidth', 1.2);
plot(ax, G(1), G(2), 'o', ...
    'MarkerSize', 7.0, ...
    'MarkerFaceColor', cfg.envelopeColor, ...
    'MarkerEdgeColor', 'w', ...
    'LineWidth', 0.8);
plot(ax, qStar(1), qStar(2), 'o', ...
    'MarkerSize', 7.5, ...
    'MarkerFaceColor', cfg.closestPointColor, ...
    'MarkerEdgeColor', 'w', ...
    'LineWidth', 0.8);

% SDF normal and segment-wise closest geometry.
drawArrow(ax, qObstacle, qStar, cfg.normalColor, 1.55);

%% Export
outDir = fullfile(projectRoot, 'results', 'figures', 'method_geometry');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

pngPath = fullfile(outDir, 'fig_chord_geometry.png');
pdfPath = fullfile(outDir, 'fig_chord_geometry.pdf');
matPath = fullfile(outDir, 'fig_chord_geometry_data.mat');
exportgraphics(fig, pngPath, ...
    'Resolution', cfg.resolutionPPI, ...
    'BackgroundColor', 'white');
exportgraphics(fig, pdfPath, ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'white');
save(matPath, 'cfg', 'P', 'knot', 'env', 'focusIdx', ...
    'M', 'N', 'G', 'lambdaG', 'alphaStar', 'qStar', ...
    'qObstacle', 'nSDF', 'clearanceValue', ...
    'obstacleBoundary', 'dMinBoundary');

fprintf('[fig_chord_geometry]\n');
fprintf('  focus u / v       : %.6f / %.6f\n', env.u(focusIdx), env.v(focusIdx));
fprintf('  chord residual    : %.3e\n', norm(N - M) - cfg.L);
fprintf('  lambda(G) / alpha*: %.4f / %.4f\n', lambdaG, alphaStar);
fprintf('  segment clearance : %.4f\n', clearanceValue);
fprintf('  PNG               : %s\n', pngPath);
fprintf('  PDF               : %s\n', pdfPath);
fprintf('  data              : %s\n', matPath);

%% Local helpers
function derivative = evaluatePathDerivative(P, u, degree, knot)
    [~, derivative] = evalBSplinePath2D(P, u, degree, knot);
end

function idx = selectFocusChord(env)
    candidate = find(env.validSegment & ...
        env.u >= 0.28 & env.u <= 0.62 & ...
        env.lambda >= 0.15 & env.lambda <= 0.85);
    if isempty(candidate)
        candidate = find(env.validSegment);
    end
    if isempty(candidate)
        error('No valid envelope candidate lies on a fixed-length segment.');
    end

    score = abs(env.u(candidate) - 0.43) ...
        + 0.35 * abs(env.lambda(candidate) - 0.38);
    [~, localIdx] = min(score);
    idx = candidate(localIdx);
end

function idx = selectFamilyChords(env, focusIdx, numChords)
    validIdx = find(env.validLine & env.u >= 0.08 & env.u <= 0.76);
    if isempty(validIdx)
        error('No valid fixed-length chords are available.');
    end

    samplePos = round(linspace(1, numel(validIdx), numChords));
    idx = unique([validIdx(samplePos); focusIdx]);
    idx(idx == focusIdx) = [];
end

function points = circleBoundary(center, radius, theta)
    theta = theta(:);
    points = center + radius * [cos(theta), sin(theta)];
end

function curve = breakLargeCurveJumps(curve, threshold)
    jump = vecnorm(diff(curve, 1, 1), 2, 2);
    breakIdx = find(jump > threshold);
    curve(breakIdx + 1,:) = nan;
end

function plotClosedCurve(ax, points, lineStyle, color, lineWidth)
    plot(ax, [points(:,1); points(1,1)], ...
        [points(:,2); points(1,2)], ...
        'LineStyle', lineStyle, ...
        'Color', color, ...
        'LineWidth', lineWidth);
end

function drawArrow(ax, p0, p1, color, lineWidth)
    delta = p1 - p0;
    quiver(ax, p0(1), p0(2), delta(1), delta(2), 0, ...
        'Color', color, ...
        'LineWidth', lineWidth, ...
        'MaxHeadSize', 0.34);
end
