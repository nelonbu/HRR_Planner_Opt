clear; clc; close all;

%% CSSC-FTL 2D demo with selectable gradient mode
% Required first if not already on path:
%   initCSSCProjectPath;

try
    projectRoot = initCSSCProjectPath;
catch
    % If this script is already under a path-managed session, continue.
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end

%% 1. Environment: obstacles as unified structs
obstacles = [
    obsRect2D([0.50,  0.08], [0.10, 0.125], 0.0)
    obsRect2D([0.50, -0.24], [0.10, 0.125], 0.0)
];

%% 2. FTL segment parameters
params = struct();
params.L = 0.15;
params.dMin = 0.02;
params.dPref = 0.03;

%% 3. B-spline path
params.degree = 3;
startPt = [0.0, 0.0];
goalPt  = [1.0, 0.0];

pathLenApprox = norm(goalPt - startPt);
hCtrl = 0.5 * params.L;
nCtrl = max(params.degree + 1, ceil(pathLenApprox / hCtrl) + 1);

x = linspace(startPt(1), goalPt(1), nCtrl).';
y = 0.015 * sin(2*pi*(x - x(1)) / (x(end) - x(1)));
Pinit = [x, y];
Pinit(1,:) = startPt;
Pinit(end,:) = goalPt;
Pref = Pinit;

% Shared knot vector.
params.knot = makeClampedUniformKnot(size(Pinit,1), params.degree);

%% 4. Envelope options
params.envOpts = struct();
params.envOpts.uRange = [0, 1];
params.envOpts.vSearchRange = [0, 1];
params.envOpts.nU = 140;
% params.envOpts.nVGrid = 180;
params.envOpts.epsV = 1e-6;
params.envOpts.tolDen = 1e-6;

%% 5. Objective weights
params.wObs = 600.0;
params.wClear = 100.0;
params.wRef = 0.01;
params.wSmooth = 0.5;
params.wLength = 0.002;
params.wTrust = 0.0;

% Curvature is deliberately disabled in the first semi-analytic version.
params.wCurv = 0.0;
params.kappaMax = 2.0;

%% 6. Solver settings
params.solver = struct();
params.solver.gradMode = 'semi-analytic';
% params.solver.gradMode = 'finite-diff';

params.numIter = 200;
params.lr = 0.001;
params.fdStep = 1e-5;
params.gradClip = 5.0;
params.printInterval = 10;
params.saveInterval = 20;

params.activeTopK = 100;
params.activeClearanceMargin = 0.0125;

params.enableTimingDebug = true;
params.timingPrintInterval = 10;
params.timingPrintWindow = 5;

%% 7. Output / plotting settings
params.enablePathSample = false;   % Plotting functions turn this on when needed.

params.output = struct();
params.output.enable = true;
params.output.mode = 'tmp';        % 'tmp' or 'formal'
params.output.rootDir = fullfile(projectRoot, 'results');

params.plot = struct();
params.plot.outputDir = '';
params.plot.saveFinalFigure = true;
params.plot.showProcess = true;
params.plot.showProcessFigure = true;
params.plot.saveProcessImage = true;
params.plot.saveProcessVideo = true;
params.plot.maxProcessCurves = 25;
params.plot.videoFPS = 8;
params.plot.videoQuality = 95;
params.plot.videoDrawChords = true;
params.plot.videoDrawMinClear = true;

output = prepareCSSCOutput(params);
if output.enable
    params.plot.outputDir = output.dir;
    fprintf('[output] %s\n', output.dir);
end

%% 8. Optimize
fprintf('nCtrl = %d, optimized variables = %d\n', nCtrl, 2*(nCtrl-2));

state0 = evaluateCSSCGlobal(Pinit, obstacles, params);
fprintf('Initial minClear = %.6f\n', state0.minClear);

[Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params);

fprintf('\nFinal objective: %.6g\n', info.finalJ);
fprintf('Final minimum swept-segment clearance: %.4f\n', info.finalMinClear);
fprintf('Required clearance dMin: %.4f\n', params.dMin);

if output.enable
    save(fullfile(output.dir, 'result.mat'), ...
        'Pinit', 'Popt', 'Pref', 'obstacles', 'params', 'info', 'output');
end

%% 9. Plot
plotCSSCResult2D(Pinit, Popt, obstacles, params, info);
