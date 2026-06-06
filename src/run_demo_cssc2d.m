clear; clc; close all;

%% ============================================================
% CSSC-FTL 2D MVP demo
%
% Required files in the same folder:
%   fixedChordEnvelope.m       % your existing file
%   evalBSplinePath2D.m
%   objectiveCSSC2D.m
%   optimizeCSSC2D.m
%   plotCSSCResult2D.m
%% ============================================================

%% 1. Environment: circular obstacles [cx, cy, radius]
obstacles = [
    2.2,  0.42, 0.38;
    3.2, -0.45, 0.42;
    4.3,  0.50, 0.42;
    5.3, -0.45, 0.40;
    6.4,  0.38, 0.35
];

%% 2. Robot / FTL segment parameters
params = struct();
params.L = 0.90;               % fixed chord length, terminal link length
params.robotRadius = 0.06;     % radius around the moving segment
params.dMin = 0.03;            % required clearance
params.dPref = 0.22;           % preferred clearance buffer

%% 3. Path representation: clamped cubic B-spline
params.degree = 3;
startPt = [0.0, 0.0];
goalPt  = [8.0, 0.0];

% Dense control points: roughly one control point per link length.
pathLenApprox = norm(goalPt - startPt);
hCtrl = 0.75 * params.L;
nCtrl = max(params.degree + 1, ceil(pathLenApprox / hCtrl) + 1);

x = linspace(startPt(1), goalPt(1), nCtrl).';
y = 0.12 * sin(2*pi*(x - x(1)) / (x(end) - x(1)));
Pinit = [x, y];
Pinit(1,:) = startPt;
Pinit(end,:) = goalPt;

Pref = Pinit;

%% 4. Envelope sampling options
params.envOpts = struct();
params.envOpts.uRange = [0, 1];
params.envOpts.nU = 140;
params.envOpts.nVGrid = 180;
params.envOpts.epsV = 1e-8;
params.envOpts.tolDen = 1e-10;

%% 5. Objective weights
params.wObs = 350.0;
params.wClear = 2.0;
params.wRef = 0.02;
params.wSmooth = 0.8;
params.wLength = 0.02;
params.wCurv = 0.05;
params.kappaMax = 2.0;
params.curvSamples = 80;
params.epsObs = 0.02;
params.epsClear = 0.05;

%% 6. Optimizer settings
params.numIter = 20;
params.lr = 0.025;
params.fdStep = 1e-4;
params.gradClip = 100.0;
params.printInterval = 5;
params.saveInterval = 10;



%% 7. Optimize
fprintf('nCtrl = %d, optimized variables = %d\n', nCtrl, 2*(nCtrl-2));
[Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params);

fprintf('\nFinal objective: %.6g\n', info.finalJ);
fprintf('Final minimum swept-segment clearance: %.4f\n', info.finalMinClear);
fprintf('Required clearance dMin: %.4f\n', params.dMin);

%% 8. Plot
plotCSSCResult2D(Pinit, Popt, obstacles, params, info);
