clear; clc; close all;

%% Main proposed CSSC-FTL 2D demo
% Stable proposed pipeline:
%   B-spline path -> CSSC fixed-chord swept envelope optimization
%   -> semi-analytic gradient -> bestP/patience early stop
%
% Final reported success, minClear, and dMin satisfaction are evaluated by
% evaluateCSSCHighPrecision, which is not used inside the optimizer.

try
    projectRoot = initCSSCProjectPath;
catch
    % If this script is already under a path-managed session, continue.
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
commonCfg = getCSSCDemoConfig2D();

%% 1. Environment: obstacles as unified structs
obstacles = [
    obsRect2D([0.50,  0.08], [0.10, 0.125], 0.0)
    obsRect2D([0.50, -0.24], [0.10, 0.125], 0.0)
];

%% 2. FTL segment parameters
params = struct();
params.L = commonCfg.L;
params.dMin = commonCfg.dMin;
params.dPref = commonCfg.dPref;

%% 3. B-spline path
params.degree = commonCfg.degree;
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

% Clearance backend:
%   'segment'  : conservative default, exact segment-obstacle clearance.
%   'envelope' : fast G/M/N point probe only, approximate.
%   'hybrid'   : G/M/N probe plus selective exact segment refinement.
params.clearanceMode = 'segment';
params.hybrid.triggerFactor = 2.0;
params.hybrid.refineActiveTopK = true;
params.hybrid.forceExactStride = 0;

% Run mode:
%   'formal' : fast optimization mode. Disables timing spam, diagnostic
%              point clearances, optimization-time path sampling, and
%              process video rendering. Console prints key iteration lines.
%   'debug'  : diagnostic mode. Enables detailed timing, M/N/G point SDF
%              clearances, optimization-time path sampling, and process
%              image/video output for checking intermediate behavior.
params.runMode = 'formal';  % 'formal' for fast runs, 'debug' for detailed diagnostics.

params.numIter = 200;
params.lr = 0.001;
params.fdStep = 1e-5;
params.gradClip = 5.0;
params.printInterval = 10;
params.saveInterval = 20;

params.activeTopK = 100;
params.activeClearanceMargin = 0.0125;

params.timingPrintInterval = 10;
params.timingPrintWindow = 5;

%% 7. Output / plotting settings
switch lower(params.runMode)
    case 'formal'
        params.printEvalTiming = false;
        params.enableTimingDebug = false;
        params.enablePathSample = false;
        params.enablePointClearance = false;
        params.enableObstacleMetadata = false;
        plotProcessDefault = false;

    case 'debug'
        params.printEvalTiming = true;
        params.enableTimingDebug = true;
        params.enablePathSample = true;
        params.enablePointClearance = true;
        params.enableObstacleMetadata = true;
        plotProcessDefault = true;

    otherwise
        error('Unknown params.runMode: %s', params.runMode);
end

params.output = struct();
params.output.enable = true;
params.output.mode = 'tmp';        % 'tmp' or 'formal'
params.output.rootDir = fullfile(projectRoot, 'results');

params.plot = struct();
params.plot.outputDir = '';
params.plot.saveFinalFigure = true;
params.plot.showProcess = plotProcessDefault;
params.plot.showProcessFigure = plotProcessDefault;
params.plot.saveProcessImage = plotProcessDefault;
params.plot.saveProcessVideo = plotProcessDefault;
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

paramsEval = params;
paramsEval.envOpts.nU = 480;
paramsEval.pathSampleN = 1200;
paramsEval.pointClearanceResolution = 0.001;
paramsEval.enablePathSample = true;
paramsEval.enablePointClearance = false;
paramsEval.enableObstacleMetadata = true;

tic;
highPrecisionInit = evaluateCSSCHighPrecision(Pinit, obstacles, paramsEval);
fprintf('Initial high-precision clearance = %.6f\n', highPrecisionInit.minClear);

[Popt, info] = optimizeCSSC2D(Pinit, Pref, obstacles, params);

highPrecisionMetrics = evaluateCSSCHighPrecision(Popt, obstacles, paramsEval);

fprintf('\nFinal objective: %.6g\n', info.finalJ);
fprintf('Final high-precision clearance: %.4f\n', highPrecisionMetrics.minClear);
fprintf('Required clearance dMin: %.4f\n', params.dMin);

if output.enable
    paths = struct('Pinit', Pinit, 'Popt', Popt, 'Pref', Pref, ...
        'pathRRT', [], 'pathBSplineInit', highPrecisionInit.pathSample, ...
        'pathOptimized', highPrecisionMetrics.pathSample);
    timing = struct('highPrecisionInit', highPrecisionInit.timing, ...
        'highPrecisionFinal', highPrecisionMetrics.timing);
    successFlags = struct( ...
        'plannerSuccess', true, ...
        'highPrecisionSuccess', highPrecisionMetrics.success, ...
        'dMinSatisfied', highPrecisionMetrics.dMinSatisfied, ...
        'pointSuccess', highPrecisionMetrics.pointSuccess);
    seed = struct('envSeed', nan, 'plannerSeed', nan, 'optimizerSeed', nan);
    result = makeCSSCExperimentResult(params, seed, obstacles, paths, ...
        info, highPrecisionMetrics, timing, successFlags);
    result.highPrecisionInit = highPrecisionInit;

    save(fullfile(output.dir, 'result.mat'), 'result');
end
toc

%% 9. Plot
plotCSSCResult2D(Pinit, Popt, obstacles, params, info);
