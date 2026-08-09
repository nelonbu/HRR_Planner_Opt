clear; clc; close all;

%% Demo 3: CSSC path to equal-link FTL robot simulation
% Full pipeline:
%   environment -> RRT path -> shortcut/B-spline initialization ->
%   CSSC optimization -> B-spline path to robot joint parameters ->
%   equal-link robot reconstruction and visualization.
%
% The robot is a planar chain of equal-length links. The link length equals
% the CSSC fixed chord length L. Each link is drawn as a rectangle, and
% hinge joints are drawn as circles.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeDemoConfig(projectRoot);
paramsOpt = makeOptimizationParams(cfg);

if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

fprintf('\n[demo3 robot following simulation]\n');
fprintf('  output dir: %s\n', cfg.outDir);

%% 1. Environment
[obstacles, envInfo] = makeDemoEnvironment(cfg);
envInfo.obstacles = obstacles;
fprintf('  environment : %s / %s, obstacles=%d\n', cfg.envMode, cfg.sceneType, numel(obstacles));

%% 2. RRT frontend and B-spline initialization
rrtOpts = makeRRTOpts(cfg, envInfo);
[pathRRT, rrtInfo] = runRRTWithRestarts(envInfo, rrtOpts, cfg.numRRTStart);
if ~rrtInfo.success
    error('RRT failed: %s', rrtInfo.message);
end

shortcutOpts = rrtOpts;
shortcutOpts.seed = cfg.shortcutSeed;
shortcutOpts.numShortcut = cfg.numShortcut;
[pathShort, shortcutInfo] = shortcutPath2D(pathRRT, obstacles, shortcutOpts);

approxLen = polylineLengthLocal(pathShort);
nCtrl = max(paramsOpt.degree + 1, ceil(approxLen / (0.5 * paramsOpt.L)) + 1);
[Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, ...
    struct('degree', paramsOpt.degree, 'nCtrl', nCtrl));

paramsOpt.knot = splineInfo.knot;
Pref = Pinit;

%% 3. CSSC optimization
fprintf('  RRT nodes    : %d\n', rrtInfo.numNodes);
fprintf('  shortcut     : %d -> %d nodes, accepted=%d\n', ...
    size(pathRRT,1), size(pathShort,1), shortcutInfo.numAccepted);
fprintf('  B-spline ctrl: %d\n', size(Pinit,1));

optLog = evalc('[Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, paramsOpt);'); %#ok<NASGU>

paramsEval = makeHighPrecisionEvalParams(paramsOpt);
metricsInit = evaluateCSSCHighPrecision(Pinit, obstacles, paramsEval);
metricsOpt = evaluateCSSCHighPrecision(Popt, obstacles, paramsEval);

fprintf('  init minClear: %.5f\n', metricsInit.minClear);
fprintf('  opt  minClear: %.5f\n', metricsOpt.minClear);
fprintf('  stop reason  : %s, bestIter=%d\n', optInfo.stopReason, optInfo.bestIter);

%% 4. Robot parameter conversion and reconstruction
robot = makeFTLRobot2D(paramsOpt.L, cfg.numRobotLinks, cfg.robot);

motionOpts = struct();
motionOpts.advanceStep = cfg.advanceStep;
motionOpts.pathSampleN = cfg.motionPathSampleN;
motionOpts.goalTol = cfg.robotGoalTol;
motionOpts.entryDirection = [1, 0];
motion = bsplinePathToFTLRobotMotion2D(Popt, paramsOpt, robot, motionOpts);

% Sanity check: reconstruct one middle frame from q and root position.
midId = max(1, round(size(motion.joints, 1) / 2));
Jmid = squeeze(motion.joints(midId,:,:));
Jrec = reconstructFTLRobot2D(motion.q(midId,:), motion.rootPositions(midId,:), robot);
reconErr = max(vecnorm(Jmid - Jrec, 2, 2));
fprintf('  robot frames : %d\n', size(motion.joints, 1));
fprintf('  robot stop   : %s\n', motion.stopReason);
if motion.failed
    fprintf('  robot warn   : %s\n', motion.failMessage);
end
fprintf('  recon error  : %.3e\n', reconErr);

%% 5. Visualization and save
animOpts = struct();
animOpts.outputDir = cfg.outDir;
animOpts.saveVideo = true;
animOpts.saveFramesImage = true;
animOpts.visible = cfg.figureVisible;
animOpts.fps = cfg.videoFPS;
animOpts.frameStride = cfg.frameStride;
animOpts.numMontageFrames = cfg.numMontageFrames;
renderInfo = animateFTLRobotMotion2D(motion, robot, obstacles, animOpts);
renderInfo.figure = [];
renderInfo.jointAngleFigurePath = plotRobotJointAngles2D(motion, cfg);

save(fullfile(cfg.outDir, 'result.mat'), ...
    'cfg', 'envInfo', 'obstacles', 'pathRRT', 'pathShort', ...
    'Pinit', 'Popt', 'Pref', 'paramsOpt', 'paramsEval', ...
    'rrtInfo', 'shortcutInfo', 'optInfo', ...
    'metricsInit', 'metricsOpt', 'robot', 'motion', 'renderInfo');

fprintf('[saved] %s\n', cfg.outDir);

%% Configuration

function cfg = makeDemoConfig(projectRoot)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_robot_sim_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'robot_sim', cfg.runName);

    cfg.envSeed = 1603;

    % Environment switch:
    %   structured sceneType: singleSlit, offsetDoubleSlit, fourRectSChannel, staggeredBaffles3
    %   random sceneType    : randomCircles, randomRects, randomMixed
    cfg.envMode = 'structured';
    cfg.sceneType = 'fourRectSChannel';
    cfg.sceneOpts = struct();
    level = cfg.representativeDifficultyIndex;
    cfg.sceneOpts.dGap = cfg.sChannel.dGap(level);
    cfg.sceneOpts.xWalls = cfg.doubleSlit.xWalls;
    cfg.sceneOpts.gapCentersY = [-0.12, 0.14];
    cfg.sceneOpts.gapHeight = cfg.doubleSlit.dGap(level);
    cfg.sceneOpts.wallThickness = cfg.doubleSlit.wallThickness;
    cfg.rrtSeed = 42000;
    cfg.numRRTStart = 8;
    cfg.shortcutSeed = 52000;

    % The robot demo keeps a larger retry budget than batch experiments.
    cfg.maxIter = 8000;

    cfg.numRobotLinks = 12;
    cfg.advanceStep = 0.005;  % Must divide paramsOpt.L=0.15.
    cfg.motionPathSampleN = 1800;
    cfg.robotGoalTol = 2e-3;

    cfg.figureVisible = 'on';
    cfg.videoFPS = 12;
    cfg.frameStride = 2;
    cfg.numMontageFrames = 8;
    cfg.saveJointAngleFigure = true;
    cfg.angleFigureVisible = 'off';
    cfg.robot = struct();
    cfg.robot.bodyWidth = 0.040;
    cfg.robot.jointRadius = 0.018;
    cfg.robot.drawTipJoint = false;
end

function params = makeOptimizationParams(cfg)
    params = struct();
    params.L = cfg.L;
    params.dMin = cfg.dMin;
    params.dPref = cfg.dPref;
    params.degree = cfg.degree;

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = 120;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;

    params.wObs = 60000.0;
    params.wClear = 100.0;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0.0;
    params.wCurv = 0.0;
    params.kappaMax = 2.0;

    params.solver = struct();
    params.solver.gradMode = 'semi-analytic';
    params.numIter = 100;
    params.lr = 0.001;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 100;

    params.activeTopK = 100;
    params.activeClearanceMargin = 0.0125;

    params.stop = struct();
    params.stop.enable = true;
    params.stop.minIter = 10;
    params.stop.window = 8;
    params.stop.tolRelJ = 1e-2;
    params.stop.tolGrad = 1e-1;
    params.stop.tolStep = 1e-3;
    params.stop.patience = 10;
    params.stop.tolBestRel = 1e-4;
    params.stop.requireSafe = true;
    params.stop.clearanceMargin = 0.0;

    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

function paramsEval = makeHighPrecisionEvalParams(paramsOpt)
    paramsEval = paramsOpt;
    paramsEval.envOpts.nU = 480;
    paramsEval.enablePathSample = true;
    paramsEval.pathSampleN = 1200;
    paramsEval.enablePointClearance = false;
    paramsEval.enableObstacleMetadata = true;
    paramsEval.printEvalTiming = false;
    paramsEval.pointClearanceResolution = 0.001;
end

function [obstacles, envInfo] = makeDemoEnvironment(cfg)
    opts = cfg.sceneOpts;
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    if ~isfield(opts, 'seed') || isempty(opts.seed)
        opts.seed = cfg.envSeed;
    end

    switch lower(char(cfg.envMode))
        case {'structured','structure','scene'}
            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(cfg.sceneType, opts);
        case {'random','randomized'}
            [obstacles, envInfo] = generateCSSCEnvironment2D(cfg.sceneType, opts);
        otherwise
            error('Unknown cfg.envMode: %s. Use structured or random.', char(cfg.envMode));
    end
end

function outPath = plotRobotJointAngles2D(motion, cfg)
    outPath = "";
    if ~isfield(cfg, 'saveJointAngleFigure') || ~cfg.saveJointAngleFigure
        return;
    end

    if ~exist(cfg.outDir, 'dir')
        mkdir(cfg.outDir);
    end

    t = motion.feedDistance(:);
    if isempty(t) || all(~isfinite(t))
        t = (0:(size(motion.q, 1)-1)).';
    end

    relDeg = motion.jointAnglesRel * 180 / pi;
    nRel = size(relDeg, 2);

    visible = 'off';
    if isfield(cfg, 'angleFigureVisible')
        visible = cfg.angleFigureVisible;
    end

    fig = figure('Color','w', 'Name','Robot relative joint angles', ...
        'Visible', visible, 'Position',[100 100 980 520]);
    ax = axes(fig);
    hold(ax, 'on');

    colors = lines(max(nRel, 1));
    for i = 1:nRel
        plot(ax, t, relDeg(:,i), 'LineWidth', 1.2, ...
            'Color', colors(i,:), 'DisplayName', sprintf('joint %d', i));
    end

    grid(ax, 'on');
    xlabel(ax, 'feed distance');
    ylabel(ax, 'relative angle (deg)');
    title(ax, 'Relative hinge angles');
    if nRel > 0
        legend(ax, 'Location','eastoutside');
    else
        text(ax, 0.5, 0.5, 'No relative hinge angle for a single-link robot.', ...
            'Units','normalized', 'HorizontalAlignment','center');
    end

    outPath = fullfile(cfg.outDir, 'robot_joint_angles.jpg');
    exportgraphics(fig, outPath, 'Resolution', 300, 'BackgroundColor','white');
    close(fig);
    outPath = string(outPath);
end

function opts = makeRRTOpts(cfg, envInfo)
    opts = struct();
    opts.bounds = envInfo.bounds;
    opts.stepSize = cfg.stepSize;
    opts.goalBias = cfg.goalBias;
    opts.goalTol = cfg.goalTol;
    opts.maxIter = cfg.maxIter;
    opts.collisionResolution = cfg.collisionResolution;
    opts.inflateRadius = cfg.inflateRadius;
    opts.seed = cfg.rrtSeed;
end

function [path, info] = runRRTWithRestarts(envInfo, opts, numRestart)
    path = zeros(0, 2);
    info = struct('success', false, 'message', 'RRT failed.', ...
        'numIter', 0, 'numNodes', 0, 'numRestartUsed', 0);

    for k = 1:numRestart
        optsK = opts;
        optsK.seed = opts.seed + k - 1;
        [pathK, infoK] = planRRT2D(envInfo.startPt, envInfo.goalPt, ...
            envInfo.obstacles, optsK);
        infoK.numRestartUsed = k;
        info = infoK;
        if infoK.success
            path = pathK;
            return;
        end
    end
end

function len = polylineLengthLocal(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end









