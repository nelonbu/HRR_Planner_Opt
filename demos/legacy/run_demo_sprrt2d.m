clear; clc; close all;

%% Adaptive Sp-RRT-2D single-scene smoke demo
% Runs reverse goal-to-start search with up to 12 fixed-length leader-path
% segments and 40-degree turn limits. nominalLinkCount=6 is robot metadata,
% not a search-depth limit. The optimized polyline is evaluated exactly as
% a degree-1 B-spline by the common high-precision fixed-chord evaluator.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeDemoConfig(projectRoot);
if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

obstacles = obsCircle2D([0.40, 0.12], 0.05);
opts = makePlannerOpts(cfg);
[pathOptimized, plannerInfo] = planAdaptiveSpRRT2D( ...
    cfg.startPt, cfg.goalPt, obstacles, opts);

evalOpts = struct( ...
    'L', cfg.L, ...
    'dMin', cfg.dMin, ...
    'envOpts', struct('nU', 480, 'uRange', [0,1], ...
        'vSearchRange', [0,1], 'epsV', 1e-8, 'tolDen', 1e-10), ...
    'pathSampleN', 1200, ...
    'pointClearanceResolution', 0.001);
tEval = tic;
metrics = evaluateSpRRTPath2D(pathOptimized, obstacles, evalOpts);
evalTimeSec = toc(tEval);

paths = struct( ...
    'Pinit', pathOptimized, ...
    'Popt', pathOptimized, ...
    'Pref', [], ...
    'pathRRT', plannerInfo.rawPath, ...
    'pathRaw', plannerInfo.rawPath, ...
    'pathBSplineInit', [], ...
    'pathOptimized', pathOptimized);
timing = struct( ...
    'planTimeSec', plannerInfo.planningTimeSec, ...
    'pathOptimizationTimeSec', plannerInfo.pathOptimizationTimeSec, ...
    'evalTimeSec', evalTimeSec);
flags = struct( ...
    'plannerSuccess', plannerInfo.success, ...
    'highPrecisionSuccess', metrics.success, ...
    'dMinSatisfied', metrics.dMinSatisfied, ...
    'wholeBodySuccess', metrics.dMinSatisfied, ...
    'pointSuccess', metrics.pointSuccess);
seed = struct('plannerSeed', cfg.plannerSeed);
result = makeCSSCExperimentResult( ...
    cfg, seed, obstacles, paths, plannerInfo, metrics, timing, flags);

matPath = fullfile(cfg.outDir, 'result.mat');
figPath = fullfile(cfg.outDir, 'sprrt2d_result.jpg');
save(matPath, 'result');
drawResult(cfg, obstacles, plannerInfo, metrics, figPath);

fprintf('\n[Adaptive Sp-RRT-2D]\n');
fprintf('  planner success      : %d\n', plannerInfo.success);
fprintf('  termination          : %s\n', plannerInfo.terminationReason);
fprintf('  iterations / nodes   : %d / %d\n', ...
    plannerInfo.numIter, plannerInfo.numNodes);
fprintf('  abandoned candidates : %d\n', ...
    plannerInfo.abandonedCandidateCount);
fprintf('  raw / optimized len  : %.5f / %.5f\n', ...
    plannerInfo.rawPathLength, plannerInfo.optimizedPathLength);
fprintf('  nominal / max segments: %d / %d\n', ...
    plannerInfo.nominalLinkCount, plannerInfo.maxSegmentCount);
fprintf('  raw segments / extra : %d / %d\n', ...
    plannerInfo.finalRawSegmentCount, ...
    plannerInfo.numberOfExtraSegments);
fprintf('  direct / nominal / max length: %.5f / %.5f / %.5f\n', ...
    plannerInfo.directDistance, plannerInfo.nominalTotalLength, ...
    plannerInfo.maximumAllowedLength);
fprintf('  maximum depth reached: %d\n', plannerInfo.maxDepthReached);
fprintf('  fixed-chord clearance: %.5f\n', metrics.minClear);
fprintf('  dMin satisfied       : %d\n', metrics.dMinSatisfied);
if strlength(string(plannerInfo.warningMessage)) > 0
    fprintf('  structural warning   : %s\n', plannerInfo.warningMessage);
end
fprintf('  result               : %s\n', matPath);
fprintf('  figure               : %s\n', figPath);

function cfg = makeDemoConfig(projectRoot)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_sprrt2d_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    % This smoke case intentionally uses a longer workspace than the
    % common benchmark while retaining shared robot/planner parameters.
    cfg.bounds = [0, 1.4; -0.3, 0.3];
    cfg.startPt = [0.10, 0.0];
    cfg.goalPt = [1.30, 0.0];
    cfg.plannerSeed = 42021;
    cfg.figureVisible = 'on';
end

function opts = makePlannerOpts(cfg)
    opts = defaultSpRRT2DParams();
    opts.L = cfg.L;
    opts.nominalLinkCount = cfg.sprrtNominalLinkCount;
    opts.maxSegmentCount = cfg.sprrtMaxSegmentCount;
    opts.thetaMax = deg2rad(cfg.sprrtThetaMaxDeg);
    opts.bounds = cfg.bounds;
    opts.seed = cfg.plannerSeed;
    opts.maxIter = cfg.maxIter;
    opts.maxTimeSec = cfg.sprrtMaxTimeSec;
    opts.entranceBias = cfg.sprrtEntranceBias;
    opts.collisionResolution = cfg.collisionResolution;
    opts.inflateRadius = cfg.inflateRadius;
    opts.pathOptimization.mode = cfg.sprrtPathOptimizationMode;
    opts.pathOptimization.thetaMax = opts.thetaMax;
    opts.emitWarnings = true;
end

function drawResult(cfg, obstacles, plannerInfo, metrics, filePath)
    fig = figure('Color', 'w', 'Visible', cfg.figureVisible, ...
        'Position', [100, 100, 900, 500]);
    ax = axes(fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlim(ax, cfg.bounds(1,:));
    ylim(ax, cfg.bounds(2,:));

    theta = linspace(0, 2*pi, 120);
    patch(ax, obstacles.center(1) + obstacles.radius*cos(theta), ...
        obstacles.center(2) + obstacles.radius*sin(theta), ...
        [0.78, 0.79, 0.80], 'EdgeColor', [0.15, 0.15, 0.15], ...
        'DisplayName', 'obstacle');

    if ~isempty(plannerInfo.rawPath)
        plot(ax, plannerInfo.rawPath(:,1), plannerInfo.rawPath(:,2), ...
            'o--', 'Color', [0.25, 0.45, 0.75], 'LineWidth', 1.1, ...
            'MarkerSize', 4, 'DisplayName', 'raw Sp-RRT');
    end
    if ~isempty(metrics.pathSample)
        plot(ax, metrics.pathSample(:,1), metrics.pathSample(:,2), ...
            '-', 'Color', [0.05, 0.58, 0.34], 'LineWidth', 2.0, ...
            'DisplayName', 'optimized Sp-RRT');
    end
    plot(ax, cfg.startPt(1), cfg.startPt(2), 'o', ...
        'MarkerFaceColor', [0.08, 0.58, 0.55], ...
        'DisplayName', 'start');
    plot(ax, cfg.goalPt(1), cfg.goalPt(2), 's', ...
        'MarkerFaceColor', [0.93, 0.64, 0.12], ...
        'DisplayName', 'goal');
    title(ax, sprintf('Adaptive Sp-RRT-2D | %s', ...
        plannerInfo.terminationReason), ...
        'Interpreter', 'none');
    legend(ax, 'Location', 'bestoutside');
    exportgraphics(fig, filePath, 'Resolution', 300, ...
        'BackgroundColor', 'white');
end
