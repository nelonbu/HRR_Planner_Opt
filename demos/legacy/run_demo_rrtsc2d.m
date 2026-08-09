clear; clc; close all;

%% RRTSC-2D smoke demo
% Runs the formal global-replanning baseline:
%   RRT -> B-spline smoothing -> centerline validation ->
%   fixed-length chord-sweep validation.
% Failed candidates are discarded in full; no shortcut or local optimizer
% is used. The accepted path is evaluated by evaluateCSSCHighPrecision and
% saved through the stable result.mat structure.

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

obstacles = [
    obsCircle2D([0.40,  0.14], 0.070)
    obsCircle2D([0.62, -0.14], 0.070)
    obsRect2D(  [0.78,  0.15], [0.035, 0.080], 0.12)
];

rrtscOpts = makeRRTSCOpts(cfg);
[Paccepted, pathRRT, rrtscInfo] = planRRTSC2D( ...
    cfg.startPt, cfg.goalPt, obstacles, rrtscOpts);

evalOpts = makeEvaluationOpts(cfg, rrtscInfo);
tEval = tic;
metrics = evaluateCSSCHighPrecision(Paccepted, obstacles, evalOpts);
highPrecisionEvalTimeSec = toc(tEval);

paths = struct( ...
    'pathRRT', pathRRT, ...
    'Pinit', Paccepted, ...
    'Popt', Paccepted, ...
    'Pref', [], ...
    'pathBSplineInit', metrics.pathSample, ...
    'pathOptimized', metrics.pathSample);
timing = rrtscInfo.timing;
timing.highPrecisionEvalTimeSec = highPrecisionEvalTimeSec;
successFlags = struct( ...
    'plannerSuccess', rrtscInfo.success, ...
    'highPrecisionSuccess', metrics.success, ...
    'dMinSatisfied', metrics.dMinSatisfied, ...
    'pointSuccess', metrics.pointSuccess);
seed = struct('envSeed', cfg.envSeed, 'plannerSeed', cfg.plannerSeed);

result = makeCSSCExperimentResult(cfg, seed, obstacles, paths, ...
    rrtscInfo, metrics, timing, successFlags);
save(fullfile(cfg.outDir, 'result.mat'), 'result');

figPath = fullfile(cfg.outDir, 'rrtsc2d_result.jpg');
plotRRTSCResult(cfg, obstacles, pathRRT, metrics, rrtscInfo, figPath);

fprintf('\n[RRTSC-2D]\n');
fprintf('  success             : %d\n', rrtscInfo.success);
fprintf('  termination         : %s\n', rrtscInfo.terminationReason);
fprintf('  attempts            : %d\n', rrtscInfo.attemptCount);
fprintf('  global replans      : %d\n', rrtscInfo.globalReplanningCount);
fprintf('  centerline rejects  : %d\n', rrtscInfo.centerlineRejectCount);
fprintf('  chord rejects       : %d\n', rrtscInfo.chordRejectCount);
fprintf('  numerical failures  : %d\n', rrtscInfo.numericalFailureCount);
fprintf('  planning time       : %.4f s\n', rrtscInfo.planningTimeSec);
fprintf('  evaluation time     : %.4f s\n', highPrecisionEvalTimeSec);
fprintf('  high-precision clear: %.6f\n', metrics.minClear);
fprintf('  result              : %s\n', fullfile(cfg.outDir, 'result.mat'));
fprintf('  figure              : %s\n', figPath);

function cfg = makeDemoConfig(projectRoot)
    cfg = getCSSCDemoConfig2D();
    cfg.projectRoot = projectRoot;
    cfg.runName = ['run_rrtsc2d_' datestr(now, 'yyyymmdd_HHMMSS')];
    cfg.outDir = fullfile(projectRoot, 'results', 'runs', cfg.runName);
    cfg.envSeed = 1;
    cfg.plannerSeed = 42001;
    cfg.figureVisible = 'off';
end

function opts = makeRRTSCOpts(cfg)
    opts = defaultRRTSC2DParams();
    opts.L = cfg.L;
    opts.dMin = cfg.dMin;
    opts.safetyMargin = cfg.dMin;
    opts.degree = cfg.degree;
    opts.bounds = cfg.bounds;
    opts.seed = cfg.plannerSeed;
    opts.maxAttempts = cfg.rrtscMaxAttempts;
    opts.maxTotalTime = cfg.rrtscMaxTotalTime;
    opts.verbose = true;

    opts.rrt.bounds = cfg.bounds;
    opts.rrt.stepSize = cfg.stepSize;
    opts.rrt.goalBias = cfg.goalBias;
    opts.rrt.goalTol = cfg.goalTol;
    opts.rrt.maxIter = cfg.maxIter;
    opts.rrt.collisionResolution = cfg.collisionResolution;
    opts.rrt.inflateRadius = cfg.inflateRadius;

    opts.centerline.sampleResolution = cfg.rrtscCenterlineResolution;
    opts.centerline.minSamples = cfg.rrtscCenterlineMinSamples;
    opts.chord.nU = cfg.rrtscChordNU;
    opts.chord.maxRefinement = cfg.rrtscMaxRefinement;
end

function opts = makeEvaluationOpts(cfg, info)
    opts = struct();
    opts.L = cfg.L;
    opts.dMin = cfg.dMin;
    opts.degree = cfg.degree;
    if info.success && isstruct(info.finalSplineInfo)
        opts.knot = info.finalSplineInfo.knot;
    end
    opts.envOpts = struct( ...
        'uRange', [0, 1], ...
        'vSearchRange', [0, 1], ...
        'nU', 480, ...
        'epsV', 1e-8, ...
        'tolDen', 1e-10);
    opts.pathSampleN = 1200;
    opts.pointClearanceResolution = 0.001;
end

function plotRRTSCResult(cfg, obstacles, pathRRT, metrics, info, filePath)
    fig = figure('Color', 'w', 'Visible', cfg.figureVisible, ...
        'Name', 'RRTSC-2D result', 'Position', [100, 100, 900, 520]);
    ax = axes(fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlim(ax, cfg.bounds(1,:));
    ylim(ax, cfg.bounds(2,:));

    drawObstacles(ax, obstacles);
    if ~isempty(pathRRT)
        plot(ax, pathRRT(:,1), pathRRT(:,2), ':', ...
            'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.2, ...
            'DisplayName', 'accepted RRT');
    end
    if ~isempty(metrics.pathSample)
        plot(ax, metrics.pathSample(:,1), metrics.pathSample(:,2), '-', ...
            'Color', [0.05, 0.48, 0.76], 'LineWidth', 2.0, ...
            'DisplayName', 'RRTSC B-spline');
    end
    drawAcceptedChords(ax, info);

    plot(ax, cfg.startPt(1), cfg.startPt(2), 'o', ...
        'MarkerSize', 7, 'MarkerFaceColor', [0.08, 0.58, 0.55], ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'DisplayName', 'start');
    plot(ax, cfg.goalPt(1), cfg.goalPt(2), 's', ...
        'MarkerSize', 7, 'MarkerFaceColor', [0.93, 0.64, 0.12], ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'DisplayName', 'goal');

    title(ax, sprintf('RRTSC-2D | %s | attempts=%d', ...
        info.terminationReason, info.attemptCount), 'Interpreter', 'none');
    xlabel(ax, 'x');
    ylabel(ax, 'y');
    legend(ax, 'Location', 'bestoutside');
    exportgraphics(fig, filePath, 'Resolution', 300, ...
        'BackgroundColor', 'white');
    close(fig);
end

function drawAcceptedChords(ax, info)
    if ~info.success || isempty(info.chordValidation) || ...
            isempty(info.chordValidation.env)
        return;
    end

    env = info.chordValidation.env;
    idx = find(env.validLine);
    if isempty(idx)
        return;
    end
    stride = max(1, ceil(numel(idx) / 24));
    idx = idx(1:stride:end);
    for i = idx(:).'
        plot(ax, [env.M(i,1), env.N(i,1)], ...
            [env.M(i,2), env.N(i,2)], '-', ...
            'Color', [0.65, 0.65, 0.65], ...
            'LineWidth', 0.65, 'HandleVisibility', 'off');
    end
end

function drawObstacles(ax, obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                t = linspace(0, 2*pi, 100).';
                vertices = obs.center + obs.radius * [cos(t), sin(t)];
            case 'rect'
                vertices = rectangleVertices(obs);
            case 'polygon'
                vertices = obs.vertices;
            otherwise
                continue;
        end
        patch(ax, vertices(:,1), vertices(:,2), [0.76, 0.78, 0.80], ...
            'FaceAlpha', 0.90, 'EdgeColor', [0.16, 0.17, 0.18], ...
            'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
end

function vertices = rectangleVertices(obs)
    h = obs.halfSize;
    local = [-h(1), -h(2); h(1), -h(2); ...
        h(1), h(2); -h(1), h(2)];
    R = [cos(obs.yaw), -sin(obs.yaw); ...
        sin(obs.yaw), cos(obs.yaw)];
    vertices = local * R.' + obs.center;
end
