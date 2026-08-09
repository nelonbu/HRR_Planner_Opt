clear; clc; close all;

%% FIG_METHOD_FRAMEWORK_STATES Reproducible Method Framework assets.
% Part 1: environment/RRT -> shortcut centerline -> cubic B-spline init.
% Part 2: B-spline leader path -> fixed-length chord family -> segment
% clearance of the most critical chord.
% Part 3: active chords -> objective/SAG -> Adam update -> best-safe return.
% Part 4: optimized path -> independent high-precision evaluation -> FTL
% robot-level following result.
%
% Edit only the "Reproducible case selection" block to choose another
% benchmark instance. The first run saves all geometry to a MAT cache;
% later runs reuse it so visual styling can be changed without replanning.
% Set cfg.forceRegenerate = true after changing generation parameters.
%
% This formal figure script depends only on stable project modules and does
% not depend on any debug_*.m script. It calls the frozen optimizer and
% evaluators without changing their algorithms or success definitions.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

common = getCSSCDemoConfig2D();

%% Reproducible case selection
% sceneId: double_slit | s_channel | staggered_baffles | random_mixed
% difficultyId: easy | normal | hard
cfg.sceneId = "double_slit";
cfg.difficultyId = "normal";
cfg.envSeed = 1337;
cfg.plannerSeed = 501037;
cfg.frontendMethod = 'rrt';
cfg.forceRegenerate = false;

% Initialization policy. These values reproduce the formal CSSC candidate
% generation policy used before the frozen CSSC optimization stage.
cfg.maxInitialCandidates = common.csscInitMaxCandidates;
cfg.initialSeedStride = common.csscInitSeedStride;
cfg.numShortcut = common.numShortcut;
cfg.fixedChordNU = 260;
cfg.pathSampleN = 900;

% Part 3-4 data extraction. saveInterval=1 is used only to retain one-case
% visualization snapshots; it does not alter the CSSC update sequence.
cfg.optimizationMaxTimeSec = common.maxPlanningTimeSec;
cfg.highPrecisionNU = 480;
cfg.highPrecisionPathN = 1200;
cfg.gradientArrowLength = 0.100;
cfg.adamArrowMagnification = 10;
cfg.robotNumLinks = 6;
cfg.robotAdvanceStep = 0.005;       % Must divide L exactly.
cfg.robotPathSampleN = 1800;
cfg.robotGoalTol = 2e-3;
cfg.robotPoseFractions = [0.45, 0.72, 1.00];

%% Figure style
cfg.visible = 'on';                  % Set 'off' for unattended export.
cfg.resolutionPPI = 400;
cfg.fontName = 'Times New Roman';
cfg.singleFigureSizeCm = [7.6, 6.2];
cfg.closeupFigureSizeCm = [7.6, 6.2];
cfg.legendFigureSizeCm = [5.4, 6.2];
cfg.fontScale = 1.5;
cfg.lineScale = 1.2;
cfg.chordDisplayCount = 34;
cfg.chordCloseupCount = 50;
cfg.normalArrowLength = 0.055;

style = makeStyle(cfg);
caseTag = makeCaseTag(cfg, common);
outDir = fullfile(projectRoot, 'results', 'figures', ...
    'method_framework', caseTag);
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
removeObsoleteCompositeFiles(outDir);
statePath = fullfile(outDir, 'framework_case_state.mat');

%% Generate once or load the cached intermediate state
signature = makeGenerationSignature(cfg, common);
frameworkState = [];
if ~cfg.forceRegenerate && exist(statePath, 'file')
    cached = load(statePath, 'frameworkState');
    if isfield(cached, 'frameworkState') && ...
            isfield(cached.frameworkState, 'signature') && ...
            isequaln(cached.frameworkState.signature, signature)
        frameworkState = cached.frameworkState;
        fprintf('[fig_method_framework_states] reused cached state\n');
    end
end

if isempty(frameworkState)
    frameworkState = generateFrameworkState(cfg, common, signature);
    save(statePath, 'frameworkState');
    fprintf('[fig_method_framework_states] generated and cached state\n');
end

%% Part 1: RRT-B-spline initialization (four independent assets)
part1Kinds = {'environment', 'rrt', 'shortcut', 'bspline'};
part1Names = { ...
    'part1_initialization_00_environment', ...
    'part1_initialization_01_rrt_path', ...
    'part1_initialization_02_shortcut_centerline', ...
    'part1_initialization_03_bspline_initialization'};
for i = 1:numel(part1Kinds)
    fig = drawInitializationPanel( ...
        frameworkState, cfg, style, part1Kinds{i});
    exportFigurePair(fig, outDir, part1Names{i}, cfg);
end

%% Part 2: fixed-chord overview and critical-region close-up
[figOverview, closeupLimits] = drawFixedChordOverview( ...
    frameworkState, cfg, style);
exportFigurePair(figOverview, outDir, ...
    'part2_fixed_chord_01_overview', cfg);

figCloseup = drawFixedChordCloseup( ...
    frameworkState, cfg, style, closeupLimits);
exportFigurePair(figCloseup, outDir, ...
    'part2_fixed_chord_02_critical_closeup', cfg);

%% Part 3: CSSC optimization states
figActive = drawOptimizationActiveSet(frameworkState, cfg, style);
exportFigurePair(figActive, outDir, ...
    'part3_optimization_01_active_set', cfg);

figUpdate = drawOptimizationUpdate(frameworkState, cfg, style);
exportFigurePair(figUpdate, outDir, ...
    'part3_optimization_02_sag_adam_update', cfg);

figObjective = drawOptimizationHistory(frameworkState, cfg, style);
exportFigurePair(figObjective, outDir, ...
    'part3_optimization_03_objective_pbest', cfg);

figComparison = drawOptimizationBeforeAfter(frameworkState, cfg, style);
exportFigurePair(figComparison, outDir, ...
    'part3_optimization_04_before_after', cfg);

%% Part 4: optimized result, high-precision evaluation, and FTL following
figHighPrecision = drawHighPrecisionResult(frameworkState, cfg, style);
exportFigurePair(figHighPrecision, outDir, ...
    'part4_result_01_high_precision', cfg);

figRobot = drawRobotFollowingResult(frameworkState, cfg, style);
exportFigurePair(figRobot, outDir, ...
    'part4_result_02_ftl_following', cfg);

fprintf('  case : %s / %s, envSeed=%d, plannerSeed=%d\n', ...
    char(cfg.sceneId), char(cfg.difficultyId), ...
    cfg.envSeed, cfg.plannerSeed);
fprintf('  init : selected attempt %d, %d RRT nodes -> %d shortcut nodes -> %d controls\n', ...
    frameworkState.initialization.selectedAttempt, ...
    size(frameworkState.paths.rrt,1), ...
    size(frameworkState.paths.shortcut,1), ...
    size(frameworkState.paths.Pinit,1));
fprintf('  chord: %d valid, minClear=%.6f, alpha*=%.4f\n', ...
    nnz(frameworkState.fixedChord.validLine), ...
    frameworkState.fixedChord.minClear, ...
    frameworkState.fixedChord.closestAlpha( ...
        frameworkState.fixedChord.minIdx));
fprintf('  opt  : iter=%d, returnedIter=%d, stop=%s\n', ...
    frameworkState.optimization.info.numIterActual, ...
    frameworkState.optimization.info.returnedIter, ...
    frameworkState.optimization.info.stopReason);
fprintf('  HP   : minClear=%.6f, dMinSatisfied=%d\n', ...
    frameworkState.highPrecision.optimized.minClear, ...
    frameworkState.highPrecision.optimized.dMinSatisfied);
fprintf('  FTL  : frames=%d, stop=%s, failed=%d\n', ...
    size(frameworkState.robot.motion.joints,1), ...
    frameworkState.robot.motion.stopReason, ...
    frameworkState.robot.motion.failed);
fprintf('  data : %s\n', statePath);
fprintf('  figs : %s\n', outDir);

%% State generation
function state = generateFrameworkState(cfg, common, signature)
    spec = selectSceneSpec(common, cfg.sceneId, cfg.difficultyId);
    [obstacles, envInfo] = generateCSSCBenchmarkScene2D( ...
        spec, common, cfg.envSeed);

    plannerOpts = struct( ...
        'bounds', envInfo.bounds, ...
        'stepSize', common.stepSize, ...
        'goalBias', common.goalBias, ...
        'goalTol', common.goalTol, ...
        'maxIter', common.maxIter, ...
        'maxTimeSec', inf, ...
        'collisionResolution', common.collisionResolution, ...
        'inflateRadius', common.dMin, ...
        'recordTree', true);

    scoreParams = makeChordParams(common, 160, false, false);
    scoreParams.dMin = common.dMin + ...
        common.csscOptimizationClearanceBuffer;

    initOpts = struct( ...
        'maxCandidates', cfg.maxInitialCandidates, ...
        'maxTimeSec', inf, ...
        'plannerSeed', cfg.plannerSeed, ...
        'seedStride', cfg.initialSeedStride, ...
        'shortcutSeedBase', common.shortcutSeedBase, ...
        'numShortcut', cfg.numShortcut, ...
        'acceptMargin', 0, ...
        'frontendMethod', cfg.frontendMethod);

    bundle = prepareCSSCInitialBundle2D( ...
        envInfo, obstacles, plannerOpts, scoreParams, initOpts);
    if ~bundle.success
        error('Framework initialization failed: %s', bundle.message);
    end

    candidate = bundle.selected;
    chordParams = makeChordParams( ...
        common, cfg.fixedChordNU, true, true);
    chordParams.knot = candidate.splineInfo.knot;
    chordParams.pathSampleN = cfg.pathSampleN;
    fixedChord = evaluateCSSCGlobal( ...
        candidate.Pinit, obstacles, chordParams);
    validateFixedChordState(fixedChord);

    % Frozen CSSC optimizer with figure-only per-iteration snapshot storage.
    paramsOpt = makeOptimizationParams(common, cfg, candidate.splineInfo.knot);
    Pref = candidate.Pinit;
    [initialJ, initialDetails, initialGradP] = ...
        objectiveCSSC2D_SemiGrad(candidate.Pinit, Pref, obstacles, paramsOpt);
    optimizerConsole = evalc( ...
        '[Popt, optInfo] = optimizeCSSC2D(candidate.Pinit, Pref, obstacles, paramsOpt);');
    snapshotIds = find(~cellfun(@isempty, optInfo.snapshots));
    if isempty(snapshotIds)
        error('CSSC optimizer returned no control-point snapshot.');
    end
    firstUpdateIter = snapshotIds(1);
    firstUpdateP = optInfo.snapshots{firstUpdateIter};

    % Independent final evaluation retains the full high-resolution state.
    paramsEval = makeHighPrecisionParams( ...
        common, cfg, candidate.splineInfo.knot);
    metricsInit = evaluateCSSCHighPrecision( ...
        candidate.Pinit, obstacles, paramsEval);
    metricsOpt = evaluateCSSCHighPrecision(Popt, obstacles, paramsEval);
    validateFixedChordState(metricsOpt.state);

    % Robot-level FTL conversion uses the optimized path without changing it.
    robotOpts = struct('bodyWidth', 0.040, 'jointRadius', 0.018, ...
        'drawTipJoint', false);
    robot = makeFTLRobot2D(common.L, cfg.robotNumLinks, robotOpts);
    motionOpts = struct('advanceStep', cfg.robotAdvanceStep, ...
        'pathSampleN', cfg.robotPathSampleN, ...
        'goalTol', cfg.robotGoalTol, 'entryDirection', [1,0]);
    robotPathParams = struct('degree', common.degree, ...
        'knot', candidate.splineInfo.knot);
    motion = bsplinePathToFTLRobotMotion2D( ...
        Popt, robotPathParams, robot, motionOpts);
    poseIndices = selectRobotPoseIndices(motion, cfg.robotPoseFractions);

    state = struct();
    state.version = 'method-framework-state-v3';
    state.createdAt = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    state.signature = signature;
    state.sceneSpec = spec;
    state.envInfo = envInfo;
    state.obstacles = obstacles;
    state.commonParameters = struct( ...
        'L', common.L, 'dMin', common.dMin, ...
        'dPref', common.dPref, 'degree', common.degree);
    state.paths = struct( ...
        'rrt', candidate.pathRRT, ...
        'shortcut', candidate.pathShortcut, ...
        'Pinit', candidate.Pinit, ...
        'Popt', Popt, ...
        'knot', candidate.splineInfo.knot);
    state.initialization = struct( ...
        'selectedAttempt', bundle.selectedIndex, ...
        'selectedSeed', candidate.seed, ...
        'numAttempts', bundle.numAttempts, ...
        'numValidCandidates', bundle.numValidCandidates, ...
        'frontendInfo', candidate.frontendInfo, ...
        'shortcutInfo', candidate.shortcutInfo, ...
        'splineInfo', candidate.splineInfo, ...
        'candidateLog', summarizeInitialCandidates(bundle.candidates));
    state.fixedChord = fixedChord;
    state.optimization = struct( ...
        'Popt', Popt, 'Pbest', Popt, 'Pref', Pref, 'params', paramsOpt, ...
        'info', optInfo, 'consoleText', optimizerConsole, ...
        'initialJ', initialJ, 'initialDetails', initialDetails, ...
        'initialGradP', initialGradP, ...
        'firstUpdateIter', firstUpdateIter, ...
        'firstUpdateP', firstUpdateP);
    state.highPrecision = struct( ...
        'params', paramsEval, 'initial', metricsInit, ...
        'optimized', metricsOpt);
    state.robot = struct('model', robot, 'motion', motion, ...
        'poseIndices', poseIndices);
end

function log = summarizeInitialCandidates(candidates)
    template = struct('attempt', 0, 'seed', 0, ...
        'frontendSuccess', false, 'valid', false, 'safe', false, ...
        'minClear', nan, 'pathLength', nan, ...
        'frontendTimeSec', 0, 'initTimeSec', 0);
    log = repmat(template, numel(candidates), 1);
    fields = fieldnames(template);
    for i = 1:numel(candidates)
        for j = 1:numel(fields)
            log(i).(fields{j}) = candidates(i).(fields{j});
        end
    end
end

function params = makeChordParams(common, nU, enablePathSample, metadata)
    params = struct();
    params.L = common.L;
    params.dMin = common.dMin;
    params.dPref = common.dPref;
    params.degree = common.degree;
    params.clearanceMode = 'segment';
    params.envOpts = struct( ...
        'uRange', [0,1], 'vSearchRange', [0,1], ...
        'nU', nU, 'epsV', 1e-6, 'tolDen', 1e-6);
    params.printEvalTiming = false;
    params.enablePathSample = enablePathSample;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = metadata;
end

function params = makeOptimizationParams(common, cfg, knot)
    % Keep these settings aligned with the frozen CSSC configuration used
    % by simu1_baseline_compare. Only saveInterval is reduced to one so the
    % first Adam update can be retained for the framework illustration.
    params = struct();
    params.L = common.L;
    params.dMin = common.dMin + common.csscOptimizationClearanceBuffer;
    params.dPref = common.dPref;
    params.degree = common.degree;
    params.knot = knot;
    params.clearanceMode = 'segment';
    params.envOpts = struct( ...
        'uRange', [0,1], 'vSearchRange', [0,1], ...
        'nU', 160, 'epsV', 1e-6, 'tolDen', 1e-6);

    params.wObs = 60000.0;
    params.wClear = 100.0;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0.0;
    params.wCurv = 0.0;
    params.kappaMax = 2.0;

    params.solver = struct('gradMode', 'semi-analytic', ...
        'objectiveMode', 'cssc-chord');
    params.returnPolicy = 'best-safe';
    params.numIter = 100;
    params.lr = 0.0015;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 1;
    params.activeTopK = 30;
    params.activeClearanceMargin = 0.008;
    params.activeMode = 'topk';

    params.stop = struct('enable', true, 'minIter', 10, ...
        'window', 8, 'tolRelJ', 1e-2, 'tolGrad', 1e-1, ...
        'tolStep', 1e-3, 'patience', 10, 'tolBestRel', 1e-4, ...
        'requireSafe', true, 'clearanceMargin', 0.0, ...
        'maxTimeSec', cfg.optimizationMaxTimeSec);
    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

function params = makeHighPrecisionParams(common, cfg, knot)
    params = struct();
    params.L = common.L;
    params.dMin = common.dMin;
    params.dPref = common.dPref;
    params.degree = common.degree;
    params.knot = knot;
    params.envOpts = struct( ...
        'uRange', [0,1], 'vSearchRange', [0,1], ...
        'nU', cfg.highPrecisionNU, 'epsV', 1e-6, 'tolDen', 1e-6);
    params.pathSampleN = cfg.highPrecisionPathN;
    params.pointClearanceResolution = common.pointClearanceResolution;
end

function poseIndices = selectRobotPoseIndices(motion, fractions)
    nStep = size(motion.joints, 1);
    if nStep < 1
        error('FTL conversion returned no robot posture.');
    end
    fractions = min(max(fractions(:).', 0), 1);
    poseIndices = unique(round(1 + fractions * (nStep - 1)), 'stable');
    poseIndices = min(max(poseIndices, 1), nStep);
end

function validateFixedChordState(state)
    if ~isfinite(state.minClear) || ~isscalar(state.minIdx) || ...
            ~isfinite(state.minIdx)
        error('No finite fixed-chord segment clearance was obtained.');
    end
    i = state.minIdx;
    required = [state.M(i,:), state.N(i,:), state.closestPoint(i,:), ...
        state.closestAlpha(i), state.closestNormal(i,:)];
    if any(~isfinite(required))
        error('The critical fixed-chord geometry contains non-finite values.');
    end
end

function spec = selectSceneSpec(common, sceneId, difficultyId)
    specs = makeCSSCBenchmarkSceneSpecs2D(struct( ...
        'bounds', common.bounds, 'maxEnvSeedTry', 1));
    sceneIndex = find(strcmpi(string({specs.id}), string(sceneId)), 1);
    level = find(strcmpi(common.difficultyIds, string(difficultyId)), 1);
    if isempty(sceneIndex)
        error('Unknown sceneId: %s', char(sceneId));
    end
    if isempty(level)
        error('Unknown difficultyId: %s', char(difficultyId));
    end

    spec = specs(sceneIndex);
    spec.difficultyIndex = level;
    spec.difficultyId = common.difficultyIds(level);
    spec.difficultyName = common.difficultyNames(level);
    switch lower(char(spec.id))
        case 'double_slit'
            spec.gapHeight = common.doubleSlit.dGap(level);
            spec.xWalls = common.doubleSlit.xWalls;
            spec.gapCenterRange = common.doubleSlit.centerRange(level,:);
            spec.wallThickness = common.doubleSlit.wallThickness;
        case 's_channel'
            spec.dGap = common.sChannel.dGap(level);
            spec.channelW = common.sChannel.W;
            spec.channelH = common.sChannel.H;
        case 'staggered_baffles'
            spec.passageWidth = common.baffles.passageWidth(level);
            spec.baffleXCenters = common.baffles.xCenters;
            spec.baffleThickness = common.baffles.thickness;
            spec.bafflePattern = common.baffles.pattern;
        case 'random_mixed'
            spec.nCircle = common.randomMixed.nCircle(level);
            spec.nRect = common.randomMixed.nRect(level);
            spec.minGap = common.randomMixed.minGap(level);
            spec.radiusRange = common.randomMixed.radiusRange;
            spec.halfSizeXRange = common.randomMixed.halfSizeXRange;
            spec.halfSizeYRange = common.randomMixed.halfSizeYRange;
            spec.yawRange = common.randomMixed.yawRange;
            spec.keepoutStart = common.randomMixed.keepoutStart;
            spec.keepoutGoal = common.randomMixed.keepoutGoal;
            spec.maxTry = common.randomMixed.maxTry;
    end
end

%% Part 1 plotting
function fig = drawInitializationPanel(state, cfg, style, kind)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    switch lower(kind)
        case 'environment'
            setPanelTitle(ax, 'Planning environment', cfg);

        case 'rrt'
            hTree = drawRRTTree(ax, state.initialization.frontendInfo, ...
                cfg, style);
            h = plot(ax, state.paths.rrt(:,1), state.paths.rrt(:,2), ...
                '-o', 'Color', style.rrtColor, ...
                'LineWidth', scaledLine(cfg, 1.25), ...
                'MarkerSize', 2.8, 'MarkerFaceColor', style.rrtColor);
            setPanelTitle(ax, 'RRT path', cfg);
            compactLegend(ax, [hTree,h], {'RRT tree','RRT path'}, cfg);

        case 'shortcut'
            hOld = plot(ax, state.paths.rrt(:,1), state.paths.rrt(:,2), ...
                ':', 'Color', style.contextColor, ...
                'LineWidth', scaledLine(cfg, 0.9));
            hShort = plot(ax, state.paths.shortcut(:,1), ...
                state.paths.shortcut(:,2), '-o', ...
                'Color', style.shortcutColor, ...
                'LineWidth', scaledLine(cfg, 1.55), ...
                'MarkerSize', 3.3, 'MarkerFaceColor', style.shortcutColor);
            setPanelTitle(ax, 'Shortcut centerline $\Gamma$', cfg, 'latex');
            compactLegend(ax, [hOld,hShort], ...
                {'RRT path','Shortcut $\Gamma$'}, cfg, 'latex');

        case 'bspline'
            hGamma = plot(ax, state.paths.shortcut(:,1), ...
                state.paths.shortcut(:,2), ':', ...
                'Color', style.shortcutContextColor, ...
                'LineWidth', scaledLine(cfg, 1.0));
            hCtrl = plot(ax, state.paths.Pinit(:,1), ...
                state.paths.Pinit(:,2), '--o', ...
                'Color', style.controlColor, ...
                'LineWidth', scaledLine(cfg, 0.9), ...
                'MarkerSize', 3.5, 'MarkerFaceColor', 'w');
            path = state.fixedChord.pathSample;
            hSpline = plot(ax, path(:,1), path(:,2), '-', ...
                'Color', style.splineColor, ...
                'LineWidth', scaledLine(cfg, 1.8));
            setPanelTitle(ax, 'Cubic B-spline initialization', cfg);
            compactLegend(ax, [hGamma,hCtrl,hSpline], ...
                {'Shortcut $\Gamma$','Control polygon','B-spline'}, ...
                cfg, 'latex');

        otherwise
            error('Unknown initialization panel: %s', kind);
    end
    drawStartGoal(ax, state, cfg, style);
end

%% Part 2 plotting
function [fig, closeupLimits] = drawFixedChordOverview(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);
    chord = state.fixedChord;
    hFamily = drawChordFamily(ax, chord, cfg.chordDisplayCount, cfg, style);
    hPath = plot(ax, chord.pathSample(:,1), chord.pathSample(:,2), '-', ...
        'Color', style.splineColor, 'LineWidth', scaledLine(cfg, 1.8));

    geom = criticalChordGeometry(chord, cfg);
    hCritical = plot(ax, [geom.M(1),geom.N(1)], ...
        [geom.M(2),geom.N(2)], '-', ...
        'Color', style.criticalChordColor, ...
        'LineWidth', scaledLine(cfg, 2.3));
    closeupLimits = criticalViewLimits(geom);
    rectangle(ax, 'Position', [closeupLimits(1,1), closeupLimits(2,1), ...
        diff(closeupLimits(1,:)), diff(closeupLimits(2,:))], ...
        'EdgeColor', style.closeupBoxColor, 'LineStyle', '--', ...
        'LineWidth', scaledLine(cfg, 1.0));

    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'Fixed-chord family and critical region', cfg);
    compactLegend(ax, [hPath,hFamily,hCritical], ...
        {'Leader path','Fixed-length chords','Critical chord'}, cfg);
end

function fig = drawFixedChordCloseup(state, cfg, style, closeupLimits)
    fig = makeFigure(cfg.closeupFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);
    xlim(ax, closeupLimits(1,:));
    ylim(ax, closeupLimits(2,:));

    chord = state.fixedChord;
    drawChordFamily(ax, chord, cfg.chordCloseupCount, cfg, style);
    plot(ax, chord.pathSample(:,1), chord.pathSample(:,2), '-', ...
        'Color', style.splineColor, 'LineWidth', scaledLine(cfg, 1.8));
    geom = criticalChordGeometry(chord, cfg);

    plot(ax, [geom.M(1),geom.N(1)], [geom.M(2),geom.N(2)], '-', ...
        'Color', style.criticalChordColor, ...
        'LineWidth', scaledLine(cfg, 2.3));
    plot(ax, [geom.qBoundary(1),geom.qStar(1)], ...
        [geom.qBoundary(2),geom.qStar(2)], '-', ...
        'Color', style.clearanceColor, ...
        'LineWidth', scaledLine(cfg, 1.7));
    plot(ax, geom.M(1), geom.M(2), 'o', 'MarkerSize', 6.0, ...
        'MarkerFaceColor', style.endpointColor, 'MarkerEdgeColor', 'k', ...
        'LineWidth', scaledLine(cfg, 0.7));
    plot(ax, geom.N(1), geom.N(2), 'o', 'MarkerSize', 6.0, ...
        'MarkerFaceColor', style.endpointColor, 'MarkerEdgeColor', 'k', ...
        'LineWidth', scaledLine(cfg, 0.7));
    plot(ax, geom.qStar(1), geom.qStar(2), 'd', 'MarkerSize', 7.0, ...
        'MarkerFaceColor', style.closestPointColor, ...
        'MarkerEdgeColor', 'k', 'LineWidth', scaledLine(cfg, 0.7));
    plot(ax, geom.qBoundary(1), geom.qBoundary(2), '.', ...
        'Color', style.clearanceColor, 'MarkerSize', 12);
    quiver(ax, geom.qStar(1), geom.qStar(2), ...
        cfg.normalArrowLength*geom.normal(1), ...
        cfg.normalArrowLength*geom.normal(2), 0, ...
        'Color', style.normalColor, ...
        'LineWidth', scaledLine(cfg, 1.2), 'MaxHeadSize', 0.65);

    addChordLabels(ax, geom.M, geom.N, geom.qStar, geom.qBoundary, ...
        geom.normal, geom.alphaStar, cfg, style);
    setPanelTitle(ax, 'Critical chord close-up', cfg);
end

function hFamily = drawChordFamily(ax, chord, count, cfg, style)
    valid = find(chord.validLine(:) & ...
        all(isfinite(chord.M),2) & all(isfinite(chord.N),2));
    displayIds = subsampleIndices(valid, count);
    hFamily = gobjects(1,1);
    for k = 1:numel(displayIds)
        i = displayIds(k);
        h = plot(ax, [chord.M(i,1),chord.N(i,1)], ...
            [chord.M(i,2),chord.N(i,2)], '-', ...
            'Color', style.chordColor, ...
            'LineWidth', scaledLine(cfg, 0.65));
        if k == 1
            hFamily = h;
        end
    end
end

function hTree = drawRRTTree(ax, frontendInfo, cfg, style)
    if ~isfield(frontendInfo, 'treeNodes') || ...
            ~isfield(frontendInfo, 'treeParent') || ...
            isempty(frontendInfo.treeNodes)
        error(['RRT tree data are unavailable. Set cfg.forceRegenerate=true ' ...
            'to rebuild the framework cache.']);
    end
    nodes = frontendInfo.treeNodes;
    parent = frontendInfo.treeParent(:);
    child = find(parent > 0);
    x = [nodes(child,1).'; nodes(parent(child),1).'; nan(1,numel(child))];
    y = [nodes(child,2).'; nodes(parent(child),2).'; nan(1,numel(child))];
    hTree = patch(ax, x(:), y(:), 'w', ...
        'FaceColor', 'none', 'EdgeColor', style.treeColor, ...
        'EdgeAlpha', 0.24, 'LineWidth', scaledLine(cfg, 0.45));
end

function geom = criticalChordGeometry(chord, cfg)
    i = chord.minIdx;
    geom.M = chord.M(i,:);
    geom.N = chord.N(i,:);
    geom.qStar = chord.closestPoint(i,:);
    geom.alphaStar = chord.closestAlpha(i);
    geom.normal = chord.closestNormal(i,:);
    geom.normal = geom.normal / max(norm(geom.normal), eps);
    geom.clearance = chord.clearance(i);
    geom.qBoundary = geom.qStar - geom.clearance * geom.normal;
    geom.qNormalEnd = geom.qStar + cfg.normalArrowLength * geom.normal;
end

function limits = criticalViewLimits(geom)
    points = [geom.M; geom.N; geom.qStar; ...
        geom.qBoundary; geom.qNormalEnd];
    center = 0.5 * (min(points,[],1) + max(points,[],1));
    span = max(points,[],1) - min(points,[],1);
    span = max(span + [0.075,0.075], [0.20,0.18]);
    limits = [center(1)+[-0.5,0.5]*span(1); ...
        center(2)+[-0.5,0.5]*span(2)];
end

function addChordLabels(ax, M, N, qStar, qBoundary, normal, alphaStar, cfg, style)
    e = N - M;
    eUnit = e / max(norm(e), eps);
    perp = [-eUnit(2), eUnit(1)];
    if dot(perp, normal) < 0
        perp = -perp;
    end
    text(ax, M(1)-0.012, M(2)+0.018, '$\mathbf{M}(u)$', ...
        'Interpreter', 'latex', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8));
    text(ax, N(1)+0.008, N(2)+0.018, '$\mathbf{N}(u)$', ...
        'Interpreter', 'latex', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8));
    mid = 0.5 * (M + N) + 0.018 * perp;
    text(ax, mid(1), mid(2), '$L$', 'Interpreter', 'latex', ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 8), ...
        'HorizontalAlignment', 'center');
    text(ax, qStar(1)+0.012*perp(1), qStar(2)+0.012*perp(2), ...
        '$\mathbf{q}^{\star}$', 'Interpreter', 'latex', ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 8), ...
        'Color', style.closestPointColor);
    alphaPos = M + 0.55 * alphaStar * (N-M) - 0.016 * perp;
    text(ax, alphaPos(1), alphaPos(2), '$\alpha^{\star}$', ...
        'Interpreter', 'latex', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8), ...
        'HorizontalAlignment', 'center');
    cPos = 0.5 * (qStar + qBoundary) + 0.012 * [-normal(2), normal(1)];
    text(ax, cPos(1), cPos(2), '$c(u)$', 'Interpreter', 'latex', ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 8), ...
        'Color', style.clearanceColor, 'HorizontalAlignment', 'center');
    nPos = qStar + 0.058 * normal;
    text(ax, nPos(1), nPos(2), '$\mathbf{n}$', 'Interpreter', 'latex', ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 8), ...
        'Color', style.normalColor);
end

%% Part 3 plotting
function fig = drawOptimizationActiveSet(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    details = state.optimization.initialDetails;
    chord = details.state;
    hFamily = drawChordFamily(ax, chord, cfg.chordDisplayCount, cfg, style);
    initialPath = sampleSpline(state.paths.Pinit, state, cfg.pathSampleN);
    hPath = plot(ax, initialPath(:,1), initialPath(:,2), '-', ...
        'Color', style.splineColor, 'LineWidth', scaledLine(cfg, 1.7));

    activeIds = details.active.indices(:);
    hActive = gobjects(1,1);
    hSample = gobjects(1,1);
    for k = 1:numel(activeIds)
        i = activeIds(k);
        if ~chord.validLine(i) || any(~isfinite([chord.M(i,:), chord.N(i,:)]))
            continue;
        end
        h = plot(ax, [chord.M(i,1),chord.N(i,1)], ...
            [chord.M(i,2),chord.N(i,2)], '-', ...
            'Color', style.activeColor, ...
            'LineWidth', scaledLine(cfg, 1.45));
        if ~isgraphics(hActive)
            hActive = h;
        end
        if all(isfinite(chord.closestPoint(i,:)))
            hp = plot(ax, chord.closestPoint(i,1), chord.closestPoint(i,2), ...
                '.', 'Color', style.activePointColor, 'MarkerSize', 9);
            if ~isgraphics(hSample)
                hSample = hp;
            end
        end
    end

    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'Active dangerous chords', cfg);
    handles = [hPath,hFamily];
    labels = {'Initial B-spline','Fixed-length chords'};
    if isgraphics(hActive)
        handles(end+1) = hActive; %#ok<AGROW>
        labels{end+1} = 'Active chords'; %#ok<AGROW>
    end
    if isgraphics(hSample)
        handles(end+1) = hSample; %#ok<AGROW>
        labels{end+1} = 'Active samples'; %#ok<AGROW>
    end
    text(ax, 0.02, 0.97, sprintf( ...
        '$|\\mathcal{A}|=%d,\\quad J_0=%.3g$', ...
        numel(activeIds), state.optimization.initialJ), ...
        'Units', 'normalized', 'Interpreter', 'latex', ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 6.5), ...
        'VerticalAlignment', 'top');
    compactLegend(ax, handles, labels, cfg);
end

function fig = drawOptimizationUpdate(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    P0 = state.paths.Pinit;
    P1 = state.optimization.firstUpdateP;
    path0 = sampleSpline(P0, state, cfg.pathSampleN);
    path1 = sampleSpline(P1, state, cfg.pathSampleN);
    hPath0 = plot(ax, path0(:,1), path0(:,2), '--', ...
        'Color', style.contextColor, 'LineWidth', scaledLine(cfg, 1.15));
    hPath1 = plot(ax, path1(:,1), path1(:,2), '-', ...
        'Color', style.splineColor, 'LineWidth', scaledLine(cfg, 1.75));
    plot(ax, P0(:,1), P0(:,2), ':o', 'Color', style.controlColor, ...
        'LineWidth', scaledLine(cfg, 0.75), 'MarkerSize', 3.5, ...
        'MarkerFaceColor', 'w');

    descent = -state.optimization.initialGradP;
    descent([1,end],:) = 0;
    descent = scaleVectorField(descent, cfg.gradientArrowLength);
    interior = 2:size(P0,1)-1;
    hSAG = quiver(ax, P0(interior,1), P0(interior,2), ...
        descent(interior,1), descent(interior,2), 0, ...
        'Color', style.sagColor, 'LineWidth', scaledLine(cfg, 0.88), ...
        'MaxHeadSize', 0.7);

    adamStep = cfg.adamArrowMagnification * (P1 - P0);
    hAdam = quiver(ax, P0(interior,1), P0(interior,2), ...
        adamStep(interior,1), adamStep(interior,2), 0, ...
        'Color', style.adamColor, 'LineWidth', scaledLine(cfg, 0.96), ...
        'MaxHeadSize', 0.7);
    plot(ax, P1(:,1), P1(:,2), 's', 'Color', style.adamColor, ...
        'MarkerFaceColor', style.adamColor, 'MarkerSize', 3.1);

    text(ax, 0.02, 0.97, sprintf('Adam displacement $\\times%d$', ...
        cfg.adamArrowMagnification), 'Units', 'normalized', ...
        'Interpreter', 'latex', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 6.5), 'VerticalAlignment', 'top');
    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'SAG and first Adam update', cfg);
    compactLegend(ax, [hPath0,hPath1,hSAG,hAdam], ...
        {'Before update','After update','SAG descent','Adam update'}, cfg);
end

function fig = drawOptimizationHistory(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    hold(ax, 'on');
    info = state.optimization.info;
    iter = (1:numel(info.Jhist)).';

    hJ = semilogy(ax, iter, positiveForLog(info.Jhist), '-', ...
        'Color', style.objectiveColor, 'LineWidth', scaledLine(cfg, 1.8));
    hObs = semilogy(ax, iter, positiveForLog(info.JobsHist), '--', ...
        'Color', style.activeColor, 'LineWidth', scaledLine(cfg, 1.15));
    hClear = semilogy(ax, iter, positiveForLog(info.JclearHist), '-.', ...
        'Color', style.clearanceColor, 'LineWidth', scaledLine(cfg, 1.15));
    hReg = semilogy(ax, iter, positiveForLog(info.JregHist), ':', ...
        'Color', style.regularizationColor, 'LineWidth', scaledLine(cfg, 1.35));

    returnedIter = max(1, min(numel(info.Jhist), info.returnedIter));
    hBest = plot(ax, returnedIter, positiveForLog(info.Jhist(returnedIter)), ...
        'p', 'Color', style.bestColor, 'MarkerFaceColor', style.bestColor, ...
        'MarkerSize', 8, 'LineWidth', scaledLine(cfg, 0.8));
    xline(ax, returnedIter, '--', 'Color', style.bestColor, ...
        'LineWidth', scaledLine(cfg, 0.8), 'HandleVisibility', 'off');

    xlabel(ax, 'Iteration', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8));
    ylabel(ax, 'Objective value', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8));
    set(ax, 'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 7.5), ...
        'LineWidth', scaledLine(cfg, 0.7), 'Color', 'w', ...
        'YMinorGrid', 'off');
    grid(ax, 'on');
    box(ax, 'on');
    setPanelTitle(ax, 'Objective history and returned $P_{\mathrm{best}}$', ...
        cfg, 'latex');
    if info.returnedSafe
        bestLabel = '$P_{\mathrm{best}}^{\mathrm{safe}}$';
    else
        bestLabel = '$P_{\mathrm{best}}$';
    end
    compactLegend(ax, [hJ,hObs,hClear,hReg,hBest], ...
        {'$J$','$J_{obs}$','$J_{clear}$','$J_{reg}$',bestLabel}, ...
        cfg, 'latex');
end

function fig = drawOptimizationBeforeAfter(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    metricsInit = state.highPrecision.initial;
    metricsOpt = state.highPrecision.optimized;
    hInit = plot(ax, metricsInit.pathSample(:,1), ...
        metricsInit.pathSample(:,2), '--', ...
        'Color', style.shortcutColor, ...
        'LineWidth', scaledLine(cfg, 1.35));
    hOpt = plot(ax, metricsOpt.pathSample(:,1), ...
        metricsOpt.pathSample(:,2), '-', ...
        'Color', style.optimizedColor, ...
        'LineWidth', scaledLine(cfg, 1.8));

    geomInit = criticalChordGeometry(metricsInit.state, cfg);
    geomOpt = criticalChordGeometry(metricsOpt.state, cfg);
    hInitChord = plot(ax, [geomInit.M(1),geomInit.N(1)], ...
        [geomInit.M(2),geomInit.N(2)], '--', ...
        'Color', style.activeColor, ...
        'LineWidth', scaledLine(cfg, 1.55));
    hOptChord = plot(ax, [geomOpt.M(1),geomOpt.N(1)], ...
        [geomOpt.M(2),geomOpt.N(2)], '-', ...
        'Color', style.criticalChordColor, ...
        'LineWidth', scaledLine(cfg, 2.0));
    plot(ax, geomInit.qStar(1), geomInit.qStar(2), 'x', ...
        'Color', style.activeColor, 'MarkerSize', 6.5, ...
        'LineWidth', scaledLine(cfg, 1.1));
    plot(ax, geomOpt.qStar(1), geomOpt.qStar(2), 'd', ...
        'MarkerFaceColor', style.closestPointColor, ...
        'MarkerEdgeColor', 'k', 'MarkerSize', 6.2, ...
        'LineWidth', scaledLine(cfg, 0.65));

    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'CSSC optimization result', cfg);
    compactLegend(ax, [hInit,hOpt,hInitChord,hOptChord], ...
        {'Initial B-spline','Optimized $P_{\mathrm{best}}$', ...
        'Initial critical chord','Optimized critical chord'}, ...
        cfg, 'latex');
end

%% Part 4 plotting
function fig = drawHighPrecisionResult(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    metrics = state.highPrecision.optimized;
    chord = metrics.state;
    hFamily = drawChordFamily(ax, chord, cfg.chordDisplayCount, cfg, style);
    hPath = plot(ax, metrics.pathSample(:,1), metrics.pathSample(:,2), '-', ...
        'Color', style.optimizedColor, 'LineWidth', scaledLine(cfg, 1.8));
    geom = criticalChordGeometry(chord, cfg);
    hCritical = plot(ax, [geom.M(1),geom.N(1)], ...
        [geom.M(2),geom.N(2)], '-', ...
        'Color', style.criticalChordColor, ...
        'LineWidth', scaledLine(cfg, 2.25));
    hClosest = plot(ax, geom.qStar(1), geom.qStar(2), 'd', ...
        'MarkerSize', 6.5, 'MarkerFaceColor', style.closestPointColor, ...
        'MarkerEdgeColor', 'k', 'LineWidth', scaledLine(cfg, 0.65));
    plot(ax, [geom.qBoundary(1),geom.qStar(1)], ...
        [geom.qBoundary(2),geom.qStar(2)], '-', ...
        'Color', style.clearanceColor, 'LineWidth', scaledLine(cfg, 1.6));

    text(ax, 0.02, 0.97, sprintf( ...
        '$c_{min}^{WB}=%.4f,\\quad d_{min}=%.3f$', ...
        metrics.minClear, metrics.dMin), 'Units', 'normalized', ...
        'Interpreter', 'latex', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 7), 'VerticalAlignment', 'top');
    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'Independent high-precision evaluation', cfg);
    compactLegend(ax, [hPath,hFamily,hCritical,hClosest], ...
        {'Optimized $P_{\mathrm{best}}$','Fixed-length chords', ...
        'Critical chord','$\mathbf{q}^{\star}$'}, cfg, 'latex');
end

function fig = drawRobotFollowingResult(state, cfg, style)
    fig = makeFigure(cfg.singleFigureSizeCm, cfg.visible);
    ax = axes(fig);
    setupSceneAxes(ax, state, cfg, style);

    motion = state.robot.motion;
    robot = state.robot.model;
    hPath = plot(ax, motion.pathSample(:,1), motion.pathSample(:,2), '-', ...
        'Color', style.optimizedColor, 'LineWidth', scaledLine(cfg, 1.65));

    poseIds = state.robot.poseIndices;
    poseColors = interpolatePoseColors(numel(poseIds), style);
    allJoints = zeros(0,2);
    hPose = gobjects(1,1);
    for k = 1:numel(poseIds)
        joints = squeeze(motion.joints(poseIds(k),:,:));
        allJoints = [allJoints; joints]; %#ok<AGROW>
        robotPose = robot;
        robotPose.bodyColor = poseColors(k,:);
        robotPose.jointColor = poseColors(k,:);
        robotPose.tipColor = poseColors(k,:);
        robotPose.bodyFaceAlpha = 0.22 + 0.23 * k / numel(poseIds);
        handles = plotFTLRobot2D(ax, joints, robotPose, struct());
        setRobotHandleStyle(handles, cfg, k == numel(poseIds));
        if ~isgraphics(hPose)
            hPose = handles.links(1);
        end
        tip = joints(end,:);
        text(ax, tip(1)+0.012, tip(2)+0.012, sprintf('$t_%d$', k), ...
            'Interpreter', 'latex', 'FontName', cfg.fontName, ...
            'FontSize', scaledFont(cfg, 6.5), 'Color', poseColors(k,:));
    end
    expandLimitsForRobot(ax, state.envInfo.bounds, allJoints);
    drawStartGoal(ax, state, cfg, style);
    setPanelTitle(ax, 'FTL robot following on $P_{\mathrm{best}}$', ...
        cfg, 'latex');
    compactLegend(ax, [hPath,hPose], ...
        {'Optimized leader path','FTL robot poses'}, cfg);
end

function path = sampleSpline(P, state, nSample)
    u = linspace(0, 1, nSample).';
    path = evalBSplinePath2D(P, u, ...
        state.commonParameters.degree, state.paths.knot);
end

function vectors = scaleVectorField(vectors, targetMaxLength)
    maxLength = max(vecnorm(vectors, 2, 2), [], 'omitnan');
    if isempty(maxLength) || ~isfinite(maxLength) || maxLength <= eps
        vectors(:) = 0;
    else
        vectors = targetMaxLength * vectors / maxLength;
    end
end

function value = positiveForLog(value)
    value = abs(value);
    value(~isfinite(value)) = nan;
    finiteValue = value(isfinite(value) & value > 0);
    if isempty(finiteValue)
        floorValue = 1e-12;
    else
        floorValue = max(1e-12, min(finiteValue) * 0.2);
    end
    value(isfinite(value) & value <= 0) = floorValue;
end

function colors = interpolatePoseColors(n, style)
    if n <= 1
        colors = style.robotLateColor;
        return;
    end
    t = linspace(0,1,n).';
    colors = (1-t) .* style.robotEarlyColor + t .* style.robotLateColor;
end

function setRobotHandleStyle(handles, cfg, isFinal)
    linkWidth = scaledLine(cfg, 0.65 + 0.25 * isFinal);
    jointWidth = scaledLine(cfg, 0.55 + 0.20 * isFinal);
    validLinks = handles.links(isgraphics(handles.links));
    validJoints = handles.joints(isgraphics(handles.joints));
    set(validLinks, 'LineWidth', linkWidth);
    set(validJoints, 'LineWidth', jointWidth);
end

function expandLimitsForRobot(ax, bounds, joints)
    if isempty(joints)
        return;
    end
    points = [joints; bounds(1,1),bounds(2,1); bounds(1,2),bounds(2,2)];
    lo = min(points, [], 1);
    hi = max(points, [], 1);
    span = max(hi-lo, [0.1,0.1]);
    xPad = 0.035 * span(1);
    yPad = 0.055 * span(2);
    xlim(ax, [lo(1)-xPad, hi(1)+xPad]);
    ylim(ax, [lo(2)-yPad, hi(2)+yPad]);
end

%% Common plotting and export helpers
function setupSceneAxes(ax, state, cfg, style)
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    bounds = state.envInfo.bounds;
    span = [diff(bounds(1,:)), diff(bounds(2,:))];
    xlim(ax, bounds(1,:) + [-1,1] * 0.015 * span(1));
    ylim(ax, bounds(2,:) + [-1,1] * 0.015 * span(2));
    set(ax, 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8), ...
        'Color', 'w', 'Layer', 'top');
    drawObstacles(ax, state.obstacles, cfg, style);
    drawBoundary(ax, bounds, cfg, style);
end

function drawObstacles(ax, obstacles, cfg, style)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 100);
                xy = obs.center + obs.radius * [cos(theta(:)), sin(theta(:))];
            case {'rect','rectangle','box'}
                if isfield(obs, 'vertices') && ~isempty(obs.vertices)
                    xy = obs.vertices;
                else
                    local = [-1,-1;1,-1;1,1;-1,1] .* obs.halfSize;
                    R = [cos(obs.yaw),-sin(obs.yaw);sin(obs.yaw),cos(obs.yaw)];
                    xy = local * R.' + obs.center;
                end
            case 'polygon'
                xy = obs.vertices;
            otherwise
                continue;
        end
        patch(ax, xy(:,1), xy(:,2), style.obstacleFaceColor, ...
            'FaceAlpha', 0.90, 'EdgeColor', style.obstacleEdgeColor, ...
            'LineWidth', scaledLine(cfg, 0.75));
    end
end

function drawBoundary(ax, bounds, cfg, style)
    x = bounds(1,:); y = bounds(2,:);
    plot(ax, [x(1),x(2),x(2),x(1),x(1)], ...
        [y(1),y(1),y(2),y(2),y(1)], '-', ...
        'Color', style.boundaryColor, ...
        'LineWidth', scaledLine(cfg, 1.05), ...
        'Clipping', 'off');
end

function drawStartGoal(ax, state, cfg, style)
    qS = state.envInfo.startPt;
    qG = state.envInfo.goalPt;
    text(ax, qS(1), qS(2), 'S', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8), 'FontWeight', 'bold', ...
        'Color', style.startColor, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'BackgroundColor', 'w', ...
        'Margin', 0.4);
    text(ax, qG(1), qG(2), 'G', 'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 8), 'FontWeight', 'bold', ...
        'Color', style.goalColor, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'BackgroundColor', 'w', ...
        'Margin', 0.4);
end

function compactLegend(ax, handles, labels, cfg, interpreter)
    if nargin < 5
        interpreter = 'none';
    end
    fig = ancestor(ax, 'figure');
    legendSpec = struct('handles', handles, 'labels', {labels}, ...
        'interpreter', interpreter);
    setappdata(fig, 'CSSCFrameworkLegendSpec', legendSpec);
end

function setPanelTitle(ax, label, cfg, interpreter)
    if nargin < 4
        interpreter = 'none';
    end
    title(ax, label, 'Interpreter', interpreter, ...
        'FontName', cfg.fontName, 'FontSize', scaledFont(cfg, 6.5), ...
        'FontWeight', 'normal');
end

function value = scaledFont(cfg, baseValue)
    value = cfg.fontScale * baseValue;
end

function value = scaledLine(cfg, baseValue)
    value = cfg.lineScale * baseValue;
end

function fig = makeFigure(sizeCm, visibility)
    fig = figure('Color', 'w', 'Visible', visibility, ...
        'Units', 'centimeters', 'Position', [2,2,sizeCm], ...
        'PaperUnits', 'centimeters', 'PaperPosition', [0,0,sizeCm], ...
        'Renderer', 'painters');
end

function ids = subsampleIndices(valid, count)
    if isempty(valid)
        ids = valid;
        return;
    end
    n = min(numel(valid), max(1, round(count)));
    ids = valid(unique(round(linspace(1, numel(valid), n))));
end

function exportFigurePair(fig, outDir, baseName, cfg)
    removeOldLogicalFigureFiles(outDir, baseName);
    imageName = [baseName '_image'];
    legendName = [baseName '_legend'];
    exportgraphics(fig, fullfile(outDir, [imageName '.jpg']), ...
        'Resolution', cfg.resolutionPPI, 'BackgroundColor', 'white');
    exportgraphics(fig, fullfile(outDir, [imageName '.pdf']), ...
        'ContentType', 'vector', 'BackgroundColor', 'white');

    legendFig = makeStandaloneLegendFigure(fig, cfg);
    exportgraphics(legendFig, fullfile(outDir, [legendName '.jpg']), ...
        'Resolution', cfg.resolutionPPI, 'BackgroundColor', 'white');
    exportgraphics(legendFig, fullfile(outDir, [legendName '.pdf']), ...
        'ContentType', 'vector', 'BackgroundColor', 'white');
    close(legendFig);
end

function legendFig = makeStandaloneLegendFigure(sourceFig, cfg)
    legendFig = figure('Color', 'w', 'Visible', 'off', ...
        'Units', 'centimeters', 'Position', [2,2,cfg.legendFigureSizeCm], ...
        'PaperUnits', 'centimeters', ...
        'PaperPosition', [0,0,cfg.legendFigureSizeCm], ...
        'Renderer', 'painters');
    ax = axes(legendFig, 'Position', [0.02,0.02,0.96,0.96]);
    hold(ax, 'on');
    axis(ax, 'off');
    xlim(ax, [1e6,1e6+1]);
    ylim(ax, [1e6,1e6+1]);

    if ~isappdata(sourceFig, 'CSSCFrameworkLegendSpec')
        return;
    end
    spec = getappdata(sourceFig, 'CSSCFrameworkLegendSpec');
    valid = isgraphics(spec.handles);
    sourceHandles = spec.handles(valid);
    labels = spec.labels(valid);
    if isempty(sourceHandles)
        return;
    end
    proxyHandles = gobjects(size(sourceHandles));
    for i = 1:numel(sourceHandles)
        proxyHandles(i) = copyobj(sourceHandles(i), ax);
    end
    lgd = legend(ax, proxyHandles, labels, 'Location', 'northwest', ...
        'Orientation', 'vertical', 'NumColumns', 1, 'Box', 'off', ...
        'FontName', cfg.fontName, ...
        'FontSize', scaledFont(cfg, 6.5), ...
        'Interpreter', spec.interpreter);
    lgd.ItemTokenSize = [13, 9];
end

function removeOldLogicalFigureFiles(outDir, baseName)
    names = {baseName, [baseName '_image'], [baseName '_legend']};
    extensions = {'.jpg','.pdf'};
    for i = 1:numel(names)
        for j = 1:numel(extensions)
            path = fullfile(outDir, [names{i}, extensions{j}]);
            if exist(path, 'file')
                delete(path);
            end
        end
    end
end

function style = makeStyle(~)
    style = struct();
    style.obstacleFaceColor = [0.76, 0.78, 0.80];
    style.obstacleEdgeColor = [0.16, 0.17, 0.18];
    style.boundaryColor = [0.05, 0.05, 0.05];
    style.startColor = [0.08, 0.58, 0.55];
    style.startEdgeColor = [0.05, 0.20, 0.18];
    style.goalColor = [0.93, 0.64, 0.12];
    style.goalEdgeColor = [0.35, 0.22, 0.03];
    style.rrtColor = [0.18, 0.46, 0.72];
    style.treeColor = [0.32, 0.48, 0.62];
    style.shortcutColor = [0.88, 0.43, 0.16];
    style.shortcutContextColor = [0.85, 0.66, 0.50];
    style.contextColor = [0.62, 0.68, 0.73];
    style.controlColor = [0.38, 0.40, 0.43];
    style.splineColor = [0.06, 0.28, 0.58];
    % MATLAB line objects do not expose a portable per-line alpha setting;
    % use a light neutral gray to retain the intended subdued appearance.
    style.chordColor = [0.76, 0.77, 0.79];
    style.criticalChordColor = [0.08, 0.08, 0.10];
    style.endpointColor = [0.95, 0.75, 0.20];
    style.closestPointColor = [0.67, 0.18, 0.55];
    style.clearanceColor = [0.00, 0.52, 0.55];
    style.normalColor = [0.12, 0.55, 0.25];
    style.closeupBoxColor = [0.72, 0.16, 0.16];
    style.activeColor = [0.78, 0.10, 0.12];
    style.activePointColor = [0.92, 0.22, 0.16];
    style.sagColor = [0.10, 0.52, 0.24];
    style.adamColor = [0.93, 0.52, 0.10];
    style.objectiveColor = [0.05, 0.18, 0.34];
    style.regularizationColor = [0.42, 0.35, 0.62];
    style.bestColor = [0.67, 0.18, 0.55];
    style.optimizedColor = [0.02, 0.30, 0.62];
    style.robotEarlyColor = [0.53, 0.70, 0.84];
    style.robotLateColor = [0.10, 0.34, 0.68];
end

function removeObsoleteCompositeFiles(outDir)
    names = { ...
        'fig_framework_part1_initialization', ...
        'fig_framework_part2_fixed_chord', ...
        'part1a_rrt_path', ...
        'part1b_shortcut_centerline', ...
        'part1c_bspline_initialization', ...
        'part2_fixed_chord_evaluation'};
    extensions = {'.jpg','.pdf'};
    for i = 1:numel(names)
        for j = 1:numel(extensions)
            path = fullfile(outDir, [names{i}, extensions{j}]);
            if exist(path, 'file')
                delete(path);
            end
        end
    end
end

function signature = makeGenerationSignature(cfg, common)
    signature = struct( ...
        'version', 'method-framework-generation-v3', ...
        'profileName', common.profileName, ...
        'sceneId', string(cfg.sceneId), ...
        'difficultyId', string(cfg.difficultyId), ...
        'envSeed', cfg.envSeed, ...
        'plannerSeed', cfg.plannerSeed, ...
        'frontendMethod', string(cfg.frontendMethod), ...
        'L', common.L, 'dMin', common.dMin, 'dPref', common.dPref, ...
        'degree', common.degree, ...
        'stepSize', common.stepSize, ...
        'goalBias', common.goalBias, ...
        'goalTol', common.goalTol, ...
        'maxIter', common.maxIter, ...
        'collisionResolution', common.collisionResolution, ...
        'maxInitialCandidates', cfg.maxInitialCandidates, ...
        'initialSeedStride', cfg.initialSeedStride, ...
        'numShortcut', cfg.numShortcut, ...
        'fixedChordNU', cfg.fixedChordNU, ...
        'pathSampleN', cfg.pathSampleN, ...
        'optimizationMaxTimeSec', cfg.optimizationMaxTimeSec, ...
        'optimizationClearanceBuffer', ...
            common.csscOptimizationClearanceBuffer, ...
        'highPrecisionNU', cfg.highPrecisionNU, ...
        'highPrecisionPathN', cfg.highPrecisionPathN, ...
        'pointClearanceResolution', common.pointClearanceResolution, ...
        'robotNumLinks', cfg.robotNumLinks, ...
        'robotAdvanceStep', cfg.robotAdvanceStep, ...
        'robotPathSampleN', cfg.robotPathSampleN, ...
        'robotGoalTol', cfg.robotGoalTol, ...
        'robotPoseFractions', cfg.robotPoseFractions);
end

function tag = makeCaseTag(cfg, common)
    raw = sprintf('%s_%s_e%d_p%d_%s', ...
        char(cfg.sceneId), char(cfg.difficultyId), ...
        cfg.envSeed, cfg.plannerSeed, char(common.profileName));
    tag = regexprep(lower(raw), '[^a-z0-9_-]', '_');
end
