clear; clc; close all;

%% Figure 1: qualitative results in four representative scene families
% Exports one 2-by-2 publication figure containing double slit, S channel,
% staggered baffles, and random mixed obstacles. Each panel overlays the
% B-spline initialization and proposed CSSC result. Final clearance values
% and markers are obtained from evaluateCSSCHighPrecision.

try
    projectRoot = initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

cfg = makeDisplayConfig(projectRoot);
paramsOpt = makeOptimizationParams(cfg);
paramsEval = makeHighPrecisionEvalParams(paramsOpt);
sceneSpecs = makeSceneSpecs(cfg);

if ~exist(cfg.outDir, 'dir')
    mkdir(cfg.outDir);
end

fprintf('\n[qualitative result figure]\n');
fprintf('  output dir : %s\n', cfg.outDir);

results = cell(numel(sceneSpecs), 1);
for i = 1:numel(sceneSpecs)
    spec = sceneSpecs(i);
    fprintf('\n[%d/%d] %s\n', i, numel(sceneSpecs), char(spec.name));

    results{i} = buildSceneResult(spec, cfg, paramsOpt, paramsEval);

    fprintf('  centerline clearance : %.5f\n', results{i}.initMetrics.pointMinClear);
    fprintf('  initial chord clear  : %.5f\n', results{i}.initState.minClear);
    fprintf('  CSSC chord clear     : %.5f\n', results{i}.optState.minClear);
end

jpgPath = fullfile(cfg.outDir, 'fig_qualitative_result.jpg');
pdfPath = fullfile(cfg.outDir, 'fig_qualitative_result.pdf');
matPath = fullfile(cfg.outDir, 'fig_qualitative_result_data.mat');
exportQualitativeFigure(results, cfg, jpgPath, pdfPath);
save(matPath, 'cfg', 'sceneSpecs', 'results');

fprintf('\n[done]\n');
fprintf('  JPG : %s\n', jpgPath);
fprintf('  PDF : %s\n', pdfPath);
fprintf('  data: %s\n', matPath);

%% Configuration

function cfg = makeDisplayConfig(projectRoot)
    cfg = struct();
    cfg.projectRoot = projectRoot;
    cfg.outDir = fullfile(projectRoot, 'results', 'figures', 'qualitative_result');

    cfg.bounds = [0, 1; -0.4, 0.4];
    cfg.startPt = [0.05, 0.0];
    cfg.goalPt = [0.95, 0.0];
    cfg.dMin = 0.02;
    % Keep zero for the current geometry-only project definition. Set a
    % positive value only when explicitly modelling finite body thickness.
    cfg.bodyRadius = 0.0;
    cfg.safetyInflation = cfg.bodyRadius + cfg.dMin;

    cfg.figureWidthCm = 14.0;
    cfg.figureHeightCm = 12;
    cfg.resolutionPPI = 400;
    cfg.figureVisible = 'on';
    cfg.fontName = 'Times New Roman';

    cfg.startColor = [0.08, 0.58, 0.55];
    cfg.goalColor = [0.93, 0.64, 0.12];
    cfg.initPathColor = [0.18, 0.38, 0.76];
    cfg.optPathColor = [0.05, 0.55, 0.28];
    cfg.initMarkerColor = [0.12, 0.28, 0.78];
    cfg.optMarkerColor = [0.72, 0.16, 0.62];
    cfg.safetyBoundaryColor = [0.92, 0.52, 0.10];
    cfg.chordColor = [0.30, 0.30, 0.30];
    cfg.collisionColor = [0.92, 0.08, 0.06];

    cfg.initPathLineWidth = 1.35;
    cfg.optPathLineWidth = 1.70;
    cfg.chordLineWidth = 0.72;
    cfg.chordAlpha = 0.48;
    cfg.chordStride = 12;
    cfg.sweepStride = 3;
    cfg.safeSweepAlpha = 0.055;
    cfg.collisionChordAlpha = 0.55;

    cfg.maxEnvSeedTry = 20;
    cfg.rrtSeedBase = 4300;
    cfg.shortcutSeedBase = 7300;
    cfg.requireRepresentative = true;
    cfg.maxInitialChordClear = 0.0;
    cfg.minOptimizedClear = cfg.safetyInflation;
end

function specs = makeSceneSpecs(cfg)
    specs = repmat(emptySceneSpec(), 4, 1);

    specs(1) = emptySceneSpec();
    specs(1).id = "double_slit";
    specs(1).name = "Double Slit";
    specs(1).kind = "structured";
    specs(1).type = "offsetDoubleSlit";
    specs(1).envSeeds = 1:cfg.maxEnvSeedTry;
    specs(1).gapHeight = 0.08;
    specs(1).xWalls = [0.32, 0.68];
    specs(1).gapCenterRange = [-0.3, 0.3];
    specs(1).wallThickness = 0.08;

    specs(2) = emptySceneSpec();
    specs(2).id = "s_channel";
    specs(2).name = "S Channel";
    specs(2).kind = "structured";
    specs(2).type = "fourRectSChannel";
    specs(2).envSeeds = [1593, 31, 1142, 1101:1101+cfg.maxEnvSeedTry-1];
    specs(2).dGap = 0.10;
    specs(2).channelW = 0.64;
    specs(2).channelH = cfg.bounds(2,2) - cfg.bounds(2,1);

    specs(3) = emptySceneSpec();
    specs(3).id = "staggered_baffles";
    specs(3).name = "Staggered Baffles";
    specs(3).kind = "structured";
    specs(3).type = "staggeredBaffles3";
    specs(3).envSeeds = 1:cfg.maxEnvSeedTry;
    specs(3).passageWidth = 0.36;
    specs(3).baffleXCenters = [0.25, 0.50, 0.75];
    specs(3).baffleThickness = 0.05;
    specs(3).bafflePattern = [1, -1, 1];

    specs(4) = emptySceneSpec();
    specs(4).id = "random_mixed";
    specs(4).name = "Random Mixed Obstacles";
    specs(4).kind = "random";
    specs(4).type = "randomMixed";
    specs(4).envSeeds = [2401, 2402:2401+cfg.maxEnvSeedTry];
    specs(4).nCircle = 8;
    specs(4).nRect = 6;
    specs(4).radiusRange = [0.022, 0.045];
    specs(4).halfSizeXRange = [0.022, 0.052];
    specs(4).halfSizeYRange = [0.025, 0.060];
    specs(4).yawRange = [-pi/4, pi/4];
    specs(4).minGap = 0.020;
    specs(4).keepoutStart = 0.07;
    specs(4).keepoutGoal = 0.07;
    specs(4).maxTry = 10000;
end

function spec = emptySceneSpec()
    spec = struct();
    spec.id = "";
    spec.name = "";
    spec.kind = "";
    spec.type = "";
    spec.envSeeds = 1;
    spec.gapHeight = nan;
    spec.xWalls = [nan, nan];
    spec.gapCentersY = [nan, nan];
    spec.gapCenterRange = [nan, nan];
    spec.wallThickness = nan;
    spec.dGap = nan;
    spec.channelW = nan;
    spec.channelH = nan;
    spec.passageWidth = nan;
    spec.baffleXCenters = [nan, nan, nan];
    spec.baffleThickness = nan;
    spec.bafflePattern = [nan, nan, nan];
    spec.nObs = nan;
    spec.nCircle = nan;
    spec.nRect = nan;
    spec.radiusRange = [nan, nan];
    spec.halfSizeXRange = [nan, nan];
    spec.halfSizeYRange = [nan, nan];
    spec.yawRange = [nan, nan];
    spec.minGap = nan;
    spec.keepoutStart = nan;
    spec.keepoutGoal = nan;
    spec.maxTry = nan;
end

function params = makeOptimizationParams(cfg)
    params = struct();
    params.L = 0.15;
    params.dMin = cfg.safetyInflation;
    params.dPref = cfg.safetyInflation + 0.01;
    params.degree = 3;

    params.envOpts = struct();
    params.envOpts.uRange = [0, 1];
    params.envOpts.vSearchRange = [0, 1];
    params.envOpts.nU = 120;
    params.envOpts.epsV = 1e-6;
    params.envOpts.tolDen = 1e-6;

    params.wObs = 600.0;
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
    params.printInterval = 50;
    params.saveInterval = 25;

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
    params.stop.requireSafe = false;
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
    paramsEval.pathSampleN = 900;
    paramsEval.enablePointClearance = false;
    paramsEval.enableObstacleMetadata = false;
    paramsEval.printEvalTiming = false;
end

%% Scene pipeline

function result = buildSceneResult(spec, cfg, paramsOpt, paramsEval)
    lastMessage = "";
    bestResult = [];
    bestScore = -inf;

    for iSeed = 1:numel(spec.envSeeds)
        envSeed = spec.envSeeds(iSeed);

        try
            [obstacles, envInfo] = makeScene(spec, cfg, envSeed);
            envInfo.obstacles = obstacles;

            rrtOpts = makeRRTOpts(envInfo, cfg.rrtSeedBase + iSeed);
            [pathRRT, rrtInfo] = runRRTWithRestarts(envInfo, rrtOpts);
            if ~rrtInfo.success
                lastMessage = string(rrtInfo.message);
                continue;
            end

            shortcutOpts = rrtOpts;
            shortcutOpts.seed = cfg.shortcutSeedBase + iSeed;
            shortcutOpts.numShortcut = 180;
            [pathShort, shortcutInfo] = shortcutPath2D(pathRRT, obstacles, shortcutOpts);

            params = paramsOpt;
            approxLen = polylineLength(pathShort);
            nCtrl = max(params.degree + 1, ceil(approxLen / (0.5 * params.L)) + 1);
            [Pinit, splineInfo] = polylineToBSplineInit2D(pathShort, ...
                struct('degree', params.degree, 'nCtrl', nCtrl));

            Pref = Pinit;
            params.knot = splineInfo.knot;

            paramsEvalCase = paramsEval;
            paramsEvalCase.knot = splineInfo.knot;
            initMetrics = evaluateCSSCHighPrecision(Pinit, obstacles, paramsEvalCase);
            initState = initMetrics.state;

            % Avoid expensive optimization for candidates that do not show
            % the intended centerline-safe but fixed-chord-unsafe contrast.
            initialContrast = isfinite(initMetrics.pointMinClear) && ...
                initMetrics.pointMinClear > 0 && ...
                isfinite(initMetrics.minClear) && ...
                initMetrics.minClear <= cfg.maxInitialChordClear;
            if cfg.requireRepresentative && ~initialContrast
                lastMessage = "Initial path does not exhibit the requested contrast.";
                continue;
            end

            optLog = evalc('[Popt, optInfo] = optimizeCSSC2D(Pinit, Pref, obstacles, params);'); %#ok<NASGU>
            optMetrics = evaluateCSSCHighPrecision(Popt, obstacles, paramsEvalCase);
            optState = optMetrics.state;

            candidate = struct();
            candidate.spec = spec;
            candidate.envSeed = envSeed;
            candidate.envInfo = envInfo;
            candidate.obstacles = obstacles;
            candidate.pathRRT = pathRRT;
            candidate.pathShort = pathShort;
            candidate.shortcutInfo = shortcutInfo;
            candidate.Pinit = Pinit;
            candidate.Popt = Popt;
            candidate.splineInfo = splineInfo;
            candidate.rrtInfo = rrtInfo;
            candidate.optInfo = optInfo;
            candidate.initMetrics = initMetrics;
            candidate.optMetrics = optMetrics;
            candidate.initState = initState;
            candidate.optState = optState;

            [isRepresentative, score] = scoreRepresentativeCase(candidate, cfg);
            if score > bestScore
                bestScore = score;
                bestResult = candidate;
            end

            if ~cfg.requireRepresentative || isRepresentative
                result = candidate;
                return;
            end
        catch ME
            lastMessage = string(ME.message);
        end
    end

    if ~isempty(bestResult)
        warning('No strict representative case found for "%s"; using the best candidate.', ...
            char(spec.name));
        result = bestResult;
        return;
    end

    error('Could not build scene "%s": %s', char(spec.name), char(lastMessage));
end

function [tf, score] = scoreRepresentativeCase(result, cfg)
    pointClear = result.initMetrics.pointMinClear;
    initChordClear = result.initMetrics.minClear;
    optChordClear = result.optMetrics.minClear;

    pointSafe = isfinite(pointClear) && pointClear > 0;
    initialRisk = isfinite(initChordClear) && ...
        initChordClear <= cfg.maxInitialChordClear;
    optimizedSafe = isfinite(optChordClear) && ...
        optChordClear >= cfg.minOptimizedClear;

    tf = pointSafe && initialRisk && optimizedSafe;
    improvement = optChordClear - initChordClear;
    if ~isfinite(improvement)
        improvement = -1;
    end
    score = 2 * double(pointSafe) + 3 * double(initialRisk) + ...
        4 * double(optimizedSafe) + improvement / max(cfg.dMin, eps);
end

function [obstacles, envInfo] = makeScene(spec, cfg, envSeed)
    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    opts.seed = envSeed;

    switch lower(char(spec.kind))
        case 'structured'
            switch lower(char(spec.type))
                case 'offsetdoubleslit'
                    opts.xWalls = spec.xWalls;
                    opts.gapHeight = spec.gapHeight;
                    opts.wallThickness = spec.wallThickness;
                    opts.gapCentersY = sampleDoubleSlitCenters( ...
                        spec.gapCenterRange, cfg.bounds, ...
                        opts.gapHeight, envSeed);

                case 'fourrectschannel'
                    opts.dGap = spec.dGap;
                    opts.W = spec.channelW;
                    opts.H = spec.channelH;
                    opts.yInOutRange = [-0.20, 0.20];

                case 'staggeredbaffles3'
                    spanY = cfg.bounds(2,2) - cfg.bounds(2,1);
                    halfHeight = 0.5 * (spanY - spec.passageWidth);
                    centers = zeros(3, 2);
                    centers(:,1) = spec.baffleXCenters(:);
                    for i = 1:3
                        if spec.bafflePattern(i) > 0
                            centers(i,2) = cfg.bounds(2,2) - halfHeight;
                        else
                            centers(i,2) = cfg.bounds(2,1) + halfHeight;
                        end
                    end
                    opts.centers = centers;
                    opts.halfSize = [0.5 * spec.baffleThickness, halfHeight];
            end

            [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(spec.type, opts);
            envInfo.seed = envSeed;

        case 'random'
            opts.minGap = spec.minGap;
            opts.maxTry = spec.maxTry;

            switch lower(char(spec.type))
                case 'randomcircles'
                    opts.nObs = spec.nObs;
                    opts.radiusRange = spec.radiusRange;

                case 'randomrects'
                    opts.nObs = spec.nObs;
                    opts.halfSizeXRange = spec.halfSizeXRange;
                    opts.halfSizeYRange = spec.halfSizeYRange;
                    opts.yawRange = spec.yawRange;

                case 'randommixed'
                    opts.nCircle = spec.nCircle;
                    opts.nRect = spec.nRect;
                    opts.radiusRange = spec.radiusRange;
                    opts.halfSizeXRange = spec.halfSizeXRange;
                    opts.halfSizeYRange = spec.halfSizeYRange;
                    opts.yawRange = spec.yawRange;
                    opts.keepoutStart = spec.keepoutStart;
                    opts.keepoutGoal = spec.keepoutGoal;
            end

            [obstacles, envInfo] = generateCSSCEnvironment2D(spec.type, opts);

        otherwise
            error('Unknown scene kind: %s', char(spec.kind));
    end

    envInfo.id = char(spec.id);
    envInfo.name = char(spec.name);
end

function centers = sampleDoubleSlitCenters(centerRange, bounds, gapHeight, seed)
    feasibleRange = [bounds(2,1), bounds(2,2)] + ...
        [0.5, -0.5] * gapHeight;
    sampleRange = [max(centerRange(1), feasibleRange(1)), ...
        min(centerRange(2), feasibleRange(2))];
    if sampleRange(1) >= 0 || sampleRange(2) <= 0
        error('Double-slit center range must span zero after clipping.');
    end

    stream = RandStream('mt19937ar', 'Seed', seed);
    centers = [ ...
        sampleRange(1) + (0 - sampleRange(1)) * rand(stream), ...
        0 + (sampleRange(2) - 0) * rand(stream)];
end

function rrtOpts = makeRRTOpts(envInfo, seed)
    rrtOpts = struct();
    rrtOpts.bounds = envInfo.bounds;
    rrtOpts.stepSize = 0.028;
    rrtOpts.goalBias = 0.16;
    rrtOpts.goalTol = 0.035;
    rrtOpts.maxIter = 8000;
    rrtOpts.collisionResolution = 0.003;
    rrtOpts.inflateRadius = 0.0;
    rrtOpts.seed = seed;
    rrtOpts.numRestart = 10;
end

function [path, info] = runRRTWithRestarts(envInfo, rrtOpts)
    path = zeros(0, 2);
    info = struct('success', false, 'message', 'RRT failed.', ...
        'numIter', 0, 'numNodes', 0, 'numRestartUsed', 0);

    for trial = 1:rrtOpts.numRestart
        optsTrial = rrtOpts;
        optsTrial.seed = rrtOpts.seed + trial - 1;
        [pathTrial, infoTrial] = planRRT2D( ...
            envInfo.startPt, envInfo.goalPt, envInfo.obstacles, optsTrial);

        infoTrial.numRestartUsed = trial;
        info = infoTrial;
        if infoTrial.success
            path = pathTrial;
            return;
        end
    end
end

%% Figure export

function exportQualitativeFigure(results, cfg, jpgPath, pdfPath)
    fig = figure( ...
        'Color', 'w', ...
        'Visible', cfg.figureVisible, ...
        'Units', 'centimeters', ...
        'Position', [2, 2, cfg.figureWidthCm, cfg.figureHeightCm], ...
        'PaperUnits', 'centimeters', ...
        'PaperPosition', [0, 0, cfg.figureWidthCm, cfg.figureHeightCm], ...
        'InvertHardcopy', 'off');

    layout = tiledlayout(fig, 2, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');
    panelLabels = {'(a) Double slit', '(b) S channel', ...
        '(c) Staggered baffles', '(d) Random mixed obstacles'};
    axesList = gobjects(4, 1);

    for i = 1:4
        axesList(i) = nexttile(layout, i);
        drawScenePanel(axesList(i), results{i}, cfg);
        text(axesList(i), 0.5, -0.035, panelLabels{i}, ...
            'Units', 'normalized', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'top', ...
            'FontName', cfg.fontName, ...
            'FontSize', 11.5, ...
            'FontWeight', 'normal', ...
            'Clipping', 'off');
    end

    [handles, labels] = createSharedLegendHandles(axesList(1), cfg);
    lgd = legend(axesList(1), handles, labels, ...
        'NumColumns', 2, ...
        'Location', 'southoutside', ...
        'Interpreter', 'none', ...
        'FontName', cfg.fontName, ...
        'FontSize', 10.0, ...
        'Box', 'off');
    lgd.Layout.Tile = 'south';

    exportgraphics(fig, jpgPath, ...
        'Resolution', cfg.resolutionPPI, ...
        'BackgroundColor', 'white');
    exportgraphics(fig, pdfPath, ...
        'ContentType', 'vector', ...
        'BackgroundColor', 'white');
end

function drawScenePanel(ax, result, cfg)
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    set(ax, 'FontName', cfg.fontName, 'FontSize', 10.0);

    bounds = result.envInfo.bounds;
    spanX = diff(bounds(1,:));
    spanY = diff(bounds(2,:));
    xlim(ax, bounds(1,:) + [-1, 1] * 0.012 * spanX);
    ylim(ax, bounds(2,:) + [-1, 1] * 0.012 * spanY);

    drawObstacles(ax, result.obstacles);
    drawSafetyBoundaries(ax, result.obstacles, cfg.safetyInflation, cfg);
    drawSweptChordPatches(ax, result.initState, cfg);
    drawSparseChords(ax, result.optState, cfg);
    drawBoundary(ax, bounds);

    plot(ax, result.initState.pathSample(:,1), result.initState.pathSample(:,2), ...
        '--', 'Color', cfg.initPathColor, 'LineWidth', cfg.initPathLineWidth);
    plot(ax, result.optState.pathSample(:,1), result.optState.pathSample(:,2), ...
        '-', 'Color', cfg.optPathColor, 'LineWidth', cfg.optPathLineWidth);

    drawClearanceMarkers(ax, result, cfg);
    drawStartGoal(ax, result.envInfo.startPt, result.envInfo.goalPt, cfg);
end

function drawBoundary(ax, bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot(ax, [x(1), x(2), x(2), x(1), x(1)], ...
        [y(1), y(1), y(2), y(2), y(1)], ...
        'k-', 'LineWidth', 1.15, 'Clipping', 'off');
end

function drawObstacles(ax, obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 100);
                vertices = obs.center + obs.radius * [cos(theta(:)), sin(theta(:))];
            case {'rect', 'rectangle', 'box'}
                vertices = rectangleVertices(obs.center, obs.halfSize, obs.yaw);
            otherwise
                continue;
        end

        patch(ax, vertices(:,1), vertices(:,2), [0.76, 0.78, 0.80], ...
            'FaceAlpha', 0.90, ...
            'EdgeColor', [0.16, 0.17, 0.18], ...
            'LineWidth', 0.75);
    end
end

function drawSafetyBoundaries(ax, obstacles, inflation, cfg)
    if inflation <= 0
        return;
    end

    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                theta = linspace(0, 2*pi, 120).';
                vertices = obs.center + (obs.radius + inflation) * ...
                    [cos(theta), sin(theta)];
            case {'rect', 'rectangle', 'box'}
                vertices = rectangleVertices(obs.center, obs.halfSize, obs.yaw);
                vertices = offsetRectangleOutline(vertices, inflation);
            otherwise
                continue;
        end

        plot(ax, [vertices(:,1); vertices(1,1)], ...
            [vertices(:,2); vertices(1,2)], '--', ...
            'Color', cfg.safetyBoundaryColor, ...
            'LineWidth', 0.72);
    end
end

function vertices = rectangleVertices(center, halfSize, yaw)
    local = [
        -halfSize(1), -halfSize(2)
         halfSize(1), -halfSize(2)
         halfSize(1),  halfSize(2)
        -halfSize(1),  halfSize(2)
    ];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    vertices = (R * local.').' + center;
end

function offsetVertices = offsetRectangleOutline(vertices, distance)
    vertices = removeRepeatedEndVertex(vertices);
    if signedOutlineArea(vertices) < 0
        vertices = flipud(vertices);
    end

    n = size(vertices, 1);
    offsetVertices = zeros(n, 2);
    for i = 1:n
        iPrev = mod(i - 2, n) + 1;
        iNext = mod(i, n) + 1;

        edgePrev = vertices(i,:) - vertices(iPrev,:);
        edgeNext = vertices(iNext,:) - vertices(i,:);
        edgePrev = edgePrev / max(norm(edgePrev), eps);
        edgeNext = edgeNext / max(norm(edgeNext), eps);
        normalPrev = [edgePrev(2), -edgePrev(1)];
        normalNext = [edgeNext(2), -edgeNext(1)];

        pointPrev = vertices(i,:) + distance * normalPrev;
        pointNext = vertices(i,:) + distance * normalNext;
        A = [edgePrev(:), -edgeNext(:)];

        if abs(det(A)) < 1e-10
            normal = normalPrev + normalNext;
            normal = normal / max(norm(normal), eps);
            offsetVertices(i,:) = vertices(i,:) + distance * normal;
        else
            parameter = A \ (pointNext - pointPrev).';
            offsetVertices(i,:) = pointPrev + parameter(1) * edgePrev;
        end
    end
end

function vertices = removeRepeatedEndVertex(vertices)
    if size(vertices, 1) > 1 && norm(vertices(1,:) - vertices(end,:)) < 1e-12
        vertices = vertices(1:end-1,:);
    end
end

function area = signedOutlineArea(vertices)
    next = vertices([2:end, 1], :);
    area = 0.5 * sum(vertices(:,1) .* next(:,2) - next(:,1) .* vertices(:,2));
end

function drawSweptChordPatches(ax, state, cfg)
    [valid, clearance] = chordStateData(state);
    validIdx = find(valid);
    if numel(validIdx) < 2
        return;
    end

    selected = validIdx(1:cfg.sweepStride:end);
    if selected(end) ~= validIdx(end)
        selected(end+1) = validIdx(end); %#ok<AGROW>
    end

    for k = 1:numel(selected)-1
        i = selected(k);
        j = selected(k+1);
        if any(~valid(i:j))
            continue;
        end

        vertices = [state.M(i,:); state.N(i,:); state.N(j,:); state.M(j,:)];
        colliding = any(isfinite(clearance(i:j)) & clearance(i:j) <= 0);
        if ~colliding
            patch(ax, vertices(:,1), vertices(:,2), [0.42, 0.42, 0.42], ...
                'FaceAlpha', cfg.safeSweepAlpha, ...
                'EdgeColor', 'none');
        end
    end

    collisionMask = valid & isfinite(clearance) & clearance <= 0;
    drawChordMask(ax, state.M, state.N, collisionMask, ...
        cfg.collisionColor, cfg.collisionChordAlpha, ...
        1.6 * cfg.chordLineWidth);
end

function drawSparseChords(ax, state, cfg)
    [valid, clearance] = chordStateData(state);
    idx = find(valid);
    idx = idx(1:cfg.chordStride:end);

    for k = 1:numel(idx)
        i = idx(k);
        if isfinite(clearance(i)) && clearance(i) <= 0
            color = cfg.collisionColor;
            lineWidth = 1.6 * cfg.chordLineWidth;
        else
            color = cfg.chordColor;
            lineWidth = cfg.chordLineWidth;
        end

        patch(ax, ...
            'XData', [state.M(i,1), state.N(i,1)], ...
            'YData', [state.M(i,2), state.N(i,2)], ...
            'FaceColor', 'none', ...
            'EdgeColor', color, ...
            'EdgeAlpha', cfg.chordAlpha, ...
            'LineWidth', lineWidth);
    end
end

function drawChordMask(ax, M, N, mask, color, alphaValue, lineWidth)
    idx = find(mask);
    for k = 1:numel(idx)
        i = idx(k);
        patch(ax, ...
            'XData', [M(i,1), N(i,1)], ...
            'YData', [M(i,2), N(i,2)], ...
            'FaceColor', 'none', ...
            'EdgeColor', color, ...
            'EdgeAlpha', alphaValue, ...
            'LineWidth', lineWidth);
    end
end

function [valid, clearance] = chordStateData(state)
    valid = state.validLine(:) & ...
        all(isfinite(state.M), 2) & all(isfinite(state.N), 2);

    clearance = nan(size(valid));
    if isfield(state, 'clearance') && numel(state.clearance) == numel(valid)
        clearance = state.clearance(:);
    elseif isfield(state, 'clearanceSegment') && ...
            numel(state.clearanceSegment) == numel(valid)
        clearance = state.clearanceSegment(:);
    end
end

function drawClearanceMarkers(ax, result, cfg)
    pointMin = result.initMetrics.pointMinPoint;
    if all(isfinite(pointMin))
        plot(ax, pointMin(1), pointMin(2), 'x', ...
            'Color', cfg.initMarkerColor, ...
            'MarkerSize', 7.5, ...
            'LineWidth', 1.45);
    end

    optMin = result.optState.minPoint;
    if all(isfinite(optMin))
        plot(ax, optMin(1), optMin(2), 'p', ...
            'MarkerSize', 8.5, ...
            'MarkerFaceColor', cfg.optMarkerColor, ...
            'MarkerEdgeColor', [0.10, 0.10, 0.10], ...
            'LineWidth', 0.75);
    end
end

function drawStartGoal(ax, startPt, goalPt, cfg)
    plot(ax, startPt(1), startPt(2), 'o', ...
        'MarkerSize', 7.0, ...
        'MarkerFaceColor', cfg.startColor, ...
        'MarkerEdgeColor', [0.05, 0.20, 0.18], ...
        'LineWidth', 0.75);
    plot(ax, goalPt(1), goalPt(2), 's', ...
        'MarkerSize', 7.0, ...
        'MarkerFaceColor', cfg.goalColor, ...
        'MarkerEdgeColor', [0.35, 0.22, 0.03], ...
        'LineWidth', 0.75);

    text(ax, startPt(1) + 0.018, startPt(2) - 0.038, 'S', ...
        'FontName', cfg.fontName, 'FontSize', 9.5, 'FontWeight', 'bold');
    text(ax, goalPt(1) - 0.025, goalPt(2) - 0.038, 'G', ...
        'FontName', cfg.fontName, 'FontSize', 9.5, 'FontWeight', 'bold');
end

function [handles, labels] = createSharedLegendHandles(ax, cfg)
    handles = gobjects(6, 1);
    handles(1) = plot(ax, nan, nan, '--', 'Color', cfg.initPathColor, ...
        'LineWidth', cfg.initPathLineWidth);
    handles(2) = patch(ax, ...
        'XData', [nan, nan], 'YData', [nan, nan], ...
        'FaceColor', 'none', ...
        'EdgeColor', cfg.collisionColor, ...
        'EdgeAlpha', cfg.collisionChordAlpha, ...
        'LineWidth', 1.6 * cfg.chordLineWidth);
    handles(3) = plot(ax, nan, nan, 'x', ...
        'Color', cfg.initMarkerColor, 'MarkerSize', 7.5, 'LineWidth', 1.4);
    handles(4) = plot(ax, nan, nan, '-', 'Color', cfg.optPathColor, ...
        'LineWidth', cfg.optPathLineWidth);
    handles(5) = plot(ax, nan, nan, '-', 'Color', cfg.chordColor, ...
        'LineWidth', cfg.chordLineWidth);
    handles(6) = plot(ax, nan, nan, 'p', ...
        'MarkerFaceColor', cfg.optMarkerColor, ...
        'MarkerEdgeColor', [0.1, 0.1, 0.1], 'MarkerSize', 7.5);

    % MATLAB fills multi-column legends column by column.
    labels = {'Initial path', 'Collision chords', ...
        'Initial minimum clearance', 'CSSC path', ...
        'Fixed-length chords', 'CSSC minimum clearance'};
end

%% Small utilities

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end
