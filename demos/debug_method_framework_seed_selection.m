clear; clc; close all;

%% DEBUG_METHOD_FRAMEWORK_SEED_SELECTION Select a Framework illustration case.
% Randomly evaluates four environment seeds for one scene and difficulty.
% One 4-by-3 figure is opened without saving files. Each row is one case:
%   1) initial B-spline and its fixed-chord swept shadow;
%   2) optimized B-spline and its fixed-chord swept shadow;
%   3) optimized critical-chord close-up.
% Both envSeed and the automatically sampled plannerSeed are printed so a
% selected result can be reproduced exactly in the formal figure script.

try
    projectRoot = initCSSCProjectPath; %#ok<NASGU>
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath'))); %#ok<NASGU>
    addpath(genpath(fullfile(projectRoot, 'src')));
end
common = getCSSCDemoConfig2D();

%% Selection configuration
cfg.sceneId = "double_slit";  % double_slit | s_channel | staggered_baffles | random_mixed
cfg.difficultyId = "normal";  % easy | normal | hard
cfg.numCases = 4;
cfg.envSeedRange = [1, 99999];
cfg.envSeeds = [];             % []: randomly sample four; or specify four seeds.
cfg.plannerSeedRange = [100000, 999999];

cfg.maxInitializationTimeSec = 6.0;
cfg.maxOptimizationTimeSec = common.maxPlanningTimeSec;
cfg.visualNU = 300;
cfg.pathSampleN = 900;
cfg.sweepStride = 2;
cfg.figureSizeCm = [24, 22];
cfg.fontName = 'Times New Roman';

spec = selectSceneSpecLocal(common, cfg.sceneId, cfg.difficultyId);
[envSeeds, plannerSeeds] = sampleSeedPairs(cfg);
if strcmpi(spec.id, 'staggered_baffles')
    warning(['The current staggered-baffle geometry is deterministic; ' ...
        'envSeed changes its label but not its obstacle geometry.']);
end

records = repmat(struct('envSeed',0, 'plannerSeed',0, ...
    'success',false, 'initMinClear',nan, 'optMinClear',nan, ...
    'dMinSatisfied',false, 'stopReason',"", 'message',""), ...
    cfg.numCases, 1);

fig = figure('Color','w', 'Name','Framework seed selection', ...
    'NumberTitle','off', 'Units','centimeters', ...
    'Position',[1,1,cfg.figureSizeCm], 'Renderer','painters');
tl = tiledlayout(fig, cfg.numCases, 3, ...
    'TileSpacing','compact', 'Padding','compact');
title(tl, sprintf('%s | %s | four seed candidates', ...
    char(spec.name), char(cfg.difficultyId)), ...
    'FontName',cfg.fontName, 'FontSize',12, 'FontWeight','normal');
sharedHandles = gobjects(1,5);
sharedLegendReady = false;
sharedLegendAxis = gobjects(1,1);

fprintf('\n[Framework seed selection]\n');
fprintf('  scene=%s | difficulty=%s | cases=%d\n\n', ...
    char(cfg.sceneId), char(cfg.difficultyId), cfg.numCases);

for i = 1:cfg.numCases
    records(i).envSeed = envSeeds(i);
    records(i).plannerSeed = plannerSeeds(i);
    rowAxes = gobjects(1,3);
    for column = 1:3
        rowAxes(column) = nexttile(tl, (i-1)*3+column);
    end
    try
        caseState = runOneCase(spec, common, cfg, ...
            envSeeds(i), plannerSeeds(i));
        records(i).success = true;
        records(i).initMinClear = caseState.metricsInit.minClear;
        records(i).optMinClear = caseState.metricsOpt.minClear;
        records(i).dMinSatisfied = caseState.metricsOpt.dMinSatisfied;
        records(i).stopReason = string(caseState.optInfo.stopReason);
        records(i).message = "ok";

        hInit = drawSweptState(rowAxes(1), caseState, ...
            caseState.metricsInit, cfg, common, i, ...
            'Initial B-spline and swept chord set', 'initial');
        hOpt = drawSweptState(rowAxes(2), caseState, ...
            caseState.metricsOpt, cfg, common, i, ...
            'Optimized B-spline and swept chord set', 'optimized');
        drawCriticalCloseup(rowAxes(3), caseState, ...
            caseState.metricsOpt, cfg, common, i);
        if ~sharedLegendReady
            sharedHandles = [hInit.path,hOpt.path,hOpt.sweep, ...
                hOpt.critical,hOpt.point];
            sharedLegendAxis = rowAxes(1);
            sharedLegendReady = true;
        end

        fprintf(['  case %d | envSeed=%d | plannerSeed=%d | ' ...
            'init=%.5f | opt=%.5f | dMin=%d | stop=%s\n'], ...
            i, envSeeds(i), plannerSeeds(i), ...
            records(i).initMinClear, records(i).optMinClear, ...
            records(i).dMinSatisfied, char(records(i).stopReason));
    catch ME
        records(i).message = string(ME.message);
        fprintf('  case %d | envSeed=%d | plannerSeed=%d | FAILED: %s\n', ...
            i, envSeeds(i), plannerSeeds(i), ME.message);
        for k = 1:3
            drawFailurePanel(rowAxes(k), spec, common, cfg, ...
                i, k, ME.message);
        end
    end
end

if sharedLegendReady
    lgd = legend(sharedLegendAxis, sharedHandles, ...
        {'Initial B-spline','Optimized B-spline','Swept chord set', ...
        'Critical chord','$\mathbf{q}^{\star}$'}, ...
        'Interpreter','latex', 'Orientation','vertical', ...
        'NumColumns',1, 'Box','off', 'FontName',cfg.fontName, ...
        'FontSize',9);
    lgd.Layout.Tile = 'east';
end

fprintf('\n[copy the selected pair]\n');
for i = 1:cfg.numCases
    fprintf('  case %d: cfg.envSeed = %d; cfg.plannerSeed = %d;\n', ...
        i, records(i).envSeed, records(i).plannerSeed);
end
fprintf('\nNo figure files were saved.\n');

function state = runOneCase(spec, common, cfg, envSeed, plannerSeed)
    [obstacles, envInfo] = generateCSSCBenchmarkScene2D( ...
        spec, common, envSeed);
    plannerOpts = struct('bounds', envInfo.bounds, ...
        'stepSize', common.stepSize, 'goalBias', common.goalBias, ...
        'goalTol', common.goalTol, 'maxIter', common.maxIter, ...
        'maxTimeSec', cfg.maxInitializationTimeSec, ...
        'collisionResolution', common.collisionResolution, ...
        'inflateRadius', common.dMin, 'recordTree', false);
    scoreParams = makeOptimizationParamsLocal(common, cfg, []);
    scoreParams.envOpts.nU = 160;
    scoreParams.enablePathSample = false;
    scoreParams.enableObstacleMetadata = false;
    initOpts = struct('maxCandidates', common.csscInitMaxCandidates, ...
        'maxTimeSec', cfg.maxInitializationTimeSec, ...
        'plannerSeed', plannerSeed, ...
        'seedStride', common.csscInitSeedStride, ...
        'shortcutSeedBase', common.shortcutSeedBase, ...
        'numShortcut', common.numShortcut, 'acceptMargin', 0, ...
        'frontendMethod', 'rrt');
    bundle = prepareCSSCInitialBundle2D( ...
        envInfo, obstacles, plannerOpts, scoreParams, initOpts);
    if ~bundle.success
        error('Initialization failed: %s', bundle.message);
    end

    candidate = bundle.selected;
    paramsOpt = makeOptimizationParamsLocal( ...
        common, cfg, candidate.splineInfo.knot);
    Pref = candidate.Pinit;
    optimizerConsole = evalc( ...
        '[Popt,optInfo] = optimizeCSSC2D(candidate.Pinit,Pref,obstacles,paramsOpt);'); %#ok<NASGU>
    paramsEval = makeEvaluationParamsLocal( ...
        common, cfg, candidate.splineInfo.knot);
    metricsInit = evaluateCSSCHighPrecision( ...
        candidate.Pinit, obstacles, paramsEval);
    metricsOpt = evaluateCSSCHighPrecision(Popt, obstacles, paramsEval);
    if isempty(metricsInit.state) || isempty(metricsOpt.state)
        error('High-precision evaluation returned no chord state.');
    end
    if ~isfinite(metricsInit.minClear) || ~isfinite(metricsOpt.minClear) || ...
            ~isfinite(metricsInit.state.minIdx) || ...
            ~isfinite(metricsOpt.state.minIdx)
        error('High-precision evaluation returned invalid critical geometry.');
    end

    state = struct('envInfo',envInfo, 'obstacles',obstacles, ...
        'Pinit',candidate.Pinit, 'Popt',Popt, ...
        'knot',candidate.splineInfo.knot, 'optInfo',optInfo, ...
        'metricsInit',metricsInit, 'metricsOpt',metricsOpt);
end

function params = makeOptimizationParamsLocal(common, cfg, knot)
    params = struct('L',common.L, ...
        'dMin',common.dMin+common.csscOptimizationClearanceBuffer, ...
        'dPref',common.dPref, 'degree',common.degree, ...
        'clearanceMode','segment');
    if ~isempty(knot)
        params.knot = knot;
    end
    params.envOpts = struct('uRange',[0,1], 'vSearchRange',[0,1], ...
        'nU',160, 'epsV',1e-6, 'tolDen',1e-6);
    params.wObs = 60000.0;
    params.wClear = 100.0;
    params.wRef = 0.01;
    params.wSmooth = 0.5;
    params.wLength = 0.002;
    params.wTrust = 0.0;
    params.wCurv = 0.0;
    params.kappaMax = 2.0;
    params.solver = struct('gradMode','semi-analytic', ...
        'objectiveMode','cssc-chord');
    params.returnPolicy = 'best-safe';
    params.numIter = 100;
    params.lr = 0.0015;
    params.fdStep = 1e-5;
    params.gradClip = 5.0;
    params.printInterval = 100;
    params.saveInterval = 100;
    params.activeTopK = 30;
    params.activeClearanceMargin = 0.008;
    params.activeMode = 'topk';
    params.stop = struct('enable',true, 'minIter',10, 'window',8, ...
        'tolRelJ',1e-2, 'tolGrad',1e-1, 'tolStep',1e-3, ...
        'patience',10, 'tolBestRel',1e-4, 'requireSafe',true, ...
        'clearanceMargin',0, 'maxTimeSec',cfg.maxOptimizationTimeSec);
    params.printEvalTiming = false;
    params.enableTimingDebug = false;
    params.enablePathSample = false;
    params.enablePointClearance = false;
    params.enableObstacleMetadata = false;
end

function params = makeEvaluationParamsLocal(common, cfg, knot)
    params = struct('L',common.L, 'dMin',common.dMin, ...
        'dPref',common.dPref, 'degree',common.degree, 'knot',knot, ...
        'envOpts',struct('uRange',[0,1], 'vSearchRange',[0,1], ...
        'nU',cfg.visualNU, 'epsV',1e-6, 'tolDen',1e-6), ...
        'pathSampleN',cfg.pathSampleN, ...
        'pointClearanceResolution',common.pointClearanceResolution);
end

function handles = drawSweptState(ax, caseState, metrics, cfg, common, ...
        caseId, titleText, phase)
    hold(ax,'on');
    setupSceneLocal(ax, caseState, cfg);
    hSweep = drawSweepShadowLocal(ax, metrics.state, cfg, common.dMin);
    if strcmpi(phase,'initial')
        pathColor = [0.88,0.43,0.16];
        pathStyle = '--';
    else
        pathColor = [0.02,0.30,0.62];
        pathStyle = '-';
    end
    hPath = plot(ax, metrics.pathSample(:,1), metrics.pathSample(:,2), ...
        pathStyle, 'Color', pathColor, 'LineWidth', 1.7);
    geom = criticalGeometryLocal(metrics.state);
    hCritical = plot(ax, [geom.M(1),geom.N(1)], ...
        [geom.M(2),geom.N(2)], '-', 'Color', [0.08,0.08,0.10], ...
        'LineWidth', 2.0);
    hPoint = plot(ax, geom.qStar(1),geom.qStar(2), 'd', ...
        'MarkerSize',6.5, 'MarkerFaceColor',[0.67,0.18,0.55], ...
        'MarkerEdgeColor','k');
    drawStartGoalLocal(ax, caseState.envInfo, cfg);
    title(ax, sprintf('%s | case %d | $c_{min}^{WB}=%.4f$', ...
        titleText, caseId, metrics.minClear), 'Interpreter','latex', ...
        'FontName',cfg.fontName, 'FontSize',11, 'FontWeight','normal');
    handles = struct('path',hPath, 'sweep',hSweep, ...
        'critical',hCritical, 'point',hPoint);
end

function hSweep = drawSweepShadowLocal(ax, state, cfg, dMin)
    valid = state.validLine(:) & all(isfinite(state.M),2) & ...
        all(isfinite(state.N),2);
    idx = find(valid);
    selected = idx(1:cfg.sweepStride:end);
    if ~isempty(idx) && (isempty(selected) || selected(end) ~= idx(end))
        selected(end+1) = idx(end); %#ok<AGROW>
    end
    hSweep = patch(ax,nan,nan,[0.42,0.42,0.42], ...
        'FaceAlpha',0.10, 'EdgeColor','none');
    for k = 1:numel(selected)-1
        i = selected(k); j = selected(k+1);
        if any(~valid(i:j))
            continue;
        end
        vertices = [state.M(i,:);state.N(i,:);state.N(j,:);state.M(j,:)];
        collision = any(isfinite(state.clearance(i:j)) & ...
            state.clearance(i:j) < dMin);
        if collision
            color = [0.82,0.10,0.12]; alpha = 0.20;
        else
            color = [0.42,0.42,0.42]; alpha = 0.10;
        end
        h = patch(ax,vertices(:,1),vertices(:,2),color, ...
            'FaceAlpha',alpha, 'EdgeColor','none');
        if k == 1
            hSweep = h;
        end
    end
end

function drawCriticalCloseup(ax, caseState, metrics, cfg, common, caseId)
    hold(ax,'on');
    setupSceneLocal(ax,caseState,cfg);
    state = metrics.state;
    geom = criticalGeometryLocal(state);
    limits = closeupLimitsLocal(geom);
    xlim(ax,limits(1,:)); ylim(ax,limits(2,:));

    ids = find(state.validLine(:) & all(isfinite(state.M),2) & ...
        all(isfinite(state.N),2));
    ids = ids(unique(round(linspace(1,numel(ids),min(45,numel(ids))))));
    for k = 1:numel(ids)
        j = ids(k);
        plot(ax,[state.M(j,1),state.N(j,1)], ...
            [state.M(j,2),state.N(j,2)], '-', ...
            'Color',[0.76,0.77,0.79], 'LineWidth',0.55);
    end
    plot(ax,metrics.pathSample(:,1),metrics.pathSample(:,2), '-', ...
        'Color',[0.02,0.30,0.62], 'LineWidth',1.7);
    plot(ax,[geom.M(1),geom.N(1)],[geom.M(2),geom.N(2)], '-', ...
        'Color',[0.08,0.08,0.10], 'LineWidth',2.1);
    plot(ax,[geom.qBoundary(1),geom.qStar(1)], ...
        [geom.qBoundary(2),geom.qStar(2)], '-', ...
        'Color',[0.00,0.52,0.55], 'LineWidth',1.5);
    plot(ax,geom.M(1),geom.M(2),'o','MarkerSize',5.5, ...
        'MarkerFaceColor',[0.95,0.75,0.20],'MarkerEdgeColor','k');
    plot(ax,geom.N(1),geom.N(2),'o','MarkerSize',5.5, ...
        'MarkerFaceColor',[0.95,0.75,0.20],'MarkerEdgeColor','k');
    plot(ax,geom.qStar(1),geom.qStar(2),'d','MarkerSize',6.5, ...
        'MarkerFaceColor',[0.67,0.18,0.55],'MarkerEdgeColor','k');
    quiver(ax,geom.qStar(1),geom.qStar(2), ...
        0.055*geom.normal(1),0.055*geom.normal(2),0, ...
        'Color',[0.12,0.55,0.25], 'LineWidth',1.0, 'MaxHeadSize',0.65);
    addCriticalLabelsLocal(ax,geom,cfg);
    title(ax,sprintf('Critical close-up | case %d | $d_{min}=%.3f$', ...
        caseId,common.dMin), 'Interpreter','latex', ...
        'FontName',cfg.fontName, 'FontSize',11, 'FontWeight','normal');
end

function geom = criticalGeometryLocal(state)
    i = state.minIdx;
    geom.M = state.M(i,:);
    geom.N = state.N(i,:);
    geom.qStar = state.closestPoint(i,:);
    geom.alpha = state.closestAlpha(i);
    geom.normal = state.closestNormal(i,:);
    geom.normal = geom.normal/max(norm(geom.normal),eps);
    geom.clearance = state.clearance(i);
    geom.qBoundary = geom.qStar-geom.clearance*geom.normal;
end

function limits = closeupLimitsLocal(geom)
    points = [geom.M;geom.N;geom.qStar;geom.qBoundary; ...
        geom.qStar+0.055*geom.normal];
    center = 0.5*(min(points,[],1)+max(points,[],1));
    span = max(max(points,[],1)-min(points,[],1)+[0.075,0.075], ...
        [0.20,0.18]);
    limits = [center(1)+[-0.5,0.5]*span(1); ...
        center(2)+[-0.5,0.5]*span(2)];
end

function addCriticalLabelsLocal(ax,geom,cfg)
    e = geom.N-geom.M; e = e/max(norm(e),eps);
    p = [-e(2),e(1)];
    if dot(p,geom.normal)<0; p=-p; end
    text(ax,geom.M(1)-0.012,geom.M(2)+0.018,'$\mathbf{M}(u)$', ...
        'Interpreter','latex','FontName',cfg.fontName,'FontSize',9);
    text(ax,geom.N(1)+0.008,geom.N(2)+0.018,'$\mathbf{N}(u)$', ...
        'Interpreter','latex','FontName',cfg.fontName,'FontSize',9);
    mid=0.5*(geom.M+geom.N)+0.018*p;
    text(ax,mid(1),mid(2),'$L$','Interpreter','latex', ...
        'FontName',cfg.fontName,'FontSize',9,'HorizontalAlignment','center');
    text(ax,geom.qStar(1)+0.012*p(1),geom.qStar(2)+0.012*p(2), ...
        '$\mathbf{q}^{\star}$','Interpreter','latex', ...
        'FontName',cfg.fontName,'FontSize',9,'Color',[0.67,0.18,0.55]);
    alphaPos=geom.M+0.55*geom.alpha*(geom.N-geom.M)-0.016*p;
    text(ax,alphaPos(1),alphaPos(2),'$\alpha^{\star}$', ...
        'Interpreter','latex','FontName',cfg.fontName,'FontSize',9);
    cPos=0.5*(geom.qStar+geom.qBoundary)+ ...
        0.012*[-geom.normal(2),geom.normal(1)];
    text(ax,cPos(1),cPos(2),'$c(u)$','Interpreter','latex', ...
        'FontName',cfg.fontName,'FontSize',9,'Color',[0.00,0.52,0.55]);
    nPos=geom.qStar+0.058*geom.normal;
    text(ax,nPos(1),nPos(2),'$\mathbf{n}$','Interpreter','latex', ...
        'FontName',cfg.fontName,'FontSize',9,'Color',[0.12,0.55,0.25]);
end

function setupSceneLocal(ax,caseState,cfg)
    hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    drawObstaclesLocal(ax,caseState.obstacles);
    b=caseState.envInfo.bounds;
    plot(ax,[b(1,1),b(1,2),b(1,2),b(1,1),b(1,1)], ...
        [b(2,1),b(2,1),b(2,2),b(2,2),b(2,1)],'k-','LineWidth',0.9);
    span=[diff(b(1,:)),diff(b(2,:))];
    xlim(ax,b(1,:)+[-1,1]*0.015*span(1));
    ylim(ax,b(2,:)+[-1,1]*0.015*span(2));
    set(ax,'FontName',cfg.fontName,'Color','w');
end

function drawObstaclesLocal(ax,obstacles)
    for i=1:numel(obstacles)
        obs=obstacles(i);
        switch lower(char(obs.type))
            case 'circle'
                th=linspace(0,2*pi,100).';
                xy=obs.center+obs.radius*[cos(th),sin(th)];
            case {'rect','rectangle','box'}
                if isfield(obs,'vertices') && ~isempty(obs.vertices)
                    xy=obs.vertices;
                else
                    local=[-1,-1;1,-1;1,1;-1,1].*obs.halfSize;
                    R=[cos(obs.yaw),-sin(obs.yaw);sin(obs.yaw),cos(obs.yaw)];
                    xy=local*R.'+obs.center;
                end
            case 'polygon'
                xy=obs.vertices;
            otherwise
                continue;
        end
        patch(ax,xy(:,1),xy(:,2),[0.76,0.78,0.80], ...
            'FaceAlpha',0.90,'EdgeColor',[0.16,0.17,0.18], ...
            'LineWidth',0.7);
    end
end

function drawStartGoalLocal(ax,envInfo,cfg)
    text(ax,envInfo.startPt(1),envInfo.startPt(2),'S', ...
        'FontName',cfg.fontName,'FontSize',10,'FontWeight','bold', ...
        'Color',[0.08,0.58,0.55],'HorizontalAlignment','center', ...
        'BackgroundColor','w','Margin',0.3);
    text(ax,envInfo.goalPt(1),envInfo.goalPt(2),'G', ...
        'FontName',cfg.fontName,'FontSize',10,'FontWeight','bold', ...
        'Color',[0.93,0.64,0.12],'HorizontalAlignment','center', ...
        'BackgroundColor','w','Margin',0.3);
end

function drawFailurePanel(ax,spec,common,cfg,caseId,panelId,message)
    hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    b=common.bounds;
    xlim(ax,b(1,:)); ylim(ax,b(2,:));
    text(ax,mean(b(1,:)),mean(b(2,:)), ...
        sprintf('%s\ncase %d failed\n%s',char(spec.name),caseId,message), ...
        'FontName',cfg.fontName,'FontSize',9, ...
        'HorizontalAlignment','center','Interpreter','none');
end

function [envSeeds,plannerSeeds] = sampleSeedPairs(cfg)
    rng('shuffle');
    if isempty(cfg.envSeeds)
        population=cfg.envSeedRange(1):cfg.envSeedRange(2);
        if numel(population)<cfg.numCases
            error('cfg.envSeedRange contains fewer than cfg.numCases seeds.');
        end
        ids=randperm(numel(population),cfg.numCases);
        envSeeds=population(ids);
    else
        envSeeds=round(cfg.envSeeds(:).');
        if numel(envSeeds)~=cfg.numCases
            error('cfg.envSeeds must contain exactly cfg.numCases values.');
        end
    end
    plannerSeeds=randi(cfg.plannerSeedRange,cfg.numCases,1).';
end

function spec = selectSceneSpecLocal(common,sceneId,difficultyId)
    specs=makeCSSCBenchmarkSceneSpecs2D(struct( ...
        'bounds',common.bounds,'maxEnvSeedTry',1));
    sceneIndex=find(strcmpi(string({specs.id}),string(sceneId)),1);
    level=find(strcmpi(common.difficultyIds,string(difficultyId)),1);
    if isempty(sceneIndex)||isempty(level)
        error('Unknown sceneId or difficultyId.');
    end
    spec=specs(sceneIndex);
    switch lower(char(spec.id))
        case 'double_slit'
            spec.gapHeight=common.doubleSlit.dGap(level);
            spec.xWalls=common.doubleSlit.xWalls;
            spec.gapCenterRange=common.doubleSlit.centerRange(level,:);
            spec.wallThickness=common.doubleSlit.wallThickness;
        case 's_channel'
            spec.dGap=common.sChannel.dGap(level);
            spec.channelW=common.sChannel.W;
            spec.channelH=common.sChannel.H;
        case 'staggered_baffles'
            spec.passageWidth=common.baffles.passageWidth(level);
            spec.baffleXCenters=common.baffles.xCenters;
            spec.baffleThickness=common.baffles.thickness;
            spec.bafflePattern=common.baffles.pattern;
        case 'random_mixed'
            spec.nCircle=common.randomMixed.nCircle(level);
            spec.nRect=common.randomMixed.nRect(level);
            spec.minGap=common.randomMixed.minGap(level);
            spec.radiusRange=common.randomMixed.radiusRange;
            spec.halfSizeXRange=common.randomMixed.halfSizeXRange;
            spec.halfSizeYRange=common.randomMixed.halfSizeYRange;
            spec.yawRange=common.randomMixed.yawRange;
            spec.keepoutStart=common.randomMixed.keepoutStart;
            spec.keepoutGoal=common.randomMixed.keepoutGoal;
            spec.maxTry=common.randomMixed.maxTry;
    end
end
