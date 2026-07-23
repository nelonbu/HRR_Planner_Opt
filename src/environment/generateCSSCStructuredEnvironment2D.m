function [obstacles, envInfo] = generateCSSCStructuredEnvironment2D(sceneType, opts)
%GENERATECSSCSTRUCTUREDENVIRONMENT2D Generate structured 2D benchmark scenes.
%
% Usage:
%   [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('singleSlit', opts);
%   [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('offsetDoubleSlit', opts);
%   [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('fourRectSChannel', opts);
%   [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('staggeredBaffles3', opts);
%
% Key opts:
%   common              : bounds, startPt, goalPt
%   singleSlit          : xWall, gapCenterY, gapHeight, wallThickness
%   offsetDoubleSlit    : xWalls, gapCentersY, gapHeight, wallThickness
%   fourRectSChannel    : dGap, W/channelW, H/channelH, seed, yInOutRange
%   staggeredBaffles3   : centers, halfSize

    if nargin < 1 || isempty(sceneType)
        sceneType = 'singleSlit';
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    opts = setCommonDefaults(opts);
    sceneKey = lower(char(sceneType));

    switch sceneKey
        case {'singleslit','single_slit','slit'}
            [obstacles, envInfo] = makeSingleSlitScene(opts);

        case {'offsetdoubleslit','offset_double_slit','double_slit'}
            [obstacles, envInfo] = makeOffsetDoubleSlitScene(opts);

        case {'fourrectschannel','four_rect_s_channel','s_channel','s'}
            [obstacles, envInfo] = makeFourRectSChannelScene(opts);

        case {'staggeredbaffles3','staggered_baffles_3','baffles','three_baffles'}
            [obstacles, envInfo] = makeStaggeredBafflesScene(opts);

        otherwise
            error('Unknown structured scene type: %s', char(sceneType));
    end

    envInfo.type = sceneKey;
    envInfo.category = 'structured';
    envInfo.bounds = opts.bounds;
    envInfo.startPt = opts.startPt;
    envInfo.goalPt = opts.goalPt;
    envInfo.numObstacles = numel(obstacles);
end

function opts = setCommonDefaults(opts)
    if ~isfield(opts, 'bounds'); opts.bounds = [0, 1; -0.4, 0.4]; end
    if ~isfield(opts, 'startPt'); opts.startPt = [0.05, 0.0]; end
    if ~isfield(opts, 'goalPt'); opts.goalPt = [0.95, 0.0]; end

    opts.startPt = opts.startPt(:).';
    opts.goalPt = opts.goalPt(:).';
end

function [obstacles, envInfo] = makeSingleSlitScene(opts)
    xWall = getOpt(opts, 'xWall', 0.50);
    gapCenterY = getOpt(opts, 'gapCenterY', 0.00);
    gapHeight = getOpt(opts, 'gapHeight', 0.11);
    wallThickness = getOpt(opts, 'wallThickness', 0.08);

    obstacles = emptyObstacleArray();
    obstacles = addVerticalWallWithGap(obstacles, xWall, opts.bounds(2,1), opts.bounds(2,2), ...
        gapCenterY, gapHeight, wallThickness);

    envInfo = baseInfo(opts);
    envInfo.id = 'single_slit';
    envInfo.name = 'Single Slit';
    envInfo.description = 'One wall with a centered narrow passage.';
    envInfo.obstacleLabels = {};
    envInfo.params.xWall = xWall;
    envInfo.params.gapCenterY = gapCenterY;
    envInfo.params.gapHeight = gapHeight;
    envInfo.params.wallThickness = wallThickness;
end

function [obstacles, envInfo] = makeOffsetDoubleSlitScene(opts)
    xWalls = getOpt(opts, 'xWalls', [0.32, 0.68]);
    gapCentersY = getOpt(opts, 'gapCentersY', [-0.13, 0.15]);
    gapHeight = getOpt(opts, 'gapHeight', 0.11);
    wallThickness = getOpt(opts, 'wallThickness', 0.08);

    obstacles = emptyObstacleArray();
    for i = 1:numel(xWalls)
        obstacles = addVerticalWallWithGap(obstacles, xWalls(i), ...
            opts.bounds(2,1), opts.bounds(2,2), ...
            gapCentersY(i), gapHeight, wallThickness);
    end

    envInfo = baseInfo(opts);
    envInfo.id = 'offset_double_slit';
    envInfo.name = 'Offset Double Slit';
    envInfo.description = 'Two wall gaps with opposite vertical offsets.';
    envInfo.obstacleLabels = {};
    envInfo.params.xWalls = xWalls;
    envInfo.params.gapCentersY = gapCentersY;
    envInfo.params.gapHeight = gapHeight;
    envInfo.params.wallThickness = wallThickness;
end

function [obstacles, envInfo] = makeFourRectSChannelScene(opts)
    dGap = getOpt(opts, 'dGap', 0.05);
    channelW = getOpt(opts, 'W', getOpt(opts, 'channelW', 0.64));
    defaultH = opts.bounds(2,2) - opts.bounds(2,1);
    channelH = getOpt(opts, 'H', getOpt(opts, 'channelH', defaultH));
    seed = getOpt(opts, 'seed', 31);
    yRange = getOpt(opts, 'yInOutRange', [-0.20, 0.20]);

    [obstacles, geom] = makeSChannel4Rect(opts.bounds, dGap, channelW, channelH, seed, yRange);

    envInfo = baseInfo(opts);
    envInfo.id = 'four_rect_s_channel';
    envInfo.name = 'Four-Rect S Channel';
    envInfo.description = sprintf( ...
        'S channel: dGap=%.3f, W=%.3f, H=%.3f, L1=%.3f, L2=%.3f, L3=%.3f.', ...
        dGap, geom.W, geom.H, geom.L1, geom.L2, geom.L3);
    envInfo.obstacleLabels = {'1','2','3','4'};
    envInfo.params = geom;
    envInfo.params.seed = seed;
    envInfo.params.yInOutRange = yRange;
end

function [obstacles, envInfo] = makeStaggeredBafflesScene(opts)
    centers = getOpt(opts, 'centers', [0.25, 0.115; 0.50, -0.115; 0.75, 0.115]);
    halfSize = getOpt(opts, 'halfSize', [0.025, 0.185]);

    obstacles = emptyObstacleArray();
    for i = 1:size(centers, 1)
        obstacles(end+1,1) = obsRect2D(centers(i,:), halfSize, 0.0); %#ok<AGROW>
    end

    envInfo = baseInfo(opts);
    envInfo.id = 'staggered_baffles_3';
    envInfo.name = 'Three Staggered Baffles';
    envInfo.description = 'Three longer alternating baffles, arranged top-bottom-top.';
    envInfo.obstacleLabels = {'1','2','3'};
    envInfo.params.centers = centers;
    envInfo.params.halfSize = halfSize;
end

function [obstacles, geom] = makeSChannel4Rect(bounds, dGap, totalW, totalH, seed, yRandRange)
    xCenter = mean(bounds(1,:));
    yCenter = mean(bounds(2,:));
    xLeft = xCenter - totalW/2;
    xRight = xCenter + totalW/2;
    yBottom = yCenter - totalH/2;
    yTop = yCenter + totalH/2;

    tol = 1e-12;
    if xLeft < bounds(1,1) - tol || xRight > bounds(1,2) + tol || ...
       yBottom < bounds(2,1) - tol || yTop > bounds(2,2) + tol
        error('The requested W/H does not fit inside the scenario bounds.');
    end

    rngState = rng;
    cleanupObj = onCleanup(@() rng(rngState)); %#ok<NASGU>
    rng(seed, 'twister');

    minL13 = 0.08;
    l1Max = totalW - minL13;
    if l1Max <= minL13
        error('Cannot keep L1 and L3 positive.');
    end

    L1 = minL13 + rand * (l1Max - minL13);
    L3 = totalW - L1;

    yFeasible = [yBottom + 0.5*dGap, yTop - 0.5*dGap];
    yRange = [max(yRandRange(1), yFeasible(1)), min(yRandRange(2), yFeasible(2))];
    if yRange(1) >= yRange(2)
        error('No feasible y_in/y_out range for the requested dGap and H.');
    end

    yIn = yRange(1) + rand * (yRange(2) - yRange(1));
    yOut = yRange(1) + rand * (yRange(2) - yRange(1));
    yD = yBottom;
    L2 = yOut - yIn;

    w1 = L1 - 0.5*dGap;
    w2 = L3 + 0.5*dGap;
    w3 = L1 + 0.5*dGap;
    w4 = L3 - 0.5*dGap;

    swappedByL2 = L2 < 0;
    if swappedByL2
        tmp = w1;
        w1 = w3;
        w3 = tmp;

        tmp = w2;
        w2 = w4;
        w4 = tmp;
    end

    h3 = yIn - yD - 0.5*dGap;
    h4 = yOut - yD - 0.5*dGap;
    h1 = totalH - h3 - dGap;
    h2 = totalH - h4 - dGap;

    if min([L1, L3, w1, w2, w3, w4, h1, h2, h3, h4]) <= 0
        error('dGap is too large for the four-rectangle S channel.');
    end

    x1 = [xLeft, xLeft + w1];
    x2 = [xRight - w2, xRight];
    x3 = [xLeft, xLeft + w3];
    x4 = [xRight - w4, xRight];

    y1 = [yTop - h1, yTop];
    y2 = [yTop - h2, yTop];
    y3 = [yBottom, yBottom + h3];
    y4 = [yBottom, yBottom + h4];

    obstacles = [
        rectFromEdges(x1(1), x1(2), y1(1), y1(2))
        rectFromEdges(x2(1), x2(2), y2(1), y2(2))
        rectFromEdges(x3(1), x3(2), y3(1), y3(2))
        rectFromEdges(x4(1), x4(2), y4(1), y4(2))
    ];

    geom = struct();
    geom.W = totalW;
    geom.H = totalH;
    geom.dGap = dGap;
    geom.L1 = L1;
    geom.L2 = L2;
    geom.L3 = L3;
    geom.yIn = yIn;
    geom.yOut = yOut;
    geom.yD = yD;
    geom.swappedByL2 = swappedByL2;
end

function obstacles = addVerticalWallWithGap(obstacles, xWall, yMin, yMax, gapCenterY, gapHeight, wallThickness)
    gapLow = max(yMin, gapCenterY - gapHeight/2);
    gapHigh = min(yMax, gapCenterY + gapHeight/2);

    lowerLen = gapLow - yMin;
    upperLen = yMax - gapHigh;

    if lowerLen > wallThickness
        center = [xWall, yMin + lowerLen/2];
        halfSize = [wallThickness/2, lowerLen/2];
        obstacles(end+1,1) = obsRect2D(center, halfSize, 0.0);
    end

    if upperLen > wallThickness
        center = [xWall, gapHigh + upperLen/2];
        halfSize = [wallThickness/2, upperLen/2];
        obstacles(end+1,1) = obsRect2D(center, halfSize, 0.0);
    end
end

function obs = rectFromEdges(xMin, xMax, yMin, yMax)
    center = [(xMin + xMax)/2, (yMin + yMax)/2];
    halfSize = [(xMax - xMin)/2, (yMax - yMin)/2];
    obs = obsRect2D(center, halfSize, 0.0);
end

function val = getOpt(opts, name, defaultVal)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = defaultVal;
    end
end

function envInfo = baseInfo(opts)
    envInfo = struct();
    envInfo.bounds = opts.bounds;
    envInfo.startPt = opts.startPt;
    envInfo.goalPt = opts.goalPt;
end

function obstacles = emptyObstacleArray()
    obstacles = repmat(obsCircle2D([0, 0], 0), 0, 1);
end
