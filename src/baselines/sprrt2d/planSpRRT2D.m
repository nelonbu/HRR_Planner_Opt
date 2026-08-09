function [pathOptimized, info] = ...
        planSpRRT2D(startPt, goalPt, obstacles, opts)
%PLANSPRRT2D Adaptive reverse-tree Sp-RRT for a 2-D FTL leader path.
%
% The tree grows goalPt -> startPt. Root depth is zero, and node depth is
% the number of fixed-length leader-path segments from the goal root. It is
% not a physical robot-link index. Branches grow naturally up to
% maxSegmentCount without restarting at nominalLinkCount. The first root
% edge has no turn constraint because terminal direction is disabled.

    if nargin < 4
        opts = struct();
    end
    opts = validateSpRRT2DParams(opts, startPt, goalPt);
    startPt = startPt(:).';
    goalPt = goalPt(:).';

    pathOptimized = zeros(0, 2);
    info = initInfo(opts, startPt, goalPt);
    tPlanning = tic;

    info.directDistance = norm(goalPt - startPt);
    info.nominalTotalLength = opts.nominalLinkCount * opts.L;
    info.maximumAllowedLength = opts.maxSegmentCount * opts.L;
    info.totalRobotLength = info.nominalTotalLength;
    info.reachabilityMargin = ...
        info.maximumAllowedLength - info.directDistance;

    if info.directDistance > ...
            info.maximumAllowedLength + opts.reachabilityTolerance
        info.terminationReason = 'structurallyUnreachable';
        info.message = ['Direct distance exceeds the maximum path length ' ...
            'allowed by maxSegmentCount.'];
        info.planningTimeSec = toc(tPlanning);
        return;
    end

    zeroMargin = abs(info.reachabilityMargin) <= ...
        opts.reachabilityTolerance;
    if zeroMargin
        info.structuralWarning = true;
        info.warningMessage = ['The maximum 12-segment path budget can ' ...
            'connect start and goal only as a straight path; any detour ' ...
            'exceeds maximumAllowedLength.'];
        emitWarning(opts, info.warningMessage);
    end

    if ~insideBounds(startPt, opts.bounds) || ...
            ~insideBounds(goalPt, opts.bounds) || ...
            ~isPointCollisionFree2D(startPt, obstacles, opts.inflateRadius) || ...
            ~isPointCollisionFree2D(goalPt, obstacles, opts.inflateRadius)
        info.terminationReason = 'invalidStartGoal';
        info.message = 'Start or goal is outside bounds or in collision.';
        info.planningTimeSec = toc(tPlanning);
        return;
    end

    collisionOpts = struct( ...
        'bounds', opts.bounds, ...
        'collisionResolution', opts.collisionResolution, ...
        'inflateRadius', opts.inflateRadius);
    if zeroMargin && ~isSegmentCollisionFree2D( ...
            goalPt, startPt, obstacles, collisionOpts)
        info.terminationReason = 'zeroMarginStraightPathBlocked';
        info.warningMessage = [info.warningMessage ' The unique straight ' ...
            'path is blocked by an obstacle.'];
        info.message = info.warningMessage;
        info.planningTimeSec = toc(tPlanning);
        return;
    end

    rng(opts.seed, 'twister');
    tree = struct();
    tree.position = goalPt;
    tree.parent = 0;
    tree.depth = 0;
    tree.cost = 0;

    finalIdx = 0;
    for iter = 1:opts.maxIter
        if toc(tPlanning) >= opts.maxTimeSec
            info.terminationReason = 'maxTimeSec';
            break;
        end
        info.numIter = iter;

        if rand < opts.entranceBias
            qRand = startPt;
        else
            qRand = sampleBounds(opts.bounds);
        end

        [tree, newIdx, expandStats] = ...
            expandSpRRT2D(tree, qRand, obstacles, opts);
        info = accumulateExpandStats(info, expandStats);
        if newIdx == 0
            continue;
        end
        info.maxDepthReached = max( ...
            info.maxDepthReached, tree.depth(newIdx));

        distanceToStart = norm(tree.position(newIdx,:) - startPt);
        if distanceToStart <= opts.duplicateTolerance
            finalIdx = newIdx;
        elseif tree.depth(newIdx) < opts.maxSegmentCount
            remainingLength = opts.L;
            if distanceToStart <= remainingLength + opts.lengthTolerance
                info.startConnectionAttemptCount = ...
                    info.startConnectionAttemptCount + 1;
                [accepted, detail] = connectSpRRT2D( ...
                    tree, newIdx, startPt, remainingLength, true, ...
                    obstacles, opts);
                info = accumulateConnectDetail(info, detail);
                if accepted
                    tree.position(end+1,:) = startPt;
                    tree.parent(end+1,1) = newIdx;
                    tree.depth(end+1,1) = detail.newDepth;
                    tree.cost(end+1,1) = ...
                        tree.cost(newIdx) + detail.edgeLength;
                    finalIdx = size(tree.position, 1);
                    info.maxDepthReached = max( ...
                        info.maxDepthReached, detail.newDepth);
                end
            end
        end

        if finalIdx > 0
            break;
        end
    end

    info.numNodes = size(tree.position, 1);
    info.averageCandidatesPerExpand = ...
        info.candidateAttemptCount / max(1, info.expandCallCount);

    if finalIdx == 0
        if strcmp(info.terminationReason, 'running')
            info.terminationReason = 'maxIter';
        end
        info.message = sprintf('Adaptive Sp-RRT-2D failed: %s.', ...
            info.terminationReason);
        info.planningTimeSec = toc(tPlanning);
        if opts.storeTree
            info.tree = tree;
        end
        return;
    end

    rawPath = backtrackStartToGoal(tree, finalIdx);
    rawValidation = validateSpRRTPolyline2D(rawPath, obstacles, opts);
    info.rawPath = rawPath;
    info.rawPathLength = rawValidation.pathLength;
    info.rawMaxTurn = rawValidation.maxTurn;
    info.finalRawSegmentCount = size(rawPath,1) - 1;
    info.usedExtraSegments = ...
        info.finalRawSegmentCount > opts.nominalLinkCount;
    info.numberOfExtraSegments = max(0, ...
        info.finalRawSegmentCount - opts.nominalLinkCount);
    info.rawPathTurnLimitSatisfied = ...
        rawValidation.turnLimitSatisfied;
    info.rawPathCollisionFree = rawValidation.collisionFree;

    if ~rawValidation.valid || ...
            size(rawPath,1)-1 > opts.maxSegmentCount
        info.terminationReason = 'rawPathValidationFailure';
        info.message = 'Generated raw path failed final validation.';
        info.planningTimeSec = toc(tPlanning);
        if opts.storeTree
            info.tree = tree;
        end
        return;
    end

    tOptimization = tic;
    [pathCandidate, optimizationInfo] = ...
        optimizeSpRRTPath2D(rawPath, obstacles, opts);
    info.pathOptimizationTimeSec = toc(tOptimization);
    info.pathOptimization = optimizationInfo;

    if ~optimizationInfo.validationPassed
        info.terminationReason = 'pathOptimizationFailure';
        info.message = 'Deterministic path pruning failed validation.';
        info.planningTimeSec = toc(tPlanning);
        if opts.storeTree
            info.tree = tree;
        end
        return;
    end

    pathOptimized = pathCandidate;
    info.success = true;
    info.terminationReason = 'success';
    info.message = ['Adaptive Sp-RRT-2D found and deterministically ' ...
        'pruned a leader path.'];
    info.firstSolutionIter = info.numIter;
    info.optimizedPath = pathOptimized;
    info.optimizedPathLength = optimizationInfo.optimizedPathLength;
    info.optimizedMaxTurn = optimizationInfo.optimizedMaxTurn;
    info.finalOptimizedWaypointCount = size(pathOptimized,1);
    info.shortcutCandidateCount = ...
        optimizationInfo.shortcutCandidateCount;
    info.shortcutAcceptedCount = ...
        optimizationInfo.shortcutAcceptedCount;
    info.planningTimeSec = toc(tPlanning);
    if opts.storeTree
        info.tree = tree;
    end
end

function info = initInfo(opts, startPt, goalPt)
    info = struct();
    info.method = 'Adaptive Sp-RRT-2D';
    info.success = false;
    info.terminationReason = 'running';
    info.message = '';
    info.warningMessage = '';
    info.structuralWarning = false;
    info.startPt = startPt;
    info.goalPt = goalPt;
    info.nominalLinkCount = opts.nominalLinkCount;
    info.maxSegmentCount = opts.maxSegmentCount;
    info.nLinks = opts.nominalLinkCount;
    info.L = opts.L;
    info.thetaMax = opts.thetaMax;
    info.linkLengths = opts.linkLengths;
    info.thetaLimits = opts.thetaLimits;
    info.directDistance = nan;
    info.nominalTotalLength = nan;
    info.maximumAllowedLength = nan;
    info.totalRobotLength = nan;
    info.reachabilityMargin = nan;
    info.numIter = 0;
    info.numNodes = 1;
    info.firstSolutionIter = nan;
    info.planningTimeSec = 0;
    info.pathOptimizationTimeSec = 0;
    info.expandCallCount = 0;
    info.expandSuccessCount = 0;
    info.expandFailureCount = 0;
    info.candidateAttemptCount = 0;
    info.abandonedCandidateCount = 0;
    info.averageCandidatesPerExpand = nan;
    info.angleRejectCount = 0;
    info.collisionRejectCount = 0;
    info.boundsRejectCount = 0;
    info.lengthRejectCount = 0;
    info.depthRejectCount = 0;
    info.duplicateRejectCount = 0;
    info.startConnectionAttemptCount = 0;
    info.maxDepthReached = 0;
    info.maxGeneratedTurn = 0;
    info.rawPath = zeros(0,2);
    info.optimizedPath = zeros(0,2);
    info.rawPathLength = nan;
    info.optimizedPathLength = nan;
    info.finalRawSegmentCount = 0;
    info.finalOptimizedWaypointCount = 0;
    info.usedExtraSegments = false;
    info.numberOfExtraSegments = 0;
    info.rawMaxTurn = nan;
    info.optimizedMaxTurn = nan;
    info.rawPathTurnLimitSatisfied = false;
    info.rawPathCollisionFree = false;
    info.shortcutCandidateCount = 0;
    info.shortcutAcceptedCount = 0;
    info.pathOptimization = [];
    info.tree = [];
end

function info = accumulateExpandStats(info, stats)
    fields = {
        'expandCallCount'
        'expandSuccessCount'
        'expandFailureCount'
        'candidateAttemptCount'
        'abandonedCandidateCount'
        'angleRejectCount'
        'collisionRejectCount'
        'boundsRejectCount'
        'lengthRejectCount'
        'depthRejectCount'
        'duplicateRejectCount'
    };
    for i = 1:numel(fields)
        name = fields{i};
        info.(name) = info.(name) + stats.(name);
    end
    info.maxGeneratedTurn = max( ...
        info.maxGeneratedTurn, stats.maxGeneratedTurn);
end

function info = accumulateConnectDetail(info, detail)
    if isfinite(detail.theta)
        info.maxGeneratedTurn = max(info.maxGeneratedTurn, detail.theta);
    end
    switch lower(detail.reason)
        case 'angle'
            info.angleRejectCount = info.angleRejectCount + 1;
        case 'collision'
            info.collisionRejectCount = info.collisionRejectCount + 1;
        case 'bounds'
            info.boundsRejectCount = info.boundsRejectCount + 1;
        case 'length'
            info.lengthRejectCount = info.lengthRejectCount + 1;
        case 'depth'
            info.depthRejectCount = info.depthRejectCount + 1;
    end
end

function path = backtrackStartToGoal(tree, finalIdx)
    indices = finalIdx;
    while tree.parent(indices(end)) ~= 0
        indices(end+1) = tree.parent(indices(end)); %#ok<AGROW>
    end
    path = tree.position(indices,:);
end

function q = sampleBounds(bounds)
    q = [
        bounds(1,1) + rand * (bounds(1,2) - bounds(1,1)), ...
        bounds(2,1) + rand * (bounds(2,2) - bounds(2,1))
    ];
end

function tf = insideBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
        q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end

function emitWarning(opts, message)
    if opts.emitWarnings
        warning('SpRRT2D:StructuralReachability', '%s', message);
    end
end
