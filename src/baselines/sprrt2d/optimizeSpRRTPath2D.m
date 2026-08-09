function [pathOptimized, info] = ...
        optimizeSpRRTPath2D(pathRaw, obstacles, opts)
%OPTIMIZESPRRTPATH2D Deterministic Adaptive Sp-RRT path pruning.
%
% The default non-worsening-angle mode accepts qa-qb only when the segment
% is collision-free and the new turns at both retained endpoints do not
% exceed their corresponding pre-shortcut turns.

    info = initInfo(pathRaw);
    pathOptimized = pathRaw;

    if size(pathRaw,1) < 3 || ~opts.pathOptimization.enable
        info = finalizeInfo(pathRaw, pathOptimized, obstacles, opts, info);
        return;
    end

    n = size(pathRaw,1);
    retained = 1;
    current = 1;

    while current < n
        best = current + 1;
        firstInfeasible = false;

        for candidate = current+2:n
            info.shortcutCandidateCount = ...
                info.shortcutCandidateCount + 1;
            [feasible, detail] = feasibleShortcut( ...
                pathRaw, retained, current, candidate, obstacles, opts);
            info.lastCandidateDetail = detail;

            if feasible
                best = candidate;
            else
                firstInfeasible = true;
                break;
            end
        end

        if best > current + 1
            info.shortcutAcceptedCount = ...
                info.shortcutAcceptedCount + 1;
        end
        retained(end+1,1) = best; %#ok<AGROW>
        current = best;

        if ~firstInfeasible && current >= n
            break;
        end
    end

    if retained(end) ~= n
        retained(end+1,1) = n;
    end
    pathOptimized = pathRaw(retained,:);
    info.retainedIndices = retained;
    info = finalizeInfo(pathRaw, pathOptimized, obstacles, opts, info);
end

function [feasible, detail] = feasibleShortcut( ...
        path, retained, indexA, indexB, obstacles, opts)
    qa = path(indexA,:);
    qb = path(indexB,:);
    collisionOpts = struct( ...
        'bounds', opts.bounds, ...
        'collisionResolution', opts.collisionResolution, ...
        'inflateRadius', opts.inflateRadius);

    detail = struct('collisionFree', false, ...
        'thetaAOld', nan, 'thetaANew', nan, ...
        'thetaBOld', nan, 'thetaBNew', nan, ...
        'angleAOK', true, 'angleBOK', true);

    detail.collisionFree = ...
        isSegmentCollisionFree2D(qa, qb, obstacles, collisionOpts);
    if ~detail.collisionFree
        feasible = false;
        return;
    end

    if numel(retained) >= 2
        qPrev = path(retained(end-1),:);
        detail.thetaAOld = turnAngle(qPrev - qa, path(indexA+1,:) - qa);
        detail.thetaANew = turnAngle(qPrev - qa, qb - qa);
        detail.angleAOK = angleCriterion( ...
            detail.thetaANew, detail.thetaAOld, opts);
    end

    if indexB < size(path,1)
        qNext = path(indexB+1,:);
        detail.thetaBOld = turnAngle( ...
            path(indexB-1,:) - qb, qNext - qb);
        detail.thetaBNew = turnAngle(qa - qb, qNext - qb);
        detail.angleBOK = angleCriterion( ...
            detail.thetaBNew, detail.thetaBOld, opts);
    end

    feasible = detail.angleAOK && detail.angleBOK;
end

function ok = angleCriterion(thetaNew, thetaOld, opts)
    if ~isfinite(thetaNew)
        ok = false;
        return;
    end

    mode = lower(string(opts.pathOptimization.mode));
    switch mode
        case "non-worsening-angle"
            ok = ~isfinite(thetaOld) || ...
                thetaNew <= thetaOld + ...
                opts.pathOptimization.angleTolerance;
        case "theta-limit"
            ok = thetaNew <= opts.pathOptimization.thetaMax + ...
                opts.pathOptimization.angleTolerance;
        otherwise
            error('Unknown Sp-RRT path optimization mode: %s', char(mode));
    end
end

function theta = turnAngle(vTowardPrevious, vTowardNext)
    % Both vectors point away from the vertex. Straight travel therefore has
    % an angle pi between them; convert to the path turning convention.
    denom = norm(vTowardPrevious) * norm(vTowardNext);
    if denom <= eps
        theta = nan;
        return;
    end
    cosine = dot(vTowardPrevious, vTowardNext) / denom;
    cosine = max(-1, min(1, cosine));
    theta = pi - acos(cosine);
end

function info = initInfo(pathRaw)
    info = struct();
    info.method = 'Adaptive Sp-RRT-2D deterministic path pruning';
    info.mode = '';
    info.rawPath = pathRaw;
    info.optimizedPath = pathRaw;
    info.retainedIndices = (1:size(pathRaw,1)).';
    info.shortcutCandidateCount = 0;
    info.shortcutAcceptedCount = 0;
    info.rawPathLength = nan;
    info.optimizedPathLength = nan;
    info.rawMaxTurn = nan;
    info.optimizedMaxTurn = nan;
    info.validationPassed = false;
    info.validation = [];
    info.lastCandidateDetail = [];
end

function info = finalizeInfo(pathRaw, pathOptimized, obstacles, opts, info)
    rawValidation = validateSpRRTPolyline2D(pathRaw, obstacles, opts);
    optimizedValidation = ...
        validateSpRRTPolyline2D(pathOptimized, obstacles, opts);
    info.mode = char(opts.pathOptimization.mode);
    info.rawPath = pathRaw;
    info.optimizedPath = pathOptimized;
    info.rawPathLength = rawValidation.pathLength;
    info.optimizedPathLength = optimizedValidation.pathLength;
    info.rawMaxTurn = rawValidation.maxTurn;
    info.optimizedMaxTurn = optimizedValidation.maxTurn;
    info.validationPassed = optimizedValidation.valid && ...
        optimizedValidation.pathLength <= ...
        rawValidation.pathLength + opts.lengthTolerance;
    info.validation = optimizedValidation;
end
