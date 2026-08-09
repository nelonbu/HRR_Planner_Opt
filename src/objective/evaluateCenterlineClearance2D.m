function state = evaluateCenterlineClearance2D(P, obstacles, params)
%EVALUATECENTERLINECLEARANCE2D Sample B-spline point clearance for ablation.
%
% This evaluator intentionally ignores fixed-length chords. It exists only
% for the Point-SDF ablation and must not be used for final safety metrics.

    tAll = tic;
    nCtrl = size(P,1);
    degree = params.degree;
    if isfield(params, 'knot') && ~isempty(params.knot)
        knot = params.knot;
    else
        knot = makeClampedUniformKnot(nCtrl, degree);
    end
    if isfield(params, 'envOpts') && isfield(params.envOpts, 'uRange')
        uRange = params.envOpts.uRange;
    else
        uRange = [0, 1];
    end
    if isfield(params, 'envOpts') && isfield(params.envOpts, 'nU')
        nU = params.envOpts.nU;
    else
        nU = 140;
    end

    u = linspace(uRange(1), uRange(2), nU).';
    points = evalBSplinePath2D(P, u, degree, knot);
    clearance = nan(nU,1);
    normal = nan(nU,2);
    nearestObsId = nan(nU,1);
    nearestObsType = strings(nU,1);

    for i = 1:nU
        [clearance(i), normal(i,:), obsInfo] = ...
            queryObstaclePointSDF2D(points(i,:), obstacles);
        nearestObsId(i) = obsInfo.id;
        nearestObsType(i) = string(obsInfo.type);
    end

    valid = ~isnan(clearance);
    [minClear, minIdx] = min(clearance, [], 'omitnan');
    if isempty(minClear) || isnan(minClear)
        minClear = nan;
        minIdx = nan;
        minPoint = [nan, nan];
        minU = nan;
    else
        minPoint = points(minIdx,:);
        minU = u(minIdx);
    end

    state = struct();
    state.P = P;
    state.knot = knot;
    state.u = u;
    state.v = nan(nU,1);
    state.clearance = clearance;
    state.validLine = valid;
    state.validChord = false(nU,1);
    state.closestPoint = points;
    state.closestNormal = normal;
    state.closestAlpha = zeros(nU,1);
    state.nearestObsId = nearestObsId;
    state.nearestObsType = nearestObsType;
    state.minClear = minClear;
    state.minIdx = minIdx;
    state.minPoint = minPoint;
    state.minU = minU;
    state.pathSample = points;
    state.pathW = u;
    state.paramsUsed = params;
    state.objectiveGeometry = 'centerline-point';
    state.timing = struct('total', toc(tAll), 'evaluate', toc(tAll), ...
        'obstacleGrad', 0, 'regularization', 0, 'other', 0);
end
