function validation = validateRRTSCCenterline2D(P, obstacles, opts)
%VALIDATERRTSCCENTERLINE2D Densely validate a smoothed B-spline centerline.

    tAll = tic;
    validation = emptyValidation();

    if isempty(P) || size(P,2) ~= 2 || size(P,1) < opts.degree + 1
        validation.numericalFailure = true;
        validation.message = 'Invalid B-spline control points.';
        validation.timeSec = toc(tAll);
        return;
    end

    if isfield(opts, 'knot') && ~isempty(opts.knot)
        knot = opts.knot;
    else
        knot = makeClampedUniformKnot(size(P,1), opts.degree);
    end

    controlLength = sum(vecnorm(diff(P, 1, 1), 2, 2));
    nSample = max(opts.centerline.minSamples, ...
        ceil(controlLength / opts.centerline.sampleResolution) + 1);
    u = linspace(0, 1, nSample).';
    path = evalBSplinePath2D(P, u, opts.degree, knot);

    if any(~isfinite(path(:)))
        validation.numericalFailure = true;
        validation.message = 'Non-finite B-spline centerline samples.';
        validation.timeSec = toc(tAll);
        return;
    end

    minClear = inf;
    minPoint = [nan, nan];
    minIndex = nan;
    insideBounds = true;
    for i = 1:size(path,1)
        q = path(i,:);
        if opts.centerline.checkBounds && ~pointInBounds(q, opts.bounds)
            insideBounds = false;
            minClear = -inf;
            minPoint = q;
            minIndex = i;
            break;
        end

        d = queryObstaclePointSDF2D(q, obstacles);
        if d < minClear
            minClear = d;
            minPoint = q;
            minIndex = i;
        end
    end

    validation.safe = insideBounds && isfiniteOrPositiveInf(minClear) && ...
        minClear >= opts.safetyMargin;
    validation.minClear = minClear;
    validation.minPoint = minPoint;
    validation.minIndex = minIndex;
    validation.numSamples = nSample;
    validation.pathSample = path;
    validation.insideBounds = insideBounds;
    validation.message = ternary(validation.safe, 'safe', 'unsafe');
    validation.timeSec = toc(tAll);
end

function validation = emptyValidation()
    validation = struct();
    validation.safe = false;
    validation.numericalFailure = false;
    validation.minClear = nan;
    validation.minPoint = [nan, nan];
    validation.minIndex = nan;
    validation.numSamples = 0;
    validation.pathSample = zeros(0, 2);
    validation.insideBounds = false;
    validation.message = '';
    validation.timeSec = 0;
end

function tf = pointInBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
        q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end

function tf = isfiniteOrPositiveInf(x)
    tf = isfinite(x) || isinf(x) && x > 0;
end

function value = ternary(tf, a, b)
    if tf
        value = a;
    else
        value = b;
    end
end
