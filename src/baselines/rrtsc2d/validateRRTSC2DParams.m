function opts = validateRRTSC2DParams(opts, startPt, goalPt)
%VALIDATERRTSC2DPARAMS Validate normalized RRTSC-2D options.

    opts = defaultRRTSC2DParams(opts);
    startPt = startPt(:).';
    goalPt = goalPt(:).';

    requirePositiveScalar(opts.L, 'opts.L');
    requireNonnegativeScalar(opts.dMin, 'opts.dMin');
    requireNonnegativeScalar(opts.safetyMargin, 'opts.safetyMargin');
    requirePositiveInteger(opts.degree, 'opts.degree');
    requirePositiveInteger(opts.maxAttempts, 'opts.maxAttempts');
    requirePositiveScalar(opts.maxTotalTime, 'opts.maxTotalTime');
    requirePositiveScalar(opts.controlPointSpacingFactor, ...
        'opts.controlPointSpacingFactor');
    requirePositiveInteger(opts.attemptSeedStride, 'opts.attemptSeedStride');

    if ~isequal(size(opts.bounds), [2, 2]) || ...
            any(~isfinite(opts.bounds(:))) || ...
            any(opts.bounds(:,2) <= opts.bounds(:,1))
        error('opts.bounds must be [xmin xmax; ymin ymax].');
    end
    if numel(startPt) ~= 2 || numel(goalPt) ~= 2 || ...
            any(~isfinite([startPt, goalPt]))
        error('startPt and goalPt must be finite 1-by-2 points.');
    end
    if ~pointInBounds(startPt, opts.bounds) || ...
            ~pointInBounds(goalPt, opts.bounds)
        error('startPt and goalPt must lie inside opts.bounds.');
    end

    requirePositiveScalar(opts.centerline.sampleResolution, ...
        'opts.centerline.sampleResolution');
    requirePositiveInteger(opts.centerline.minSamples, ...
        'opts.centerline.minSamples');
    requirePositiveInteger(opts.chord.nU, 'opts.chord.nU');
    requireNonnegativeInteger(opts.chord.maxRefinement, ...
        'opts.chord.maxRefinement');
    if opts.chord.refinementFactor <= 1 || ...
            ~isfinite(opts.chord.refinementFactor)
        error('opts.chord.refinementFactor must be greater than one.');
    end
    requirePositiveInteger(opts.chord.minValidChords, ...
        'opts.chord.minValidChords');

    opts.bounds = double(opts.bounds);
    opts.rrt.bounds = opts.bounds;
    opts.seed = normalizeSeed(opts.seed);
end

function tf = pointInBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
        q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end

function seed = normalizeSeed(seed)
    if ~isscalar(seed) || ~isfinite(seed)
        error('opts.seed must be a finite scalar.');
    end
    seed = mod(round(double(seed)) - 1, 2147483646) + 1;
end

function requirePositiveScalar(x, name)
    if ~isscalar(x) || ~isfinite(x) || x <= 0
        error('%s must be a positive finite scalar.', name);
    end
end

function requireNonnegativeScalar(x, name)
    if ~isscalar(x) || ~isfinite(x) || x < 0
        error('%s must be a nonnegative finite scalar.', name);
    end
end

function requirePositiveInteger(x, name)
    if ~isscalar(x) || ~isfinite(x) || x < 1 || x ~= round(x)
        error('%s must be a positive integer.', name);
    end
end

function requireNonnegativeInteger(x, name)
    if ~isscalar(x) || ~isfinite(x) || x < 0 || x ~= round(x)
        error('%s must be a nonnegative integer.', name);
    end
end
