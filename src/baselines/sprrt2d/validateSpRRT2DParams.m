function opts = validateSpRRT2DParams(opts, startPt, goalPt)
%VALIDATESPRRT2DPARAMS Validate Adaptive Sp-RRT-2D planner options.

    opts = defaultSpRRT2DParams(opts);

    requirePoint(startPt, 'startPt');
    requirePoint(goalPt, 'goalPt');
    requirePositiveScalar(opts.L, 'opts.L');
    requirePositiveInteger(opts.nominalLinkCount, ...
        'opts.nominalLinkCount');
    requirePositiveInteger(opts.maxSegmentCount, ...
        'opts.maxSegmentCount');
    if opts.nominalLinkCount ~= 6
        error(['Adaptive Sp-RRT-2D formal benchmark requires ' ...
            'opts.nominalLinkCount = 6.']);
    end
    if opts.maxSegmentCount ~= 12
        error(['Adaptive Sp-RRT-2D formal benchmark requires ' ...
            'opts.maxSegmentCount = 12.']);
    end
    if opts.maxSegmentCount < opts.nominalLinkCount
        error('maxSegmentCount cannot be smaller than nominalLinkCount.');
    end
    requirePositiveScalar(opts.thetaMax, 'opts.thetaMax');
    requirePositiveInteger(opts.maxIter, 'opts.maxIter');
    requireNonnegativeScalar(opts.collisionResolution, ...
        'opts.collisionResolution', false);
    requireNonnegativeScalar(opts.inflateRadius, 'opts.inflateRadius', true);
    requireNonnegativeScalar(opts.lengthTolerance, ...
        'opts.lengthTolerance', true);
    requireNonnegativeScalar(opts.angleTolerance, ...
        'opts.angleTolerance', true);
    requireNonnegativeScalar(opts.reachabilityTolerance, ...
        'opts.reachabilityTolerance', true);
    requireNonnegativeScalar(opts.duplicateTolerance, ...
        'opts.duplicateTolerance', true);

    if ~isequal(size(opts.bounds), [2, 2]) || ...
            any(~isfinite(opts.bounds(:))) || ...
            any(opts.bounds(:,2) <= opts.bounds(:,1))
        error('opts.bounds must be [xmin xmax; ymin ymax].');
    end
    if ~isscalar(opts.entranceBias) || ~isfinite(opts.entranceBias) || ...
            opts.entranceBias < 0 || opts.entranceBias > 1
        error('opts.entranceBias must lie in [0,1].');
    end
    if ~isscalar(opts.maxTimeSec) || isnan(opts.maxTimeSec) || ...
            opts.maxTimeSec <= 0
        error('opts.maxTimeSec must be positive or Inf.');
    end
    if ~isscalar(opts.seed) || ~isfinite(opts.seed)
        error('opts.seed must be a finite scalar.');
    end

    if opts.thetaMax > 40*pi/180 + opts.angleTolerance
        error(['Adaptive Sp-RRT-2D thetaMax may be stricter for tests, ' ...
            'but cannot exceed the formal 40-degree limit.']);
    end
    opts.nLinks = opts.nominalLinkCount;
    opts.linkLengths = opts.L * ones(1, opts.nominalLinkCount);
    opts.thetaLimits = opts.thetaMax * ...
        ones(1, opts.maxSegmentCount);
    opts.pathOptimization.thetaMax = opts.thetaMax;
    if opts.terminalDirectionEnabled
        error(['Terminal-direction constraints are not supported in the ' ...
            'formal Adaptive Sp-RRT-2D benchmark.']);
    end

    mode = lower(string(opts.pathOptimization.mode));
    if mode ~= "non-worsening-angle" && mode ~= "theta-limit"
        error(['opts.pathOptimization.mode must be ' ...
            '''non-worsening-angle'' or ''theta-limit''.']);
    end
end

function requirePoint(q, name)
    if ~isnumeric(q) || numel(q) ~= 2 || any(~isfinite(q))
        error('%s must be a finite 2-D point.', name);
    end
end

function requirePositiveScalar(x, name)
    if ~isscalar(x) || ~isfinite(x) || x <= 0
        error('%s must be a positive finite scalar.', name);
    end
end

function requirePositiveInteger(x, name)
    requirePositiveScalar(x, name);
    if x ~= round(x)
        error('%s must be an integer.', name);
    end
end

function requireNonnegativeScalar(x, name, allowZero)
    if ~isscalar(x) || ~isfinite(x) || x < 0 || (~allowZero && x == 0)
        error('%s has an invalid value.', name);
    end
end
