function opts = defaultRRTSC2DParams(opts)
%DEFAULTRRTSC2DPARAMS Fill defaults for the 2-D RRTSC baseline.
%
% RRT defaults mirror planRRT2D. The common FTL defaults (L, dMin, degree)
% mirror the stable project defaults. Callers should override them with the
% same values used by the proposed method in formal comparisons.

    if nargin < 1 || isempty(opts)
        opts = struct();
    end

    opts = setDefault(opts, 'L', 0.15);
    opts = setDefault(opts, 'dMin', 0.02);
    opts = setDefault(opts, 'safetyMargin', opts.dMin);
    opts = setDefault(opts, 'degree', 3);
    opts = setDefault(opts, 'bounds', [0, 1; -0.4, 0.4]);
    opts = setDefault(opts, 'seed', 1);
    opts = setDefault(opts, 'maxAttempts', 50);
    opts = setDefault(opts, 'maxTotalTime', 30.0);
    opts = setDefault(opts, 'controlPointSpacingFactor', 0.5);
    opts = setDefault(opts, 'attemptSeedStride', 104729);
    opts = setDefault(opts, 'verbose', false);

    if ~isfield(opts, 'rrt') || isempty(opts.rrt)
        opts.rrt = struct();
    end
    opts.rrt = setDefault(opts.rrt, 'bounds', opts.bounds);
    opts.rrt = setDefault(opts.rrt, 'stepSize', 0.035);
    opts.rrt = setDefault(opts.rrt, 'goalBias', 0.12);
    opts.rrt = setDefault(opts.rrt, 'goalTol', 0.04);
    opts.rrt = setDefault(opts.rrt, 'maxIter', 3000);
    opts.rrt = setDefault(opts.rrt, 'collisionResolution', 0.004);
    opts.rrt = setDefault(opts.rrt, 'inflateRadius', 0.0);

    if ~isfield(opts, 'centerline') || isempty(opts.centerline)
        opts.centerline = struct();
    end
    opts.centerline = setDefault(opts.centerline, 'sampleResolution', 0.001);
    opts.centerline = setDefault(opts.centerline, 'minSamples', 800);
    opts.centerline = setDefault(opts.centerline, 'checkBounds', true);

    if ~isfield(opts, 'chord') || isempty(opts.chord)
        opts.chord = struct();
    end
    opts.chord = setDefault(opts.chord, 'nU', 240);
    opts.chord = setDefault(opts.chord, 'maxRefinement', 2);
    opts.chord = setDefault(opts.chord, 'refinementFactor', 2);
    opts.chord = setDefault(opts.chord, 'minValidChords', 1);
    opts.chord = setDefault(opts.chord, 'useObstacleFilter', true);

    if ~isfield(opts.chord, 'envOpts') || isempty(opts.chord.envOpts)
        opts.chord.envOpts = struct();
    end
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'uRange', [0, 1]);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'vSearchRange', [0, 1]);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'epsV', 1e-8);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'tolDen', 1e-10);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'maxNewtonIter', 8);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'vAcceptTol', 1e-5);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'newtonTriggerTol', 1e-5);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'vStepTol', 1e-11);
    opts.chord.envOpts = setDefault(opts.chord.envOpts, 'maxNewtonStep', 0.20);
end

function s = setDefault(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end
