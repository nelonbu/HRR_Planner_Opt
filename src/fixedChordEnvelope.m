function env = fixedChordEnvelope(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE Compute envelope of fixed-length chords on a 2D parametric curve.
%
% Problem:
%   M = r(u)
%   N = r(v(u))
%   ||N - M|| = L, v > u
%
% Segment:
%   P(u, lambda) = r(u) + lambda * (r(v(u)) - r(u))
%
% Envelope:
%   Gamma(u) = r(u) + lambda(u) * d(u)
%
% where:
%   d(u) = r(v(u)) - r(u)
%   lambda(u) = cross(r'(u), d(u)) / cross(d(u), d'(u))
%   d'(u) = r'(v) * v'(u) - r'(u)
%   v'(u) = dot(d, r'(u)) / dot(d, r'(v))
%
% Inputs:
%   rFun  : function handle, rFun(w) -> [x, y]
%   drFun : function handle, drFun(w) -> [dx, dy]
%           If drFun = [], numerical finite difference is used.
%   L     : fixed chord length
%   opts  : optional struct
%
% opts fields:
%   opts.uRange      : [uMin, uMax], default [0, 1]
%   opts.nU          : number of u samples, default 300
%   opts.nVGrid      : grid samples for root bracketing, default 500
%   opts.epsV        : minimum v-u gap, default 1e-8
%   opts.tolDen      : degeneracy tolerance, default 1e-10
%   opts.diffStep    : finite difference step if drFun=[], default 1e-6
%   opts.rootMode    : 'first' only for now, default 'first'
%
% Outputs:
%   env.u            : sampled u values
%   env.v            : solved v(u)
%   env.vp           : v'(u)
%   env.M            : M points
%   env.N            : N points
%   env.G            : envelope points
%   env.lambda       : lambda values
%   env.validLine    : true if envelope of extended line is valid
%   env.validSegment : true if lambda in [0,1], true finite-segment envelope
%   env.residual     : ||N-M|| - L
%   env.opts         : used options

    if nargin < 4
        opts = struct();
    end

    opts = setDefaultOpts(opts);

    if isempty(drFun)
        drFun = @(w) numericalDerivative(rFun, w, opts.diffStep, opts.uRange);
    end

    uMin = opts.uRange(1);
    uMax = opts.uRange(2);

    uList = linspace(uMin, uMax, opts.nU).';

    MList = nan(opts.nU, 2);
    NList = nan(opts.nU, 2);
    GList = nan(opts.nU, 2);
    vList = nan(opts.nU, 1);
    vpList = nan(opts.nU, 1);
    lambdaList = nan(opts.nU, 1);
    residualList = nan(opts.nU, 1);

    validLine = false(opts.nU, 1);
    validSegment = false(opts.nU, 1);

    for k = 1:opts.nU
        u = uList(k);

        M = evalRow(rFun, u);
        ru = evalRow(drFun, u);

        v = solveForwardV(rFun, u, L, opts);

        if isnan(v)
            continue;
        end

        N = evalRow(rFun, v);
        rv = evalRow(drFun, v);

        d = N - M;
        residual = norm(d) - L;

        denomVp = dot(d, rv);
        if abs(denomVp) < opts.tolDen
            continue;
        end

        vp = dot(d, ru) / denomVp;

        dp = rv * vp - ru;

        den = cross2(d, dp);
        if abs(den) < opts.tolDen
            continue;
        end

        lambda = cross2(ru, d) / den;

        G = M + lambda * d;

        MList(k, :) = M;
        NList(k, :) = N;
        GList(k, :) = G;
        vList(k) = v;
        vpList(k) = vp;
        lambdaList(k) = lambda;
        residualList(k) = residual;

        validLine(k) = true;

        if lambda >= -1e-9 && lambda <= 1 + 1e-9
            validSegment(k) = true;
        end
    end

    env = struct();
    env.u = uList;
    env.v = vList;
    env.vp = vpList;
    env.M = MList;
    env.N = NList;
    env.G = GList;
    env.lambda = lambdaList;
    env.validLine = validLine;
    env.validSegment = validSegment;
    env.residual = residualList;
    env.opts = opts;
end

%% ============================================================
% Helper functions
%% ============================================================

function opts = setDefaultOpts(opts)
    if ~isfield(opts, 'uRange')
        opts.uRange = [0, 1];
    end
    if ~isfield(opts, 'nU')
        opts.nU = 300;
    end
    if ~isfield(opts, 'nVGrid')
        opts.nVGrid = 500;
    end
    if ~isfield(opts, 'epsV')
        opts.epsV = 1e-8;
    end
    if ~isfield(opts, 'tolDen')
        opts.tolDen = 1e-10;
    end
    if ~isfield(opts, 'diffStep')
        opts.diffStep = 1e-6;
    end
    if ~isfield(opts, 'rootMode')
        opts.rootMode = 'first';
    end
end

function vRoot = solveForwardV(rFun, u, L, opts)
    uMax = opts.uRange(2);

    if u >= uMax - opts.epsV
        vRoot = NaN;
        return;
    end

    M = evalRow(rFun, u);

    vGrid = linspace(u + opts.epsV, uMax, opts.nVGrid);
    g = nan(size(vGrid));

    for i = 1:numel(vGrid)
        R = evalRow(rFun, vGrid(i));
        g(i) = norm(R - M) - L;
    end

    % Find the first sign change.
    idx = find(g(1:end-1) <= 0 & g(2:end) >= 0, 1, 'first');

    % Handle the rare case where a grid sample is already very close to zero.
    if isempty(idx)
        [minAbsG, iMin] = min(abs(g));
        if minAbsG < 1e-8
            vRoot = vGrid(iMin);
        else
            vRoot = NaN;
        end
        return;
    end

    v1 = vGrid(idx);
    v2 = vGrid(idx + 1);

    fun = @(v) norm(evalRow(rFun, v) - M) - L;

    try
        vRoot = fzero(fun, [v1, v2]);
    catch
        vRoot = NaN;
    end

    if isnan(vRoot) || vRoot <= u || vRoot > uMax
        vRoot = NaN;
    end
end

function x = evalRow(fun, w)
    x = fun(w);
    x = x(:).';
    if numel(x) ~= 2
        error('Curve function must return a 2D point [x, y].');
    end
end

function dx = numericalDerivative(rFun, w, h, uRange)
    w0 = max(uRange(1), w - h);
    w1 = min(uRange(2), w + h);

    if abs(w1 - w0) < eps
        dx = [0, 0];
        return;
    end

    x0 = evalRow(rFun, w0);
    x1 = evalRow(rFun, w1);
    dx = (x1 - x0) / (w1 - w0);
end

function z = cross2(a, b)
    z = a(1) * b(2) - a(2) * b(1);
end