function env = fixedChordEnvelope_old(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE Compute fixed-length chord envelope on a 2D curve.
%
% M = r(u), N = r(v(u)), ||N-M|| = L, v > u.
% The finite-segment envelope point is:
%   G(u) = r(u) + lambda(u) * (r(v)-r(u))
% where:
%   lambda = cross(r'(u), d) / cross(d, d')
%
% opts.uRange controls which u samples are evaluated.
% opts.vSearchRange controls where v is searched. Keep this [0,1] for local
% u evaluation, otherwise fixed-length chords may be incorrectly truncated.

    opts = setDefaultOpts(opts);

    uList = linspace(opts.uRange(1), opts.uRange(2), opts.nU).';
    nU = numel(uList);

    MList = nan(nU, 2);
    NList = nan(nU, 2);
    GList = nan(nU, 2);
    vList = nan(nU, 1);
    vpList = nan(nU, 1);
    lambdaList = nan(nU, 1);
    residualList = nan(nU, 1);
    validLine = false(nU, 1);
    validSegment = false(nU, 1);

    for k = 1:nU
        u = uList(k);
        M = rowVec(rFun(u));
        ru = rowVec(drFun(u));

        v = solveForwardV(rFun, u, L, opts);
        if isnan(v)
            continue;
        end

        N = rowVec(rFun(v));
        rv = rowVec(drFun(v));
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

        MList(k,:) = M;
        NList(k,:) = N;
        GList(k,:) = G;
        vList(k) = v;
        vpList(k) = vp;
        lambdaList(k) = lambda;
        residualList(k) = residual;
        validLine(k) = true;
        validSegment(k) = lambda >= -opts.lambdaTol && lambda <= 1 + opts.lambdaTol;
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

function opts = setDefaultOpts(opts)
    if nargin < 1 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'uRange'); opts.uRange = [0, 1]; end
    if ~isfield(opts, 'vSearchRange'); opts.vSearchRange = [0, 1]; end
    if ~isfield(opts, 'nU'); opts.nU = 140; end
    if ~isfield(opts, 'nVGrid'); opts.nVGrid = 180; end
    if ~isfield(opts, 'epsV'); opts.epsV = 1e-8; end
    if ~isfield(opts, 'tolDen'); opts.tolDen = 1e-10; end
    if ~isfield(opts, 'lambdaTol'); opts.lambdaTol = 1e-9; end
end

function vRoot = solveForwardV(rFun, u, L, opts)
    vMin = max(u + opts.epsV, opts.vSearchRange(1));
    vMax = opts.vSearchRange(2);

    if vMin >= vMax
        vRoot = NaN;
        return;
    end

    M = rowVec(rFun(u));
    vGrid = linspace(vMin, vMax, opts.nVGrid);
    g = nan(size(vGrid));

    for i = 1:numel(vGrid)
        R = rowVec(rFun(vGrid(i)));
        g(i) = norm(R - M) - L;
    end

    idx = find(g(1:end-1) <= 0 & g(2:end) >= 0, 1, 'first');

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
    fun = @(v) norm(rowVec(rFun(v)) - M) - L;

    try
        vRoot = fzero(fun, [v1, v2]);
    catch
        vRoot = NaN;
    end

    if isnan(vRoot) || vRoot <= u || vRoot > vMax
        vRoot = NaN;
    end
end

function x = rowVec(x)
    x = x(:).';
end

function z = cross2(a, b)
    z = a(1)*b(2) - a(2)*b(1);
end
