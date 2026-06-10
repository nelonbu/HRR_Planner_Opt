function env = fixedChordEnvelope(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE Fast fixed-length chord envelope on a 2D curve.
%
% M = r(u), N = r(v(u)), ||N-M|| = L, v > u.
%
% This version uses continuation / warm-start for v(u):
%   v_k^0 = v_{k-1} + v'_{k-1} * (u_k - u_{k-1})
%
% Then Newton iteration solves:
%   norm(r(v)-r(u)) - L = 0
%
% opts.nU keeps the original u density.
% opts.nVGrid is only used as a fallback, not for every u.

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

    newtonSuccess = false(nU, 1);
    fallbackUsed = false(nU, 1);
    newtonIters = zeros(nU, 1);

    hasPrev = false;
    prevU = NaN;
    prevV = NaN;
    prevVp = NaN;

    for k = 1:nU
        u = uList(k);
        M = rowVec(rFun(u));
        ru = rowVec(drFun(u));

        % ------------------------------------------------------------
        % Initial guess for v
        % ------------------------------------------------------------
        if hasPrev && isfinite(prevV)
            du = u - prevU;

            if isfinite(prevVp)
                vInit = prevV + prevVp * du;
            else
                vInit = prevV + du;
            end

            % If predicted value becomes invalid, fall back to speed estimate.
            if ~isfinite(vInit) || vInit <= u + opts.epsV || vInit > opts.vSearchRange(2)
                vInit = initialGuessBySpeed(u, ru, L, opts);
            end
        else
            vInit = initialGuessBySpeed(u, ru, L, opts);
        end

        [v, solveInfo] = solveForwardVWarm(rFun, drFun, M, u, L, opts, vInit);

        newtonSuccess(k) = solveInfo.newtonSuccess;
        fallbackUsed(k) = solveInfo.fallbackUsed;
        newtonIters(k) = solveInfo.newtonIters;

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

        % Use this valid v/vp for continuation even if envelope line is invalid.
        hasPrev = true;
        prevU = u;
        prevV = v;
        prevVp = vp;

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

    env.stats = struct();
    env.stats.newtonSuccess = newtonSuccess;
    env.stats.fallbackUsed = fallbackUsed;
    env.stats.newtonIters = newtonIters;
    env.stats.numNewtonSuccess = sum(newtonSuccess);
    env.stats.numFallbackUsed = sum(fallbackUsed);
    env.stats.avgNewtonIters = mean(newtonIters(newtonIters > 0));
end

function opts = setDefaultOpts(opts)
    if nargin < 1 || isempty(opts)
        opts = struct();
    end

    if ~isfield(opts, 'uRange'); opts.uRange = [0, 1]; end
    if ~isfield(opts, 'vSearchRange'); opts.vSearchRange = [0, 1]; end
    if ~isfield(opts, 'nU'); opts.nU = 140; end

    % nVGrid is no longer used every time. It is only fallback-related.
    if ~isfield(opts, 'nVGrid'); opts.nVGrid = 180; end
    if ~isfield(opts, 'fallbackNGrid'); opts.fallbackNGrid = min(opts.nVGrid, 40); end

    if ~isfield(opts, 'epsV'); opts.epsV = 1e-8; end
    if ~isfield(opts, 'tolDen'); opts.tolDen = 1e-10; end
    if ~isfield(opts, 'lambdaTol'); opts.lambdaTol = 1e-9; end

    % Newton settings. Accuracy does not need to be too high.
    if ~isfield(opts, 'maxNewtonIter'); opts.maxNewtonIter = 6; end
    if ~isfield(opts, 'vResidualTol'); opts.vResidualTol = 1e-5; end
    if ~isfield(opts, 'vAcceptTol'); opts.vAcceptTol = 5e-4; end
    if ~isfield(opts, 'vStepTol'); opts.vStepTol = 1e-8; end
    if ~isfield(opts, 'maxNewtonStep'); opts.maxNewtonStep = 0.15; end
    if ~isfield(opts, 'minSpeed'); opts.minSpeed = 1e-8; end

    if ~isfield(opts, 'enableFallback'); opts.enableFallback = true; end
end

function vInit = initialGuessBySpeed(u, ru, L, opts)
    speed = norm(ru);

    if speed < opts.minSpeed
        dv = 0.05;
    else
        % First-order estimate:
        % ||r(u+dv)-r(u)|| ≈ ||r'(u)|| dv = L
        dv = L / speed;
    end

    vInit = u + dv;
    vInit = clampV(vInit, u, opts);
end

function [vRoot, info] = solveForwardVWarm(rFun, drFun, M, u, L, opts, vInit)
    info = struct();
    info.newtonSuccess = false;
    info.fallbackUsed = false;
    info.newtonIters = 0;

    vMin = max(u + opts.epsV, opts.vSearchRange(1));
    vMax = opts.vSearchRange(2);

    if vMin >= vMax
        vRoot = NaN;
        return;
    end

    v = clamp(vInit, vMin, vMax);

    bestV = v;
    bestAbsF = inf;

    % ------------------------------------------------------------
    % Newton iteration
    % ------------------------------------------------------------
    for it = 1:opts.maxNewtonIter
        R = rowVec(rFun(v));
        rv = rowVec(drFun(v));

        d = R - M;
        dist = norm(d);
        F = dist - L;

        info.newtonIters = it;

        if abs(F) < bestAbsF
            bestAbsF = abs(F);
            bestV = v;
        end

        if abs(F) <= opts.vResidualTol
            vRoot = v;
            info.newtonSuccess = true;
            return;
        end

        if dist < opts.tolDen
            break;
        end

        dF = dot(d, rv) / dist;

        if abs(dF) < opts.tolDen || ~isfinite(dF)
            break;
        end

        step = F / dF;

        % Limit Newton step for robustness.
        if abs(step) > opts.maxNewtonStep
            step = sign(step) * opts.maxNewtonStep;
        end

        vNew = v - step;

        % Damped projection if Newton jumps outside search range.
        if vNew <= vMin
            vNew = 0.5 * (v + vMin);
        elseif vNew >= vMax
            vNew = 0.5 * (v + vMax);
        end

        if abs(vNew - v) <= opts.vStepTol
            v = vNew;
            break;
        end

        v = vNew;
    end

    % Accept approximate solution if residual is small enough.
    if bestAbsF <= opts.vAcceptTol
        vRoot = bestV;
        info.newtonSuccess = true;
        return;
    end

    % ------------------------------------------------------------
    % Fallback: low-density coarse search + linear interpolation
    % ------------------------------------------------------------
    if opts.enableFallback
        info.fallbackUsed = true;
        vRoot = solveForwardVCoarse(rFun, M, u, L, opts);
    else
        vRoot = NaN;
    end
end

function vRoot = solveForwardVCoarse(rFun, M, u, L, opts)
    vMin = max(u + opts.epsV, opts.vSearchRange(1));
    vMax = opts.vSearchRange(2);

    if vMin >= vMax
        vRoot = NaN;
        return;
    end

    nGrid = opts.fallbackNGrid;
    vGrid = linspace(vMin, vMax, nGrid);
    g = nan(size(vGrid));

    for i = 1:numel(vGrid)
        R = rowVec(rFun(vGrid(i)));
        g(i) = norm(R - M) - L;
    end

    % Find first forward crossing.
    idx = find(g(1:end-1) <= 0 & g(2:end) >= 0, 1, 'first');

    if isempty(idx)
        [minAbsG, iMin] = min(abs(g));

        if minAbsG <= opts.vAcceptTol
            vRoot = vGrid(iMin);
        else
            vRoot = NaN;
        end

        return;
    end

    v1 = vGrid(idx);
    v2 = vGrid(idx + 1);
    g1 = g(idx);
    g2 = g(idx + 1);

    % Linear interpolation instead of fzero for speed.
    if abs(g2 - g1) < opts.tolDen
        vRoot = 0.5 * (v1 + v2);
    else
        vRoot = v1 - g1 * (v2 - v1) / (g2 - g1);
    end

    if isnan(vRoot) || vRoot <= u || vRoot > vMax
        vRoot = NaN;
    end
end

function v = clampV(v, u, opts)
    vMin = max(u + opts.epsV, opts.vSearchRange(1));
    vMax = opts.vSearchRange(2);
    v = clamp(v, vMin, vMax);
end

function y = clamp(x, a, b)
    y = min(max(x, a), b);
end

function x = rowVec(x)
    x = x(:).';
end

function z = cross2(a, b)
    z = a(1)*b(2) - a(2)*b(1);
end