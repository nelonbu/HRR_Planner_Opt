function env = fixedChordEnvelope(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE Local-Newton version for fixed-length chord envelopes.
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
% This version only uses local Newton continuation to solve v(u).
% It does NOT use global grid search or fzero.
%
% Inputs:
%   rFun  : function handle, rFun(w) -> [x, y]
%   drFun : function handle, drFun(w) -> [dx, dy]
%           If drFun = [], numerical finite difference is used.
%   L     : fixed chord length
%   opts  : optional struct
%
% Main opts:
%   opts.uRange          : [uMin, uMax], default [0, 1]
%   opts.nU              : number of u samples, default 300
%   opts.epsV            : minimum v-u gap, default 1e-8
%   opts.tolDen          : denominator tolerance, default 1e-10
%   opts.diffStep        : finite difference step if drFun=[], default 1e-6
%   opts.rootTol         : length residual tolerance, default 1e-6
%   opts.newtonMaxIter   : Newton max iterations, default 10
%   opts.newtonMaxStep   : max Newton step in parameter; if NaN, use 5*du
%   opts.newtonDamping   : true/false, default true
%   opts.enableTiming    : true/false, default false
%
% Outputs:
%   env.u, env.v, env.vp, env.M, env.N, env.G, env.lambda
%   env.validLine, env.validSegment, env.residual
%   env.timing

    if nargin < 4
        opts = struct();
    end

    opts = setDefaultOpts(opts);
    timing = emptyTiming();
    tTotal = tic;

    if isempty(drFun)
        drFun = @(w) numericalDerivative(rFun, w, opts.diffStep, opts.uRange);
    end

    uMin = opts.uRange(1);
    uMax = opts.uRange(2);
    uList = linspace(uMin, uMax, opts.nU).';

    if opts.nU > 1
        duDefault = mean(diff(uList));
    else
        duDefault = 1e-3;
    end

    if isnan(opts.newtonMaxStep)
        opts.newtonMaxStep = 5 * duDefault;
    end

    MList = nan(opts.nU, 2);
    NList = nan(opts.nU, 2);
    GList = nan(opts.nU, 2);
    vList = nan(opts.nU, 1);
    vpList = nan(opts.nU, 1);
    lambdaList = nan(opts.nU, 1);
    residualList = nan(opts.nU, 1);

    validLine = false(opts.nU, 1);
    validSegment = false(opts.nU, 1);

    prev = struct();
    prev.has = false;
    prev.u = NaN;
    prev.v = NaN;
    prev.vp = NaN;

    for k = 1:opts.nU
        u = uList(k);

        %% ---------- evaluate M and r'(u) ----------
        tEvalM = tic;
        M = evalRow(rFun, u);
        ru = evalRow(drFun, u);
        timing.evalM = timing.evalM + toc(tEvalM);

        %% ---------- solve v(u) by local Newton ----------
        tSolveV = tic;
        [v, solveStats] = solveForwardVNewtonLocal(rFun, drFun, M, ru, u, L, opts, prev, duDefault);
        timing.solveV = timing.solveV + toc(tSolveV);

        timing.newton = timing.newton + solveStats.newtonTime;
        timing.numNewtonCall = timing.numNewtonCall + solveStats.numNewtonCall;
        timing.numNewtonIter = timing.numNewtonIter + solveStats.numNewtonIter;
        timing.numNewtonSuccess = timing.numNewtonSuccess + solveStats.numNewtonSuccess;
        timing.numNewtonFail = timing.numNewtonFail + solveStats.numNewtonFail;
        timing.numDirectAccept = timing.numDirectAccept + solveStats.numDirectAccept;

        if isnan(v)
            continue;
        end

        %% ---------- evaluate N and r'(v) ----------
        tEvalN = tic;
        N = evalRow(rFun, v);
        rv = evalRow(drFun, v);
        timing.evalN = timing.evalN + toc(tEvalN);

        %% ---------- envelope formula ----------
        tFormula = tic;

        d = N - M;
        residual = norm(d) - L;

        denomVp = dot(d, rv);
        if abs(denomVp) < opts.tolDen
            timing.formula = timing.formula + toc(tFormula);
            continue;
        end

        vp = dot(d, ru) / denomVp;

        % Update continuation state after v and vp are valid.
        prev.has = true;
        prev.u = u;
        prev.v = v;
        prev.vp = vp;

        dp = rv * vp - ru;
        den = cross2(d, dp);
        if abs(den) < opts.tolDen
            timing.formula = timing.formula + toc(tFormula);
            continue;
        end

        lambda = cross2(ru, d) / den;
        G = M + lambda * d;

        timing.formula = timing.formula + toc(tFormula);

        %% ---------- save ----------
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

    timing.total = toc(tTotal);
    timing.other = timing.total - timing.evalM - timing.solveV - timing.evalN - timing.formula;
    timing.numU = opts.nU;
    timing.numValidLine = sum(validLine);
    timing.numValidSegment = sum(validSegment);

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
    env.timing = timing;

    if opts.enableTiming
        printEnvelopeTiming(timing, opts);
    end
end

%% ============================================================
% Local Newton solver for v(u)
%% ============================================================

function [vRoot, stats] = solveForwardVNewtonLocal(rFun, drFun, M, ru, u, L, opts, prev, duDefault)
    stats = emptySolveStats();
    stats.numNewtonCall = 1;
    vRoot = NaN;

    uMax = opts.uRange(2);
    if u >= uMax - opts.epsV
        stats.numNewtonFail = 1;
        return;
    end

    % Initial guess: continuation if possible; otherwise local tangent estimate.
    if prev.has && isfinite(prev.v) && prev.v > u
        du = u - prev.u;
        if isfinite(prev.vp)
            v = prev.v + prev.vp * du;
        else
            v = prev.v + du;
        end
    else
        speed = norm(ru);
        if speed < opts.tolDen
            stats.numNewtonFail = 1;
            return;
        end
        v = u + L / speed;
    end

    v = min(max(v, u + opts.epsV), uMax);

    tNewton = tic;

    % Direct accept if initial guess is already good enough.
    [g, gp, ok] = lengthResidualAndDerivative(rFun, drFun, M, v, L, opts.tolDen);
    if ok && abs(g) <= opts.rootTol
        vRoot = v;
        stats.numDirectAccept = 1;
        stats.numNewtonSuccess = 1;
        stats.newtonTime = toc(tNewton);
        return;
    end

    success = false;

    for it = 1:opts.newtonMaxIter
        stats.numNewtonIter = stats.numNewtonIter + 1;

        [g, gp, ok] = lengthResidualAndDerivative(rFun, drFun, M, v, L, opts.tolDen);
        if ~ok || ~isfinite(gp)
            break;
        end

        if abs(g) <= opts.rootTol
            success = true;
            break;
        end

        step = g / gp;

        % Limit step to keep Newton local and bias toward nearest forward root.
        if opts.newtonMaxStep > 0
            step = max(-opts.newtonMaxStep, min(opts.newtonMaxStep, step));
        end

        vNew = v - step;

        if vNew <= u + opts.epsV || vNew > uMax || ~isfinite(vNew)
            break;
        end

        if opts.newtonDamping
            gAbs = abs(g);
            dampCount = 0;
            while dampCount < opts.maxDampingIter
                [gNew, ~, okNew] = lengthResidualAndDerivative(rFun, drFun, M, vNew, L, opts.tolDen);
                if okNew && abs(gNew) <= opts.dampingAcceptRatio * gAbs
                    break;
                end

                step = 0.5 * step;
                vNew = v - step;

                if vNew <= u + opts.epsV || vNew > uMax || ~isfinite(vNew)
                    break;
                end

                dampCount = dampCount + 1;
            end

            if vNew <= u + opts.epsV || vNew > uMax || ~isfinite(vNew)
                break;
            end
        end

        v = vNew;
    end

    % Final accept check.
    [gFinal, ~, okFinal] = lengthResidualAndDerivative(rFun, drFun, M, v, L, opts.tolDen);
    if okFinal && abs(gFinal) <= opts.rootTol && v > u && v <= uMax
        success = true;
    end

    stats.newtonTime = toc(tNewton);

    if success
        vRoot = v;
        stats.numNewtonSuccess = 1;
    else
        vRoot = NaN;
        stats.numNewtonFail = 1;
    end
end

function [g, gp, ok] = lengthResidualAndDerivative(rFun, drFun, M, v, L, tolDen)
    Rv = evalRow(rFun, v);
    rv = evalRow(drFun, v);

    d = Rv - M;
    dist = norm(d);

    if dist < tolDen
        g = NaN;
        gp = NaN;
        ok = false;
        return;
    end

    g = dist - L;
    gp = dot(d, rv) / dist;

    ok = abs(gp) >= tolDen;
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
    if ~isfield(opts, 'epsV')
        opts.epsV = 1e-8;
    end
    if ~isfield(opts, 'tolDen')
        opts.tolDen = 1e-10;
    end
    if ~isfield(opts, 'diffStep')
        opts.diffStep = 1e-6;
    end
    if ~isfield(opts, 'enableTiming')
        opts.enableTiming = false;
    end

    % Local Newton options.
    if ~isfield(opts, 'rootTol')
        opts.rootTol = 1e-6;
    end
    if ~isfield(opts, 'newtonMaxIter')
        opts.newtonMaxIter = 10;
    end
    if ~isfield(opts, 'newtonMaxStep')
        % NaN means use 5 * mean du.
        opts.newtonMaxStep = NaN;
    end
    if ~isfield(opts, 'newtonDamping')
        opts.newtonDamping = true;
    end
    if ~isfield(opts, 'maxDampingIter')
        opts.maxDampingIter = 6;
    end
    if ~isfield(opts, 'dampingAcceptRatio')
        opts.dampingAcceptRatio = 0.9;
    end
end

function stats = emptySolveStats()
    stats = struct();
    stats.newtonTime = 0;
    stats.numNewtonCall = 0;
    stats.numNewtonIter = 0;
    stats.numNewtonSuccess = 0;
    stats.numNewtonFail = 0;
    stats.numDirectAccept = 0;
end

function timing = emptyTiming()
    timing = struct();
    timing.total = 0;
    timing.evalM = 0;
    timing.solveV = 0;
    timing.evalN = 0;
    timing.formula = 0;
    timing.newton = 0;
    timing.other = 0;

    timing.numU = 0;
    timing.numValidLine = 0;
    timing.numValidSegment = 0;
    timing.numNewtonCall = 0;
    timing.numNewtonIter = 0;
    timing.numNewtonSuccess = 0;
    timing.numNewtonFail = 0;
    timing.numDirectAccept = 0;
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

function printEnvelopeTiming(t, opts)
    safeTotal = max(t.total, 1e-12);
    safeSolveV = max(t.solveV, 1e-12);

    fprintf('\n[fixedChordEnvelope timing: local Newton only]\n');
    fprintf('  total time              : %.3f ms\n', t.total * 1000);
    fprintf('  u samples               : %d\n', t.numU);
    fprintf('  valid line envelopes    : %d\n', t.numValidLine);
    fprintf('  valid segment envelopes : %d\n', t.numValidSegment);

    fprintf('  eval M + r''(u)          : %.3f ms  (%5.1f%% total)\n', ...
        t.evalM * 1000, 100 * t.evalM / safeTotal);

    fprintf('  solve v(u)              : %.3f ms  (%5.1f%% total)\n', ...
        t.solveV * 1000, 100 * t.solveV / safeTotal);

    fprintf('    local Newton          : %.3f ms  (%5.1f%% solveV)\n', ...
        t.newton * 1000, 100 * t.newton / safeSolveV);

    fprintf('  eval N + r''(v)          : %.3f ms  (%5.1f%% total)\n', ...
        t.evalN * 1000, 100 * t.evalN / safeTotal);

    fprintf('  envelope formula        : %.3f ms  (%5.1f%% total)\n', ...
        t.formula * 1000, 100 * t.formula / safeTotal);

    fprintf('  other                   : %.3f ms  (%5.1f%% total)\n', ...
        t.other * 1000, 100 * t.other / safeTotal);

    fprintf('  Newton calls            : %d\n', t.numNewtonCall);
    fprintf('  Newton success          : %d\n', t.numNewtonSuccess);
    fprintf('  Newton failed           : %d\n', t.numNewtonFail);
    fprintf('  Direct accept           : %d\n', t.numDirectAccept);
    fprintf('  Newton total iterations : %d\n', t.numNewtonIter);

    if t.numNewtonCall > 0
        fprintf('  Newton avg iterations   : %.2f\n', ...
            t.numNewtonIter / max(1, t.numNewtonCall));
    end

    if t.numNewtonFail > 0
        fprintf('  [note] %d samples failed local Newton. This is expected near the curve end or difficult regions.\n', ...
            t.numNewtonFail);
    end

    if opts.rootTol > 1e-8
        fprintf('  [note] rootTol = %.1e, optimized for speed rather than very high precision.\n', opts.rootTol);
    end
end
