function env = fixedChordEnvelope_newtonFix(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE Fast fixed-length chord envelope with selective Newton.
%
% Strategy:
%   1. Predict v by first-order speed approximation:
%        v0 = u + L / ||r'(u)||
%   2. Evaluate residual:
%        e = ||r(v0)-r(u)|| - L
%   3. Accept accurate samples directly.
%   4. Apply Newton only to samples whose residual is too large.
%
% This balances speed and accuracy.

    opts = setDefaultOpts(opts);

    %% 1. u samples
    uList = linspace(opts.uRange(1), opts.uRange(2), opts.nU).';
    nU = numel(uList);

    %% 2. Evaluate M = r(u), ru = r'(u)
    MList  = ensureNx2(rFun(uList));
    ruList = ensureNx2(drFun(uList));

    %% 3. Initial prediction
    ruNorm = vecNorm2(ruList);
    ruNorm = max(ruNorm, opts.minSpeed);

    vMinList = max(uList + opts.epsV, opts.vSearchRange(1));
    vMax = opts.vSearchRange(2);

    vList = uList + L ./ ruNorm;
    vList = clampVec(vList, vMinList, vMax);

    %% 4. Feasibility check near the end
    Rend = ensureNx2(rFun(vMax));
    if size(Rend,1) == 1
        Rend = repmat(Rend, nU, 1);
    end

    distToEnd = vecNorm2(Rend - MList);

    feasible = distToEnd >= L - opts.vAcceptTol ...
        & vMinList < vMax ...
        & isfinite(ruNorm);

    %% 5. Evaluate predicted N and residual
    NList  = ensureNx2(rFun(vList));
    rvList = ensureNx2(drFun(vList));

    d = NList - MList;
    dist = vecNorm2(d);
    residualList = dist - L;

    rawResidualList = residualList;

    %% 6. Selective Newton
    needNewton = feasible ...
        & isfinite(residualList) ...
        & abs(residualList) > opts.newtonTriggerTol;

    newtonIters = zeros(nU,1);
    newtonUsed = false(nU,1);
    newtonConverged = feasible & abs(residualList) <= opts.vAcceptTol;

    active = needNewton;

    for it = 1:opts.maxNewtonIter
        if ~any(active)
            break;
        end

        idx = find(active);

        dNow = NList(idx,:) - MList(idx,:);
        distNow = vecNorm2(dNow);
        F = distNow - L;

        dF = sum(dNow .* rvList(idx,:), 2) ./ max(distNow, opts.tolDen);

        validDF = isfinite(dF) & abs(dF) > opts.tolDen;

        step = zeros(size(F));
        step(validDF) = F(validDF) ./ dF(validDF);
        step = clampVec(step, -opts.maxNewtonStep, opts.maxNewtonStep);

        vOld = vList(idx);
        vNew = vOld - step;

        vMinNow = vMinList(idx);

        below = vNew <= vMinNow;
        above = vNew >= vMax;

        vNew(below) = 0.5 * (vOld(below) + vMinNow(below));
        vNew(above) = 0.5 * (vOld(above) + vMax);

        bad = ~validDF | ~isfinite(vNew);
        vNew(bad) = vOld(bad);

        % Update only active samples
        vList(idx) = vNew;
        NList(idx,:) = ensureNx2(rFun(vNew));
        rvList(idx,:) = ensureNx2(drFun(vNew));

        dNew = NList(idx,:) - MList(idx,:);
        distNew = vecNorm2(dNew);
        residualNew = distNew - L;
        residualList(idx) = residualNew;

        newtonUsed(idx) = true;
        newtonIters(idx) = it;

        converged = abs(residualNew) <= opts.vAcceptTol;
        smallStep = abs(vNew - vOld) <= opts.vStepTol;

        stop = converged | smallStep | bad;
        active(idx(stop)) = false;

        newtonConverged(idx(converged)) = true;
    end

    %% 7. Final valid length check
    lengthOK = abs(residualList) <= opts.vAcceptTol;

    %% 8. Compute vp, d', lambda, G
    d = NList - MList;

    denomVp = sum(d .* rvList, 2);
    denomVp(abs(denomVp) < opts.tolDen) = NaN;

    vpList = sum(d .* ruList, 2) ./ denomVp;
    dpList = rvList .* vpList - ruList;

    den = cross2Vec(d, dpList);
    den(abs(den) < opts.tolDen) = NaN;

    lambdaList = cross2Vec(ruList, d) ./ den;
    GList = MList + lambdaList .* d;

    %% 9. Validity
    % A fixed chord can remain valid even when the differential envelope
    % point G is undefined, for example on a locally straight path. Keep a
    % chord-only view for binary swept-link validators without changing the
    % historical validLine/validSegment semantics used by CSSC.
    validChord = feasible ...
        & lengthOK ...
        & isfinite(vList) ...
        & vList > uList ...
        & vList <= vMax ...
        & all(isfinite(MList), 2) ...
        & all(isfinite(NList), 2);
    vChordList = vList;
    NChordList = NList;
    chordResidualList = residualList;

    validLine = feasible ...
        & lengthOK ...
        & isfinite(vList) ...
        & vList > uList ...
        & vList <= vMax ...
        & isfinite(vpList) ...
        & isfinite(lambdaList) ...
        & all(isfinite(GList), 2);

    validSegment = validLine ...
        & lambdaList >= -opts.lambdaTol ...
        & lambdaList <= 1 + opts.lambdaTol;

    %% 10. Set invalid outputs to NaN
    invalid = ~validLine;

    vList(invalid) = NaN;
    vpList(invalid) = NaN;
    lambdaList(invalid) = NaN;
    residualList(invalid) = NaN;
    NList(invalid,:) = NaN;
    GList(invalid,:) = NaN;

    %% 11. Pack output
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
    env.rawResidual = rawResidualList;
    env.validChord = validChord;
    env.vChord = vChordList;
    env.NChord = NChordList;
    env.chordResidual = chordResidualList;
    env.opts = opts;

    env.stats = struct();
    env.stats.newtonUsed = newtonUsed;
    env.stats.newtonSuccess = newtonConverged;
    env.stats.fallbackUsed = false(nU,1);
    env.stats.newtonIters = newtonIters;

    env.stats.numNewtonUsed = sum(newtonUsed);
    env.stats.numNewtonSuccess = sum(newtonConverged);
    env.stats.numFallbackUsed = 0;
    env.stats.avgNewtonIters = mean(newtonIters(newtonIters > 0), 'omitnan');
    env.stats.numFeasible = sum(feasible);
    env.stats.numLengthOK = sum(lengthOK & feasible);
    env.stats.numValidChord = sum(validChord);
    env.stats.maxAbsResidual = max(abs(residualList), [], 'omitnan');
    env.stats.meanAbsResidual = mean(abs(residualList), 'omitnan');
end

%% ============================================================
% Options
%% ============================================================

function opts = setDefaultOpts(opts)
    if nargin < 1 || isempty(opts)
        opts = struct();
    end

    if ~isfield(opts, 'uRange'); opts.uRange = [0, 1]; end
    if ~isfield(opts, 'vSearchRange'); opts.vSearchRange = [0, 1]; end
    if ~isfield(opts, 'nU'); opts.nU = 150; end

    if ~isfield(opts, 'epsV'); opts.epsV = 1e-8; end
    if ~isfield(opts, 'tolDen'); opts.tolDen = 1e-10; end
    if ~isfield(opts, 'lambdaTol'); opts.lambdaTol = 1e-9; end
    if ~isfield(opts, 'minSpeed'); opts.minSpeed = 1e-8; end

    % Selective Newton settings
    if ~isfield(opts, 'maxNewtonIter'); opts.maxNewtonIter = 2; end
    if ~isfield(opts, 'vResidualTol'); opts.vResidualTol = 1e-5; end
    if ~isfield(opts, 'vAcceptTol'); opts.vAcceptTol = 5e-4; end
    if ~isfield(opts, 'newtonTriggerTol'); opts.newtonTriggerTol = opts.vAcceptTol; end
    if ~isfield(opts, 'vStepTol'); opts.vStepTol = 1e-10; end
    if ~isfield(opts, 'maxNewtonStep'); opts.maxNewtonStep = 0.20; end
end

%% ============================================================
% Helpers
%% ============================================================

function A = ensureNx2(A)
    if isempty(A)
        A = nan(0,2);
        return;
    end

    if size(A,2) == 2
        return;
    end

    if size(A,1) == 2
        A = A.';
        return;
    end

    error('Input must be N-by-2 or 2-by-N.');
end

function n = vecNorm2(A)
    n = sqrt(sum(A.^2, 2));
end

function z = cross2Vec(a, b)
    z = a(:,1).*b(:,2) - a(:,2).*b(:,1);
end

function y = clampVec(x, a, b)
    y = min(max(x, a), b);
end
