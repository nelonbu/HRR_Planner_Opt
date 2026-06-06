function [J, details] = objectiveCSSC2D(Pctrl, Pref, obstacles, params)
%OBJECTIVECSSC2D Objective for continuous swept-segment clearance planning.
%
% Timing debug added:
%   details.timing.envelope
%   details.timing.obstacle
%   details.timing.regularization
%   details.timing.curvature
%
% Cost debug added:
%   details.cost.obs
%   details.cost.clear
%   details.cost.invalid
%   details.cost.ref
%   details.cost.smooth
%   details.cost.length
%   details.cost.curv

    params = setDefaultParamsLocal(params);

    details = struct();
    details.timing = emptyTiming();
    details.cost = emptyCost();

    J = 0;

    %% ---------- 1. Path and fixed chord envelope ----------
    tEnvelope = tic;

    pathOpts.degree = params.degree;
    rFun  = @(w) evalBSplinePath2D(Pctrl, w, pathOpts);
    drFun = @(w) evalBSplineDerivOnly(Pctrl, w, pathOpts);

    env = fixedChordEnvelope(rFun, drFun, params.L, params.envOpts);

    valid = env.validLine & all(isfinite(env.M), 2) & all(isfinite(env.N), 2);
    M = env.M(valid, :);
    N = env.N(valid, :);

    details.timing.envelope = toc(tEnvelope);

    %% ---------- 2. Obstacle / clearance term ----------
    tObstacle = tic;

    nObs = size(obstacles, 1);
    phi = inf(nObs, 1);
    bottleneckU = nan(nObs, 1);
    bottleneckSegPoint = nan(nObs, 2);

    if isempty(M)
        details.cost.invalid = params.wInvalid;
        J = J + details.cost.invalid;
    else
        uValid = env.u(valid);

        for i = 1:nObs
            c = obstacles(i, 1:2);
            rObs = obstacles(i, 3);

            [distList, closestPts] = pointSegmentDistanceBatch2D(c, M, N);
            clearanceList = distList - rObs - params.robotRadius;

            [phi(i), idx] = min(clearanceList);
            bottleneckU(i) = uValid(idx);
            bottleneckSegPoint(i, :) = closestPts(idx, :);

            % Hard safety margin penalty.
            gapObs = params.dMin - phi(i);
            cObs = params.wObs * smoothHinge2(gapObs, params.epsObs);
            details.cost.obs = details.cost.obs + cObs;
            J = J + cObs;

            % Preferred clearance buffer.
            gapClear = params.dPref - phi(i);
            cClear = params.wClear * smoothHinge2(gapClear, params.epsClear);
            details.cost.clear = details.cost.clear + cClear;
            J = J + cClear;
        end
    end

    details.timing.obstacle = toc(tObstacle);

    %% ---------- 3. Reference / smoothness / length ----------
    tReg = tic;

    % Reference control point cost.
    D = Pctrl - Pref;
    D(1, :) = 0;
    D(end, :) = 0;
    details.cost.ref = params.wRef * sum(D(:).^2);
    J = J + details.cost.ref;

    % Smoothness cost on control polygon.
    E = diff(Pctrl, 2, 1);
    details.cost.smooth = params.wSmooth * sum(E(:).^2);
    J = J + details.cost.smooth;

    % Control polygon length cost.
    seg = diff(Pctrl, 1, 1);
    ctrlLen = sum(sqrt(sum(seg.^2, 2) + 1e-12));
    details.cost.length = params.wLength * ctrlLen;
    J = J + details.cost.length;

    details.timing.regularization = toc(tReg);

    %% ---------- 4. Curvature penalty on the B-spline curve ----------
    tCurv = tic;

    if params.wCurv > 0
        wq = linspace(0, 1, params.curvSamples).';
        [~, dr, d2r] = evalBSplinePath2D(Pctrl, wq, pathOpts);

        crossVal = abs(dr(:,1).*d2r(:,2) - dr(:,2).*d2r(:,1));
        speed = sqrt(sum(dr.^2, 2));
        kappa = crossVal ./ (speed.^3 + 1e-9);

        excess = max(0, kappa - params.kappaMax);
        details.cost.curv = params.wCurv * mean(excess.^2);
        J = J + details.cost.curv;
    else
        kappa = [];
        details.cost.curv = 0;
    end

    details.timing.curvature = toc(tCurv);

    %% ---------- 5. Details ----------
    details.env = env;
    details.phi = phi;

    if isempty(phi)
        details.minClear = inf;
    else
        details.minClear = min(phi);
    end

    details.bottleneckU = bottleneckU;
    details.bottleneckSegPoint = bottleneckSegPoint;
    details.ctrlLen = ctrlLen;
    details.kappa = kappa;
    details.J = J;
end

%% ============================================================
% Local helpers
%% ============================================================

function timing = emptyTiming()
    timing = struct();
    timing.envelope = 0;
    timing.obstacle = 0;
    timing.regularization = 0;
    timing.curvature = 0;
end

function cost = emptyCost()
    cost = struct();
    cost.obs = 0;
    cost.clear = 0;
    cost.invalid = 0;
    cost.ref = 0;
    cost.smooth = 0;
    cost.length = 0;
    cost.curv = 0;
end

function dr = evalBSplineDerivOnly(Pctrl, w, pathOpts)
    [~, dr] = evalBSplinePath2D(Pctrl, w, pathOpts);
end

function [distList, closestPts] = pointSegmentDistanceBatch2D(c, M, N)
    V = N - M;
    W = c - M;

    denom = sum(V.^2, 2);
    denom = max(denom, 1e-12);

    alpha = sum(W .* V, 2) ./ denom;
    alpha = min(1, max(0, alpha));

    closestPts = M + alpha .* V;
    diff = c - closestPts;
    distList = sqrt(sum(diff.^2, 2));
end

function y = smoothHinge2(x, epsVal)
    % Smooth approximation of max(x,0)^2.
    sp = softplusSmooth(x, epsVal);
    y = sp^2;
end

function y = softplusSmooth(x, epsVal)
    z = x / epsVal;

    if z > 50
        y = x;
    elseif z < -50
        y = epsVal * exp(z);
    else
        y = epsVal * log1p(exp(z));
    end
end

function params = setDefaultParamsLocal(params)
    if ~isfield(params, 'degree'); params.degree = 3; end
    if ~isfield(params, 'robotRadius'); params.robotRadius = 0.05; end
    if ~isfield(params, 'dMin'); params.dMin = 0.02; end
    if ~isfield(params, 'dPref'); params.dPref = 0.25; end

    if ~isfield(params, 'wObs'); params.wObs = 200.0; end
    if ~isfield(params, 'wClear'); params.wClear = 2.0; end
    if ~isfield(params, 'wRef'); params.wRef = 0.05; end
    if ~isfield(params, 'wSmooth'); params.wSmooth = 1.0; end
    if ~isfield(params, 'wLength'); params.wLength = 0.01; end
    if ~isfield(params, 'wCurv'); params.wCurv = 0.0; end

    if ~isfield(params, 'kappaMax'); params.kappaMax = 2.0; end
    if ~isfield(params, 'curvSamples'); params.curvSamples = 80; end

    if ~isfield(params, 'epsObs'); params.epsObs = 0.02; end
    if ~isfield(params, 'epsClear'); params.epsClear = 0.05; end

    if ~isfield(params, 'wInvalid'); params.wInvalid = 1e6; end

    if ~isfield(params, 'envOpts')
        params.envOpts = struct('nU', 120, 'nVGrid', 160, 'uRange', [0 1]);
    end
end