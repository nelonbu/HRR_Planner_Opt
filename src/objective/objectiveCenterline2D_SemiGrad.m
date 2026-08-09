function [J, details, gradP] = objectiveCenterline2D_SemiGrad( ...
        P, Pref, obstacles, params)
%OBJECTIVECENTERLINE2D_SEMIGRAD Point-SDF ablation objective and gradient.

    tAll = tic;
    nCtrl = size(P,1);
    degree = params.degree;
    if isfield(params, 'knot') && ~isempty(params.knot)
        knot = params.knot;
    else
        knot = makeClampedUniformKnot(nCtrl, degree);
    end

    tEval = tic;
    state = evaluateCenterlineClearance2D(P, obstacles, params);
    dtEval = toc(tEval);

    tObs = tic;
    active = selectActiveCSSCSamples(state, params);
    gradP = zeros(size(P));
    Jobs = 0;
    Jclear = 0;
    nActive = numel(active.indices);

    for k = 1:nActive
        j = active.indices(k);
        c = state.clearance(j);
        n = state.closestNormal(j,:);
        if ~isfinite(c) || any(~isfinite(n))
            continue;
        end

        dJdq = [0, 0];
        obsGap = params.dMin - c;
        if obsGap > 0
            Jobs = Jobs + params.wObs * obsGap^2;
            dJdq = dJdq - 2 * params.wObs * obsGap * n;
        end
        clearGap = params.dPref - c;
        if params.wClear > 0 && clearGap > 0
            Jclear = Jclear + params.wClear * clearGap^2;
            dJdq = dJdq - 2 * params.wClear * clearGap * n;
        end

        if any(dJdq ~= 0)
            B = bsplineBasisVector(state.u(j), nCtrl, degree, knot);
            gradP = gradP + B(:) * dJdq;
        end
    end

    if nActive > 0
        Jobs = Jobs / nActive;
        Jclear = Jclear / nActive;
        gradP = gradP / nActive;
    else
        invalidPenalty = getInvalidPenalty(params);
        Jobs = invalidPenalty;
        Jclear = invalidPenalty;
    end
    dtObs = toc(tObs);

    tReg = tic;
    [Jreg, gradReg, reg] = regularizationCostGrad2D(P, Pref, params);
    dtReg = toc(tReg);
    gradP = gradP + gradReg;
    gradP(1,:) = 0;
    gradP(end,:) = 0;
    J = Jobs + Jclear + Jreg;

    details = packDetails(J, Jobs, Jclear, Jreg, reg, state, active, ...
        norm(gradP(:)), toc(tAll), dtEval, dtObs, dtReg);
end

function details = packDetails(J, Jobs, Jclear, Jreg, reg, state, ...
        active, gradNorm, total, evalTime, obsTime, regTime)
    details = struct('J', J, 'Jobs', Jobs, 'Jclear', Jclear, ...
        'Jreg', Jreg, 'Jref', reg.Jref, 'Jsmooth', reg.Jsmooth, ...
        'Jlen', reg.Jlen, 'Jtrust', reg.Jtrust, ...
        'minClear', state.minClear, 'finalMinClear', state.minClear, ...
        'state', state, 'active', active, 'gradNorm', gradNorm, ...
        'timing', struct('total', total, 'evaluate', evalTime, ...
        'obstacleGrad', obsTime, 'regularization', regTime, ...
        'other', max(0, total-evalTime-obsTime-regTime)));
end

function value = getInvalidPenalty(params)
    if isfield(params, 'invalidClearancePenalty')
        value = params.invalidClearancePenalty;
    else
        value = 1e6;
    end
end
