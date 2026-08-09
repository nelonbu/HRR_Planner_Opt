function [J, details, gradP] = objectiveCSSC2D_SemiGrad(P, Pref, obstacles, params)
%OBJECTIVECSSC2D_SEMIGRAD CSSC objective with semi-analytic gradient.
%
% The semi-analytic approximation fixes the current chord parameters u, v
% and closest-point alpha for one iteration. The closest point on a chord is:
%   q = (1-alpha) r(u) + alpha r(v)
% so:
%   dq/dP_i ~= ((1-alpha) B_i(u) + alpha B_i(v)) I.

    tAll = tic;

    gradP = zeros(size(P));
    nCtrl = size(P,1);
    degree = params.degree;
    if isfield(params, 'knot') && ~isempty(params.knot)
        knot = params.knot;
    else
        knot = makeClampedUniformKnot(nCtrl, degree);
    end

    %% 1. Global envelope/segment clearance evaluation
    tEval = tic;
    state = evaluateCSSCGlobal(P, obstacles, params);
    dtEval = toc(tEval);

    %% 2. Active set
    tObs = tic;
    active = selectActiveCSSCSamples(state, params);

    Jobs = 0;
    Jclear = 0;
    nActive = numel(active.indices);

    for a = 1:nActive
        j = active.indices(a);

        if ~state.validLine(j) || ~isfinite(state.clearance(j))
            continue;
        end

        u = state.u(j);
        v = state.v(j);
        alpha = state.closestAlpha(j);
        q = state.closestPoint(j,:);
        normal = state.closestNormal(j,:);
        clearance = state.clearance(j);

        if any(~isfinite([u, v, alpha, q, normal, clearance]))
            continue;
        end

        gapObs = params.dMin - clearance;
        dJdq = [0, 0];

        if gapObs > 0
            Jobs = Jobs + params.wObs * gapObs^2;
            dJdq = dJdq - 2 * params.wObs * gapObs * normal;
        end

        if isfield(params, 'wClear') && params.wClear > 0 && isfield(params, 'dPref')
            gapClear = params.dPref - clearance;
            if gapClear > 0
                Jclear = Jclear + params.wClear * gapClear^2;
                dJdq = dJdq - 2 * params.wClear * gapClear * normal;
            end
        end

        if any(dJdq ~= 0)
            Bu = bsplineBasisVector(u, nCtrl, degree, knot);
            Bv = bsplineBasisVector(v, nCtrl, degree, knot);
            coeff = (1 - alpha) * Bu + alpha * Bv;

            idxNz = find(abs(coeff) > 1e-12);
            for ii = idxNz
                gradP(ii,:) = gradP(ii,:) + coeff(ii) * dJdq;
            end
        end
    end

    if nActive > 0
        Jobs = Jobs / nActive;
        Jclear = Jclear / nActive;
        gradP = gradP / nActive;
    else
        invalidPenalty = getInvalidClearancePenalty(params);
        Jobs = invalidPenalty;
        Jclear = invalidPenalty;
    end

    dtObs = toc(tObs);

    %% 3. Regularization cost and gradients
    tReg = tic;
    [Jreg, gradReg, regDetails] = regularizationCostGrad2D(P, Pref, params);
    dtReg = toc(tReg);

    J = Jobs + Jclear + Jreg;
    gradP = gradP + gradReg;

    % Keep endpoints fixed.
    gradP(1,:) = 0;
    gradP(end,:) = 0;

    dtAll = toc(tAll);

    details = struct();
    details.J = J;
    details.Jobs = Jobs;
    details.Jclear = Jclear;
    details.Jreg = Jreg;
    details.Jref = regDetails.Jref;
    details.Jsmooth = regDetails.Jsmooth;
    details.Jlen = regDetails.Jlen;
    details.Jtrust = regDetails.Jtrust;
    details.minClear = state.minClear;
    details.finalMinClear = state.minClear;
    details.state = state;
    details.active = active;
    details.gradNorm = norm(gradP(:));
    details.timing = struct('total', dtAll, 'evaluate', dtEval, 'obstacleGrad', dtObs, 'regularization', dtReg, 'other', max(0, dtAll-dtEval-dtObs-dtReg));
end

function value = getInvalidClearancePenalty(params)
    if isfield(params, 'invalidClearancePenalty') && ...
            isscalar(params.invalidClearancePenalty) && ...
            isfinite(params.invalidClearancePenalty) && ...
            params.invalidClearancePenalty >= 0
        value = params.invalidClearancePenalty;
    else
        value = 1e6;
    end
end
