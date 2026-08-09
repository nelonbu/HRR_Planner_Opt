function [J, details] = objectiveCSSC2D_Value(P, Pref, obstacles, params)
%OBJECTIVECSSC2D_VALUE CSSC objective value only, for finite-difference mode.

    tAll = tic;

    tEval = tic;
    state = evaluateCSSCGlobal(P, obstacles, params);
    dtEval = toc(tEval);

    tObs = tic;
    active = selectActiveCSSCSamples(state, params);
    c = state.clearance(active.indices);
    c = c(~isnan(c));

    if isempty(c)
        invalidPenalty = getInvalidClearancePenalty(params);
        Jobs = invalidPenalty;
        Jclear = invalidPenalty;
    else
        obsGap = max(0, params.dMin - c);
        clearGap = max(0, params.dPref - c);
        Jobs = params.wObs * mean(obsGap.^2);
        Jclear = params.wClear * mean(clearGap.^2);
    end
    dtObs = toc(tObs);

    tReg = tic;
    [Jreg, ~, regDetails] = regularizationCostGrad2D(P, Pref, params);
    dtReg = toc(tReg);

    J = Jobs + Jclear + Jreg;
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
    details.state = state;
    details.active = active;
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
