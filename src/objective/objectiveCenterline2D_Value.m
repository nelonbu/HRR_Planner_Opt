function [J, details] = objectiveCenterline2D_Value( ...
        P, Pref, obstacles, params)
%OBJECTIVECENTERLINE2D_VALUE Point-SDF value used by finite differences.

    tAll = tic;
    tEval = tic;
    state = evaluateCenterlineClearance2D(P, obstacles, params);
    dtEval = toc(tEval);

    tObs = tic;
    active = selectActiveCSSCSamples(state, params);
    c = state.clearance(active.indices);
    c = c(~isnan(c));
    if isempty(c)
        if isfield(params, 'invalidClearancePenalty')
            invalidPenalty = params.invalidClearancePenalty;
        else
            invalidPenalty = 1e6;
        end
        Jobs = invalidPenalty;
        Jclear = invalidPenalty;
    else
        Jobs = params.wObs * mean(max(0, params.dMin-c).^2);
        Jclear = params.wClear * mean(max(0, params.dPref-c).^2);
    end
    dtObs = toc(tObs);

    tReg = tic;
    [Jreg, ~, reg] = regularizationCostGrad2D(P, Pref, params);
    dtReg = toc(tReg);
    J = Jobs + Jclear + Jreg;
    total = toc(tAll);

    details = struct('J', J, 'Jobs', Jobs, 'Jclear', Jclear, ...
        'Jreg', Jreg, 'Jref', reg.Jref, 'Jsmooth', reg.Jsmooth, ...
        'Jlen', reg.Jlen, 'Jtrust', reg.Jtrust, ...
        'minClear', state.minClear, 'state', state, 'active', active, ...
        'timing', struct('total', total, 'evaluate', dtEval, ...
        'obstacleGrad', dtObs, 'regularization', dtReg, ...
        'other', max(0, total-dtEval-dtObs-dtReg)));
end
