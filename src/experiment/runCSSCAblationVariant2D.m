function out = runCSSCAblationVariant2D(variant, candidate, obstacles, ...
        paramsOptBase, paramsEval, runOpts, initialMetrics)
%RUNCSSCABLATIONVARIANT2D Run one paired CSSC ablation variant.

    if nargin < 7 || isempty(initialMetrics)
        initialMetrics = evaluateCSSCHighPrecision( ...
            candidate.Pinit, obstacles, withKnot(paramsEval, candidate));
    end
    out = emptyOutput();
    out.variant = variant;
    out.initialMetrics = initialMetrics;
    out.candidate = candidate;
    out.paths.pathRRT = candidate.pathRRT;
    out.paths.pathShortcut = candidate.pathShortcut;
    out.paths.Pinit = candidate.Pinit;
    out.paths.Pref = candidate.Pinit;

    if ~candidate.valid || isempty(candidate.Pinit)
        out.stopReason = 'invalid-initial-candidate';
        out.finalMetrics = evaluateCSSCHighPrecision([], obstacles, paramsEval);
        return;
    end

    params = paramsOptBase;
    params.knot = candidate.splineInfo.knot;
    params.solver.objectiveMode = variant.objectiveMode;
    if variant.runOptimizer
        params.solver.gradMode = variant.gradMode;
    end
    params.returnPolicy = variant.returnPolicy;
    params.activeMode = variant.activeMode;
    params.wClear = params.wClear * variant.wClearScale;
    if ~variant.useSafetyBuffer
        params.dMin = runOpts.finalDMin;
    end
    if variant.disablePatience
        params.stop.patience = realmax;
    end
    params.stop.maxTimeSec = max(0, runOpts.optimizationBudgetSec);

    Pinit = candidate.Pinit;
    Pref = Pinit; %#ok<NASGU> Used by the evalc expression below.
    if variant.runOptimizer && params.stop.maxTimeSec > 0
        tOpt = tic;
        [out.consoleLog, Popt, optimizerInfo] = evalc( ...
            'optimizeCSSC2D(Pinit, Pref, obstacles, params)');
        out.optimizationTimeSec = toc(tOpt);
    elseif variant.runOptimizer
        Popt = Pinit;
        optimizerInfo = struct('method', variant.label, ...
            'numIterActual', 0, 'stopReason', 'noOptimizationBudget', ...
            'returnedSafe', false, 'objectiveMode', variant.objectiveMode, ...
            'gradMode', variant.gradMode);
    else
        Popt = Pinit;
        optimizerInfo = struct('method', variant.label, ...
            'numIterActual', 0, 'stopReason', 'initial-only', ...
            'returnedSafe', initialMetrics.dMinSatisfied, ...
            'objectiveMode', 'none', 'gradMode', 'none', ...
            'elapsedTimeSec', 0);
    end

    evalParams = withKnot(paramsEval, candidate);
    tEval = tic;
    finalMetrics = evaluateCSSCHighPrecision(Popt, obstacles, evalParams);
    out.finalEvaluationTimeSec = toc(tEval);
    out.success = finalMetrics.dMinSatisfied && ...
        finalMetrics.ftlGeometricRealizable;
    out.stopReason = getField(optimizerInfo, 'stopReason', '');
    out.Popt = Popt;
    out.paths.Popt = Popt;
    out.paths.pathBSplineInit = initialMetrics.pathSample;
    out.paths.pathOptimized = finalMetrics.pathSample;
    out.optimizerInfo = optimizerInfo;
    out.finalMetrics = finalMetrics;
    out.paramsUsed = params;
end

function params = withKnot(params, candidate)
    params.knot = candidate.splineInfo.knot;
end

function value = getField(s, name, defaultValue)
    if isstruct(s) && isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end

function out = emptyOutput()
    out = struct();
    out.success = false;
    out.stopReason = '';
    out.consoleLog = '';
    out.optimizationTimeSec = 0;
    out.finalEvaluationTimeSec = 0;
    out.Popt = zeros(0,2);
    out.variant = struct();
    out.candidate = struct();
    out.optimizerInfo = struct();
    out.initialMetrics = struct();
    out.finalMetrics = struct();
    out.paramsUsed = struct();
    out.paths = struct('Pinit', [], 'Popt', [], 'Pref', [], ...
        'pathRRT', [], 'pathShortcut', [], ...
        'pathBSplineInit', [], 'pathOptimized', []);
end
