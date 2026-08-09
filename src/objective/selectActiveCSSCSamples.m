function active = selectActiveCSSCSamples(state, params)
%SELECTACTIVECSSCSAMPLES Select active clearance samples for optimization.
%
% The active set contains samples with clearance below dMin + margin. If no
% such samples exist, top-K smallest clearance samples are selected to improve
% preferred clearance.

    if ~isfield(params, 'activeClearanceMargin')
        params.activeClearanceMargin = 0.05;
    end
    if ~isfield(params, 'activeTopK')
        params.activeTopK = 20;
    end
    if ~isfield(params, 'activeMode') || isempty(params.activeMode)
        params.activeMode = 'topk';
    end

    c = state.clearance(:);
    % Positive Inf is a valid "no obstacle" clearance. NaN denotes an
    % unavailable chord-clearance evaluation and must not enter the set.
    valid = ~isnan(c) & state.validLine(:);

    % In hybrid clearance mode, G/M/N probes are used to select candidate
    % chords, but active optimization samples should use exact segment
    % clearance whenever such refined samples exist.
    if isfield(state, 'paramsUsed') ...
            && isfield(state.paramsUsed, 'clearanceMode') ...
            && strcmpi(state.paramsUsed.clearanceMode, 'hybrid') ...
            && isfield(state, 'isExactSegment')
        exact = state.isExactSegment(:);
        if any(valid & exact)
            valid = valid & exact;
        end
    end

    idxValid = find(valid);

    if isempty(idxValid)
        active.indices = [];
        active.threshold = params.dMin + params.activeClearanceMargin;
        active.mode = 'empty';
        return;
    end

    threshold = params.dMin + params.activeClearanceMargin;
    activeMode = lower(char(string(params.activeMode)));
    if strcmp(activeMode, 'all')
        active.indices = idxValid(:);
        active.threshold = threshold;
        active.mode = 'all';
        return;
    elseif ~any(strcmp(activeMode, {'topk','threshold-topk'}))
        error('Unknown params.activeMode: %s', char(params.activeMode));
    end

    idxUnsafe = idxValid(c(idxValid) < threshold);

    if isempty(idxUnsafe)
        [~, order] = sort(c(idxValid), 'ascend');
        K = min(params.activeTopK, numel(order));
        activeIdx = idxValid(order(1:K));
        mode = 'topk';
    else
        [~, order] = sort(c(idxUnsafe), 'ascend');
        K = min(params.activeTopK, numel(order));
        activeIdx = idxUnsafe(order(1:K));
        mode = 'threshold';
    end

    active.indices = activeIdx(:);
    active.threshold = threshold;
    active.mode = mode;
end
