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

    c = state.clearance(:);
    valid = isfinite(c) & state.validLine(:);
    idxValid = find(valid);

    if isempty(idxValid)
        active.indices = [];
        active.threshold = params.dMin + params.activeClearanceMargin;
        active.mode = 'empty';
        return;
    end

    threshold = params.dMin + params.activeClearanceMargin;
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
