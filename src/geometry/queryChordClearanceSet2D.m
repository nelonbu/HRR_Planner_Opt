function out = queryChordClearanceSet2D(env, obstacles, params)
%QUERYCHORDCLEARANCESET2D Clearance backend for fixed-chord CSSC samples.
%
% params.clearanceMode:
%   'segment'  : exact segment-obstacle clearance for every valid chord.
%   'envelope' : fast probe using M/N/G point SDF only; not a strict check.
%   'hybrid'   : M/N/G probe first, then exact segment clearance near dMin.
%
% Hybrid defaults:
%   params.hybrid.triggerFactor = 2.0;
%   params.hybrid.refineActiveTopK = true;
%   params.hybrid.forceExactStride = 0;

    if nargin < 3 || isempty(params)
        params = struct();
    end
    params = setDefaults(params);

    tTotal = tic;

    mode = lower(string(params.clearanceMode));
    [MQuery, NQuery, vQuery, validLineMask, validSegmentMask, ...
        validitySource] = selectQueryGeometry(env, mode);
    n = numel(env.u);

    idxValidLine = find(validLineMask);
    idxValidSegment = find(validSegmentMask);

    useObstacleFilter = params.obstacleFilter.enable;
    if useObstacleFilter
        obstacleBounds = computeObstacleBoundingCircles2D(obstacles);
    else
        obstacleBounds = [];
    end
    filterStats = emptyFilterStats();
    nObs = numel(obstacles);

    clearance = nan(n, 1);
    clearanceSegment = nan(n, 1);
    clearanceProbe = nan(n, 1);
    clearanceG = nan(n, 1);
    clearanceM = nan(n, 1);
    clearanceN = nan(n, 1);

    closestPoint = nan(n, 2);
    closestAlpha = nan(n, 1);
    closestNormal = nan(n, 2);
    probePoint = nan(n, 2);
    probeAlpha = nan(n, 1);
    probeNormal = nan(n, 2);

    clearanceSource = strings(n, 1);
    probeSource = strings(n, 1);
    isExactSegment = false(n, 1);

    if params.enableObstacleMetadata
        nearestObsId = nan(n, 1);
        nearestObsType = strings(n, 1);
    else
        nearestObsId = [];
        nearestObsType = strings(0, 1);
    end

    timeProbeMN = 0;
    timeProbeG = 0;
    timeSegment = 0;

    needProbe = params.enablePointClearance ...
        || mode == "envelope" ...
        || mode == "hybrid";

    if needProbe
        t = tic;
        for kk = 1:numel(idxValidLine)
            i = idxValidLine(kk);

            if useObstacleFilter && params.obstacleFilter.useForPoint
                [dM, gM, infoM, fsM] = queryObstaclePointSDFFiltered2D( ...
                    MQuery(i,:), obstacles, obstacleBounds, params.obstacleFilter);
                [dN, gN, infoN, fsN] = queryObstaclePointSDFFiltered2D( ...
                    NQuery(i,:), obstacles, obstacleBounds, params.obstacleFilter);
                filterStats = addPointFilterStats(filterStats, fsM);
                filterStats = addPointFilterStats(filterStats, fsN);
            else
                [dM, gM, infoM] = queryObstaclePointSDF2D(MQuery(i,:), obstacles);
                [dN, gN, infoN] = queryObstaclePointSDF2D(NQuery(i,:), obstacles);
                filterStats.pointQueries = filterStats.pointQueries + 2;
                filterStats.pointObstacleChecks = filterStats.pointObstacleChecks + 2*nObs;
                filterStats.pointObstaclePossible = filterStats.pointObstaclePossible + 2*nObs;
            end

            clearanceM(i) = dM;
            clearanceN(i) = dN;

            [bestD, which] = min([dM, dN]);
            if which == 1
                clearanceProbe(i) = bestD;
                probePoint(i,:) = MQuery(i,:);
                probeAlpha(i) = 0;
                probeNormal(i,:) = gM;
                probeSource(i) = "M";
                if params.enableObstacleMetadata
                    nearestObsId(i) = infoM.id;
                    nearestObsType(i) = string(infoM.type);
                end
            else
                clearanceProbe(i) = bestD;
                probePoint(i,:) = NQuery(i,:);
                probeAlpha(i) = 1;
                probeNormal(i,:) = gN;
                probeSource(i) = "N";
                if params.enableObstacleMetadata
                    nearestObsId(i) = infoN.id;
                    nearestObsType(i) = string(infoN.type);
                end
            end
        end
        timeProbeMN = toc(t);

        t = tic;
        for kk = 1:numel(idxValidSegment)
            i = idxValidSegment(kk);

            if useObstacleFilter && params.obstacleFilter.useForPoint
                [dG, gG, infoG, fsG] = queryObstaclePointSDFFiltered2D( ...
                    env.G(i,:), obstacles, obstacleBounds, params.obstacleFilter);
                filterStats = addPointFilterStats(filterStats, fsG);
            else
                [dG, gG, infoG] = queryObstaclePointSDF2D(env.G(i,:), obstacles);
                filterStats.pointQueries = filterStats.pointQueries + 1;
                filterStats.pointObstacleChecks = filterStats.pointObstacleChecks + nObs;
                filterStats.pointObstaclePossible = filterStats.pointObstaclePossible + nObs;
            end
            clearanceG(i) = dG;

            if ~isfinite(clearanceProbe(i)) || dG < clearanceProbe(i)
                clearanceProbe(i) = dG;
                probePoint(i,:) = env.G(i,:);
                probeAlpha(i) = env.lambda(i);
                probeNormal(i,:) = gG;
                probeSource(i) = "G";
                if params.enableObstacleMetadata
                    nearestObsId(i) = infoG.id;
                    nearestObsType(i) = string(infoG.type);
                end
            end
        end
        timeProbeG = toc(t);
    end

    switch mode
        case "segment"
            exactMask = validLineMask;

        case "envelope"
            exactMask = false(n, 1);

        case "hybrid"
            exactMask = makeHybridExactMask( ...
                validLineMask, clearanceProbe, params);

        otherwise
            error('Unknown params.clearanceMode: %s', char(params.clearanceMode));
    end

    if any(exactMask)
        t = tic;
        idxExact = find(exactMask);
        for kk = 1:numel(idxExact)
            i = idxExact(kk);

            if useObstacleFilter && params.obstacleFilter.useForSegment
                [outSeg, fsSeg] = querySegmentObstacleClearanceFiltered2D( ...
                    MQuery(i,:), NQuery(i,:), obstacles, obstacleBounds, ...
                    params.obstacleFilter);
                filterStats = addSegmentFilterStats(filterStats, fsSeg);
            else
                outSeg = querySegmentObstacleClearance2D( ...
                    MQuery(i,:), NQuery(i,:), obstacles);
                filterStats.segmentQueries = filterStats.segmentQueries + 1;
                filterStats.segmentObstacleChecks = filterStats.segmentObstacleChecks + nObs;
                filterStats.segmentObstaclePossible = filterStats.segmentObstaclePossible + nObs;
            end

            clearanceSegment(i) = outSeg.clearance;
            isExactSegment(i) = true;
            clearance(i) = outSeg.clearance;
            closestPoint(i,:) = outSeg.closestPoint;
            closestAlpha(i) = outSeg.alpha;
            closestNormal(i,:) = outSeg.normal;
            clearanceSource(i) = "segment";

            if params.enableObstacleMetadata
                nearestObsId(i) = outSeg.obsId;
                nearestObsType(i) = string(outSeg.obsType);
            end
        end
        timeSegment = toc(t);
    end

    if mode == "envelope" || mode == "hybrid"
        useProbe = validLineMask & ~isExactSegment;
        clearance(useProbe) = clearanceProbe(useProbe);
        closestPoint(useProbe,:) = probePoint(useProbe,:);
        closestAlpha(useProbe) = probeAlpha(useProbe);
        closestNormal(useProbe,:) = probeNormal(useProbe,:);
        clearanceSource(useProbe) = probeSource(useProbe);
    end

    out = struct();
    out.clearance = clearance;
    out.clearanceSegment = clearanceSegment;
    out.clearanceProbe = clearanceProbe;
    out.clearanceG = clearanceG;
    out.clearanceM = clearanceM;
    out.clearanceN = clearanceN;
    out.closestPoint = closestPoint;
    out.closestAlpha = closestAlpha;
    out.closestNormal = closestNormal;
    out.clearanceSource = clearanceSource;
    out.probeSource = probeSource;
    out.isExactSegment = isExactSegment;
    out.validLineMask = validLineMask;
    out.validSegmentMask = validSegmentMask;
    out.idxValidLine = idxValidLine;
    out.idxValidSegment = idxValidSegment;
    out.MUsed = MQuery;
    out.NUsed = NQuery;
    out.vUsed = vQuery;
    out.validitySource = validitySource;
    out.nearestObsId = nearestObsId;
    out.nearestObsType = nearestObsType;

    filterStats = updateFilterTotals(filterStats);

    timing = struct();
    timing.total = toc(tTotal);
    timing.pointMN = timeProbeMN;
    timing.pointG = timeProbeG;
    timing.segmentClearance = timeSegment;
    timing.numExactSegment = nnz(isExactSegment);
    timing.numProbe = nnz(isfinite(clearanceProbe));
    timing.exactFraction = nnz(isExactSegment) / max(1, numel(idxValidLine));
    timing.clearanceMode = char(params.clearanceMode);
    if strcmpi(params.clearanceMode, 'hybrid')
        timing.hybridTriggerThreshold = params.hybrid.triggerFactor * params.dMin;
    else
        timing.hybridTriggerThreshold = nan;
    end
    timing.obstacleFilter = filterStats;
    out.timing = timing;
end

function [M, N, v, validLineMask, validSegmentMask, source] = ...
        selectQueryGeometry(env, mode)
    M = env.M;

    switch mode
        case "segment"
            % Exact segment clearance depends only on a successfully
            % constructed length-L chord. The differential envelope point G
            % may be undefined on straight or locally parallel chord
            % families and must not invalidate the physical segment.
            if isfield(env, 'validChord') && isfield(env, 'NChord')
                N = env.NChord;
                validLineMask = logical(env.validChord);
                source = 'validChord';
                if isfield(env, 'vChord')
                    v = env.vChord;
                else
                    v = env.v;
                end
            else
                % Compatibility fallback for legacy envelope structures.
                N = env.N;
                v = env.v;
                validLineMask = logical(env.validLine);
                source = 'validLine-legacy';
            end
            validSegmentMask = false(size(validLineMask));

        case {"envelope", "hybrid"}
            N = env.N;
            v = env.v;
            validLineMask = logical(env.validLine);
            validSegmentMask = logical(env.validSegment);
            source = 'validLine';

        otherwise
            error('Unknown params.clearanceMode: %s', char(mode));
    end

    validLineMask = validLineMask ...
        & all(isfinite(M), 2) ...
        & all(isfinite(N), 2) ...
        & isfinite(v);
    validSegmentMask = validSegmentMask ...
        & all(isfinite(env.G), 2) ...
        & isfinite(env.lambda);
end

function params = setDefaults(params)
    if ~isfield(params, 'clearanceMode') || isempty(params.clearanceMode)
        params.clearanceMode = 'segment';
    end
    if ~isfield(params, 'enablePointClearance')
        params.enablePointClearance = true;
    end
    if ~isfield(params, 'enableObstacleMetadata')
        params.enableObstacleMetadata = true;
    end
    if ~isfield(params, 'dMin')
        params.dMin = 0.02;
    end
    if ~isfield(params, 'activeTopK')
        params.activeTopK = 20;
    end
    if ~isfield(params, 'obstacleFilter') || isempty(params.obstacleFilter)
        params.obstacleFilter = struct();
    end
    if ~isfield(params.obstacleFilter, 'enable')
        params.obstacleFilter.enable = false;
    end
    if ~isfield(params.obstacleFilter, 'useForSegment')
        params.obstacleFilter.useForSegment = true;
    end
    if ~isfield(params.obstacleFilter, 'useForPoint')
        params.obstacleFilter.useForPoint = true;
    end
    if ~isfield(params.obstacleFilter, 'minCandidates')
        params.obstacleFilter.minCandidates = 1;
    end
    if ~isfield(params.obstacleFilter, 'stopTol')
        params.obstacleFilter.stopTol = 1e-12;
    end
    if ~isfield(params, 'hybrid') || isempty(params.hybrid)
        params.hybrid = struct();
    end
    if ~isfield(params.hybrid, 'triggerFactor')
        params.hybrid.triggerFactor = 2.0;
    end
    if ~isfield(params.hybrid, 'refineActiveTopK')
        params.hybrid.refineActiveTopK = true;
    end
    if ~isfield(params.hybrid, 'forceExactStride')
        params.hybrid.forceExactStride = 0;
    end
end

function stats = emptyFilterStats()
    stats = struct();
    stats.segmentQueries = 0;
    stats.segmentObstacleChecks = 0;
    stats.segmentObstaclePossible = 0;
    stats.pointQueries = 0;
    stats.pointObstacleChecks = 0;
    stats.pointObstaclePossible = 0;
    stats.totalObstacleChecks = 0;
    stats.totalObstaclePossible = 0;
    stats.checkFraction = nan;
end

function stats = addSegmentFilterStats(stats, fs)
    stats.segmentQueries = stats.segmentQueries + 1;
    stats.segmentObstacleChecks = stats.segmentObstacleChecks + fs.numChecked;
    stats.segmentObstaclePossible = stats.segmentObstaclePossible + fs.numObstacles;
    stats = updateFilterTotals(stats);
end

function stats = addPointFilterStats(stats, fs)
    stats.pointQueries = stats.pointQueries + 1;
    stats.pointObstacleChecks = stats.pointObstacleChecks + fs.numChecked;
    stats.pointObstaclePossible = stats.pointObstaclePossible + fs.numObstacles;
    stats = updateFilterTotals(stats);
end

function stats = updateFilterTotals(stats)
    stats.totalObstacleChecks = stats.segmentObstacleChecks + stats.pointObstacleChecks;
    stats.totalObstaclePossible = stats.segmentObstaclePossible + stats.pointObstaclePossible;
    if stats.totalObstaclePossible > 0
        stats.checkFraction = stats.totalObstacleChecks / stats.totalObstaclePossible;
    else
        stats.checkFraction = nan;
    end
end

function exactMask = makeHybridExactMask(validLineMask, clearanceProbe, params)
    exactMask = false(size(validLineMask));
    idxValid = find(validLineMask);
    if isempty(idxValid)
        return;
    end

    triggerThreshold = params.hybrid.triggerFactor * params.dMin;
    exactMask = exactMask | (validLineMask & clearanceProbe < triggerThreshold);

    if params.hybrid.refineActiveTopK
        idxFinite = idxValid(isfinite(clearanceProbe(idxValid)));
        if ~isempty(idxFinite)
            [~, order] = sort(clearanceProbe(idxFinite), 'ascend');
            K = min(params.activeTopK, numel(order));
            exactMask(idxFinite(order(1:K))) = true;
        end
    end

    stride = params.hybrid.forceExactStride;
    if isfinite(stride) && stride > 0
        stride = max(1, round(stride));
        exactMask(idxValid(1:stride:end)) = true;
    end
end
