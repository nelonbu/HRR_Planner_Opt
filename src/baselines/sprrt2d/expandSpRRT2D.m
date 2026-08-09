function [tree, newIdx, stats] = ...
        expandSpRRT2D(tree, qRand, obstacles, opts)
%EXPANDSPRRT2D Try nearest candidates for one adaptive Sp-RRT sample.
%
% Unlike ordinary RRT, a rejected nearest node is abandoned only for the
% current qRand and the next-nearest tree node is then attempted.

    stats = emptyStats();
    stats.expandCallCount = 1;
    newIdx = 0;
    qRand = qRand(:).';

    distanceSq = sum((tree.position - qRand).^2, 2);
    [~, order] = sort(distanceSq, 'ascend');

    for k = 1:numel(order)
        nearIdx = order(k);
        stats.candidateAttemptCount = stats.candidateAttemptCount + 1;

        if tree.depth(nearIdx) >= opts.maxSegmentCount
            stats = reject(stats, 'depth');
            continue;
        end

        direction = qRand - tree.position(nearIdx,:);
        directionNorm = norm(direction);
        if directionNorm <= opts.duplicateTolerance
            stats = reject(stats, 'duplicate');
            continue;
        end

        edgeLength = opts.L;
        qNew = tree.position(nearIdx,:) + ...
            edgeLength * direction / directionNorm;

        if any(vecnorm(tree.position - qNew, 2, 2) <= ...
                opts.duplicateTolerance)
            stats = reject(stats, 'duplicate');
            continue;
        end

        [accepted, detail] = connectSpRRT2D( ...
            tree, nearIdx, qNew, edgeLength, false, obstacles, opts);
        if ~isnan(detail.theta)
            stats.maxGeneratedTurn = max(stats.maxGeneratedTurn, detail.theta);
        end
        if ~accepted
            stats = reject(stats, detail.reason);
            continue;
        end

        tree.position(end+1,:) = qNew;
        tree.parent(end+1,1) = nearIdx;
        tree.depth(end+1,1) = detail.newDepth;
        tree.cost(end+1,1) = tree.cost(nearIdx) + detail.edgeLength;
        newIdx = size(tree.position, 1);
        stats.expandSuccessCount = 1;
        return;
    end

    stats.expandFailureCount = 1;
end

function stats = reject(stats, reason)
    stats.abandonedCandidateCount = stats.abandonedCandidateCount + 1;
    switch lower(reason)
        case 'angle'
            stats.angleRejectCount = stats.angleRejectCount + 1;
        case 'collision'
            stats.collisionRejectCount = stats.collisionRejectCount + 1;
        case 'bounds'
            stats.boundsRejectCount = stats.boundsRejectCount + 1;
        case 'length'
            stats.lengthRejectCount = stats.lengthRejectCount + 1;
        case 'depth'
            stats.depthRejectCount = stats.depthRejectCount + 1;
        case 'duplicate'
            stats.duplicateRejectCount = stats.duplicateRejectCount + 1;
    end
end

function stats = emptyStats()
    stats = struct();
    stats.expandCallCount = 0;
    stats.expandSuccessCount = 0;
    stats.expandFailureCount = 0;
    stats.candidateAttemptCount = 0;
    stats.abandonedCandidateCount = 0;
    stats.angleRejectCount = 0;
    stats.collisionRejectCount = 0;
    stats.boundsRejectCount = 0;
    stats.lengthRejectCount = 0;
    stats.depthRejectCount = 0;
    stats.duplicateRejectCount = 0;
    stats.maxGeneratedTurn = 0;
end
