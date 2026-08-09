function [accepted, detail] = connectSpRRT2D( ...
        tree, nearIdx, q2, edgeLength, allowShortEdge, obstacles, opts)
%CONNECTSPRRT2D Check one adaptive Sp-RRT path segment.
%
% tree.depth(root)=0. A node at depth d has d leader-path segments between
% itself and the goal root. Every segment uses length L and thetaMax. Depth
% is not a physical robot-link index.

    accepted = false;
    detail = struct('reason', 'unknown', 'theta', nan, ...
        'newDepth', nan, 'linkIndex', nan, 'edgeLength', nan);

    q1 = tree.position(nearIdx,:);
    q2 = q2(:).';
    newDepth = tree.depth(nearIdx) + 1;
    detail.newDepth = newDepth;
    detail.linkIndex = newDepth;
    detail.edgeLength = norm(q2 - q1);

    if newDepth > opts.maxSegmentCount
        detail.reason = 'depth';
        return;
    end
    if ~insideBounds(q2, opts.bounds)
        detail.reason = 'bounds';
        return;
    end

    if allowShortEdge
        lengthOK = detail.edgeLength <= edgeLength + opts.lengthTolerance;
    else
        lengthOK = abs(detail.edgeLength - edgeLength) <= ...
            opts.lengthTolerance;
    end
    if ~lengthOK || detail.edgeLength <= opts.duplicateTolerance
        detail.reason = 'length';
        return;
    end

    if tree.depth(nearIdx) >= 1
        parentIdx = tree.parent(nearIdx);
        vIn = q1 - tree.position(parentIdx,:);
        vOut = q2 - q1;
        detail.theta = turnAngle(vIn, vOut);
        if ~isfinite(detail.theta) || ...
                detail.theta > opts.thetaMax + opts.angleTolerance
            detail.reason = 'angle';
            return;
        end
    end

    collisionOpts = struct( ...
        'bounds', opts.bounds, ...
        'collisionResolution', opts.collisionResolution, ...
        'inflateRadius', opts.inflateRadius);
    if ~isSegmentCollisionFree2D(q1, q2, obstacles, collisionOpts)
        detail.reason = 'collision';
        return;
    end

    accepted = true;
    detail.reason = 'accepted';
end

function theta = turnAngle(vIn, vOut)
    denom = norm(vIn) * norm(vOut);
    if denom <= eps
        theta = nan;
        return;
    end
    cosine = dot(vIn, vOut) / denom;
    cosine = max(-1, min(1, cosine));
    theta = acos(cosine);
end

function tf = insideBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
        q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end
