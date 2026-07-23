function tf = isSegmentCollisionFree2D(a, b, obstacles, opts)
%ISSEGMENTCOLLISIONFREE2D Sampled point-robot segment collision check.
%
% Required opts fields are collisionResolution, inflateRadius, and bounds.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'collisionResolution'); opts.collisionResolution = 0.004; end
    if ~isfield(opts, 'inflateRadius'); opts.inflateRadius = 0.0; end
    if ~isfield(opts, 'bounds'); opts.bounds = [0, 1; -0.4, 0.4]; end

    a = a(:).';
    b = b(:).';
    segLen = norm(b - a);
    nSample = max(2, ceil(segLen / opts.collisionResolution) + 1);

    for k = 1:nSample
        t = (k - 1) / (nSample - 1);
        q = (1 - t) * a + t * b;

        if ~isPointInsideBounds(q, opts.bounds) || ...
           ~isPointCollisionFree2D(q, obstacles, opts.inflateRadius)
            tf = false;
            return;
        end
    end

    tf = true;
end

function tf = isPointInsideBounds(q, bounds)
    tf = q(1) >= bounds(1,1) && q(1) <= bounds(1,2) && ...
         q(2) >= bounds(2,1) && q(2) <= bounds(2,2);
end
