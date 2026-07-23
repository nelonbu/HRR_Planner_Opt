function tf = isPointCollisionFree2D(q, obstacles, inflateRadius)
%ISPOINTCOLLISIONFREE2D Point collision check using obstacle SDF.
%
% inflateRadius enlarges obstacles by requiring SDF(q) > inflateRadius.

    if nargin < 3 || isempty(inflateRadius)
        inflateRadius = 0.0;
    end

    if isempty(obstacles)
        tf = true;
        return;
    end

    d = queryObstaclePointSDF2D(q, obstacles);
    tf = d > inflateRadius;
end
