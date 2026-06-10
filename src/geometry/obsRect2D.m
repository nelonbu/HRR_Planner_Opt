function obs = obsRect2D(center, halfSize, yaw)
%OBSRECT2D Create a rotated rectangle obstacle structure.
% halfSize = [halfWidth, halfHeight], yaw in radians.
    if nargin < 3
        yaw = 0;
    end
    obs = struct();
    obs.type = 'rect';
    obs.center = center(:).';
    obs.radius = nan;
    obs.halfSize = halfSize(:).';
    obs.yaw = yaw;
end
