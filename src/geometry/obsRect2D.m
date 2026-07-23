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
    obs.vertices = rectVertices(center(:).', halfSize(:).', yaw);
end

function V = rectVertices(center, halfSize, yaw)
    local = [
        -halfSize(1), -halfSize(2)
         halfSize(1), -halfSize(2)
         halfSize(1),  halfSize(2)
        -halfSize(1),  halfSize(2)
    ];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    V = local * R.' + center;
end
