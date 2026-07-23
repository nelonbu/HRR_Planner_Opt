function obs = obsPolygon2D(vertices)
%OBSPOLYGON2D Create a polygon obstacle structure.
% vertices is n-by-2 and should describe a simple closed polygon without
% repeating the first vertex. Rectangles may also use obsRect2D.

    if size(vertices, 2) ~= 2 || size(vertices, 1) < 3
        error('obsPolygon2D requires an n-by-2 vertex array with n >= 3.');
    end

    obs = struct();
    obs.type = 'polygon';
    obs.center = mean(vertices, 1);
    obs.radius = nan;
    obs.halfSize = [nan, nan];
    obs.yaw = 0;
    obs.vertices = vertices;
end
