function obs = obsCircle2D(center, radius)
%OBSCIRCLE2D Create a circle obstacle structure.
    obs = struct();
    obs.type = 'circle';
    obs.center = center(:).';
    obs.radius = radius;
    obs.halfSize = [nan, nan];
    obs.yaw = 0;
    obs.vertices = zeros(0, 2);
end
