function bounds = computeObstacleBoundingCircles2D(obstacles)
%COMPUTEOBSTACLEBOUNDINGCIRCLES2D Conservative bounding circles for obstacles.
%
% The returned circles are used as cheap lower bounds in broad-phase
% obstacle filtering. Each circle must contain its obstacle.

    n = numel(obstacles);
    centers = nan(n, 2);
    radii = nan(n, 1);

    for k = 1:n
        obs = obstacles(k);
        type = lower(obs.type);

        switch type
            case 'circle'
                centers(k,:) = obs.center(:).';
                radii(k) = obs.radius;

            case {'rect', 'rectangle', 'box'}
                centers(k,:) = obs.center(:).';
                if isfield(obs, 'vertices') && ~isempty(obs.vertices)
                    radii(k) = max(vecnorm(obs.vertices - centers(k,:), 2, 2));
                else
                    radii(k) = norm(obs.halfSize);
                end

            case 'polygon'
                if isfield(obs, 'center') && all(isfinite(obs.center))
                    centers(k,:) = obs.center(:).';
                else
                    centers(k,:) = mean(obs.vertices, 1);
                end
                radii(k) = max(vecnorm(obs.vertices - centers(k,:), 2, 2));

            otherwise
                if isfield(obs, 'vertices') && ~isempty(obs.vertices)
                    centers(k,:) = mean(obs.vertices, 1);
                    radii(k) = max(vecnorm(obs.vertices - centers(k,:), 2, 2));
                elseif isfield(obs, 'center') && all(isfinite(obs.center))
                    centers(k,:) = obs.center(:).';
                    radii(k) = 0;
                else
                    centers(k,:) = [0, 0];
                    radii(k) = inf;
                end
        end
    end

    bounds = struct();
    bounds.centers = centers;
    bounds.radii = radii;
    bounds.numObstacles = n;
end
