function [obstacles, sdfMap] = buildMapAndSDF2D(mapConfig, p_start, p_goal)
% buildMapAndSDF2D
% Random circle obstacle generator + analytic circle SDF interface.
%
% obstacles format:
%   obstacles = [cx, cy, radius]
%
% Required:
%   mapConfig.xRange = [xmin, xmax]
%   mapConfig.yRange = [ymin, ymax]
%
% Optional:
%   mapConfig.numObstacles          default: 5
%   mapConfig.radiusRange           default: [50, 90] mm
%   mapConfig.startGoalSafeRadius   default: 180 mm
%   mapConfig.minObstacleGap        default: 20 mm
%   mapConfig.seed                  default: 1
%   mapConfig.maxSampleAttempts     default: 5000

    %% ---------- Default parameters ----------
    if ~isfield(mapConfig, 'numObstacles')
        mapConfig.numObstacles = 5;
    end

    if ~isfield(mapConfig, 'radiusRange')
        mapConfig.radiusRange = [50, 90];
    end

    if ~isfield(mapConfig, 'startGoalSafeRadius')
        mapConfig.startGoalSafeRadius = 180;
    end

    if ~isfield(mapConfig, 'minObstacleGap')
        mapConfig.minObstacleGap = 20;
    end

    if ~isfield(mapConfig, 'seed')
        mapConfig.seed = 1;
    end

    if ~isfield(mapConfig, 'maxSampleAttempts')
        mapConfig.maxSampleAttempts = 5000;
    end

    rng(mapConfig.seed);

    xmin = mapConfig.xRange(1);
    xmax = mapConfig.xRange(2);
    ymin = mapConfig.yRange(1);
    ymax = mapConfig.yRange(2);

    rMin = mapConfig.radiusRange(1);
    rMax = mapConfig.radiusRange(2);

    numObs = mapConfig.numObstacles;

    obstacles = zeros(numObs, 3);
    obsCount = 0;
    attempt = 0;

    %% ---------- Random obstacle generation ----------
    while obsCount < numObs && attempt < mapConfig.maxSampleAttempts
        attempt = attempt + 1;

        % Random radius
        r = rMin + rand() * (rMax - rMin);

        % Ensure circle is fully inside map boundary
        cx = (xmin + r) + rand() * ((xmax - r) - (xmin + r));
        cy = (ymin + r) + rand() * ((ymax - r) - (ymin + r));

        candidate = [cx, cy, r];

        % Reject if too close to start or goal
        if isCircleTooCloseToPoint(candidate, p_start, mapConfig.startGoalSafeRadius)
            continue;
        end

        if isCircleTooCloseToPoint(candidate, p_goal, mapConfig.startGoalSafeRadius)
            continue;
        end

        % Reject if overlapping or too close to existing obstacles
        if isCircleTooCloseToExisting(candidate, obstacles(1:obsCount, :), mapConfig.minObstacleGap)
            continue;
        end

        obsCount = obsCount + 1;
        obstacles(obsCount, :) = candidate;
    end

    obstacles = obstacles(1:obsCount, :);

    if obsCount < numObs
        warning('Only generated %d / %d obstacles. Try reducing safe radius, obstacle size, or obstacle count.', ...
            obsCount, numObs);
    end

    %% ---------- Final safety check ----------
    if isPointInsideAnyCircle(p_start, obstacles)
        error('Start point is inside an obstacle.');
    end

    if isPointInsideAnyCircle(p_goal, obstacles)
        error('Goal point is inside an obstacle.');
    end

    %% ---------- SDF map interface ----------
    sdfMap.type = 'circle_analytic';
    sdfMap.obstacles = obstacles;
    sdfMap.xRange = mapConfig.xRange;
    sdfMap.yRange = mapConfig.yRange;
end


function tooClose = isCircleTooCloseToPoint(circle, p, safeRadius)
    c = circle(1:2);
    r = circle(3);

    d = norm(c - p);

    % Keep obstacle boundary at least safeRadius away from point
    tooClose = d <= r + safeRadius;
end


function tooClose = isCircleTooCloseToExisting(candidate, obstacles, minGap)
    if isempty(obstacles)
        tooClose = false;
        return;
    end

    c = candidate(1:2);
    r = candidate(3);

    centers = obstacles(:, 1:2);
    radii = obstacles(:, 3);

    d = sqrt(sum((centers - c).^2, 2));

    % Require distance between circle boundaries >= minGap
    tooClose = any(d <= radii + r + minGap);
end


function inside = isPointInsideAnyCircle(p, obstacles)
    if isempty(obstacles)
        inside = false;
        return;
    end

    centers = obstacles(:, 1:2);
    radii = obstacles(:, 3);

    d = sqrt(sum((p - centers).^2, 2));
    inside = any(d <= radii);
end