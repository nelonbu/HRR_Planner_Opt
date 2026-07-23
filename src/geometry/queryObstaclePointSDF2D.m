function [d, grad, info] = queryObstaclePointSDF2D(q, obstacles)
%QUERYOBSTACLEPOINTSDF2D Signed distance and gradient to nearest obstacle.
%
% Inputs:
%   q         : 1-by-2 query point
%   obstacles : struct array. Supported types: circle, rect, polygon
%
% Outputs:
%   d     : signed distance to nearest obstacle boundary. Positive outside.
%   grad  : d(d)/d(q), a 1-by-2 row vector, using a valid subgradient.
%   info  : nearest obstacle metadata.

    q = q(:).';

    bestD = inf;
    bestGrad = [1, 0];
    bestInfo = struct('id', nan, 'type', '', 'rawDistance', nan);

    for k = 1:numel(obstacles)
        obs = obstacles(k);

        switch lower(obs.type)
            case 'circle'
                [dk, gk] = sdfCircle2D(q, obs.center, obs.radius);
            case 'rect'
                [dk, gk] = sdfRect2D(q, obs.center, obs.halfSize, obs.yaw);
            case 'polygon'
                [dk, gk] = sdfPolygon2D(q, obs.vertices);
            otherwise
                error('Unsupported obstacle type: %s', obs.type);
        end

        if dk < bestD
            bestD = dk;
            bestGrad = gk;
            bestInfo.id = k;
            bestInfo.type = obs.type;
            bestInfo.rawDistance = dk;
        end
    end

    d = bestD;
    grad = bestGrad;
    info = bestInfo;
end

function [d, g] = sdfCircle2D(q, center, radius)
    diff = q - center;
    dist = norm(diff);
    d = dist - radius;

    if dist < 1e-12
        g = [1, 0];
    else
        g = diff / dist;
    end
end

function [d, gWorld] = sdfRect2D(q, center, halfSize, yaw)
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    qLocal = (R' * (q - center).').';

    a = abs(qLocal) - halfSize;
    outside = max(a, 0);
    outsideDist = norm(outside);
    insideDist = min(max(a(1), a(2)), 0);
    d = outsideDist + insideDist;

    if outsideDist > 1e-12
        gLocal = sign(qLocal) .* outside / outsideDist;
    else
        % Inside or on rectangle: subgradient to nearest face.
        if a(1) > a(2)
            gLocal = [signNonzero(qLocal(1)), 0];
        else
            gLocal = [0, signNonzero(qLocal(2))];
        end
    end

    gWorld = (R * gLocal.').';
end

function s = signNonzero(x)
    if x >= 0
        s = 1;
    else
        s = -1;
    end
end

function [d, g] = sdfPolygon2D(q, vertices)
    n = size(vertices, 1);
    inside = inpolygon(q(1), q(2), vertices(:,1), vertices(:,2));

    bestDist = inf;
    bestClosest = vertices(1,:);

    for i = 1:n
        a = vertices(i,:);
        b = vertices(mod(i, n) + 1,:);
        e = b - a;
        len2 = dot(e, e);
        if len2 < 1e-14
            t = 0;
        else
            t = max(0, min(1, dot(q - a, e) / len2));
        end
        c = a + t * e;
        dist = norm(q - c);
        if dist < bestDist
            bestDist = dist;
            bestClosest = c;
        end
    end

    if bestDist < 1e-12
        dir = q - mean(vertices, 1);
        if norm(dir) < 1e-12
            dir = [1, 0];
        end
        g = dir / norm(dir);
        d = 0;
    elseif inside
        d = -bestDist;
        g = (bestClosest - q) / bestDist;
    else
        d = bestDist;
        g = (q - bestClosest) / bestDist;
    end
end
