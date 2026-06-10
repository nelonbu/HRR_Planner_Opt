function out = querySegmentObstacleClearance2D(M, N, obstacles)
%QUERYSEGMENTOBSTACLECLEARANCE2D Fast clearance from a line segment to obstacles.
%
% Supported obstacle types:
%   circle : analytic segment-circle distance
%   rect   : fast candidate-based segment-rotated-rectangle SDF minimum
%
% Output:
%   out.clearance     : obstacle signed distance
%   out.closestPoint  : q = M + alpha*(N-M)
%   out.alpha         : closest alpha on segment
%   out.normal        : d(clearance)/d(q)
%   out.obsId         : nearest obstacle index
%   out.obsType       : nearest obstacle type
%   out.rawDistance   : same signed distance as clearance

    M = M(:).';
    N = N(:).';
    e = N - M;
    len2 = dot(e, e);

    bestClearance = inf;
    bestQ = M;
    bestAlpha = 0;
    bestNormal = [1, 0];
    bestObsId = nan;
    bestObsType = '';
    bestRawD = inf;

    for k = 1:numel(obstacles)
        obs = obstacles(k);
        type = lower(obs.type);

        switch type
            case 'circle'
                [rawD, q, alpha, normal] = segmentCircleFast(M, e, len2, obs);

            case {'rect', 'rectangle', 'box'}
                [rawD, q, alpha, normal] = segmentRectFast(M, e, len2, obs);

            otherwise
                % Fast generic fallback: no fminbnd.
                [rawD, q, alpha, normal] = segmentSDFSampleFast(M, e, len2, obs);
        end

        clearance = rawD;

        if clearance < bestClearance
            bestClearance = clearance;
            bestQ = q;
            bestAlpha = alpha;
            bestNormal = normal;
            bestObsId = k;
            bestObsType = obs.type;
            bestRawD = rawD;
        end
    end

    out = struct();
    out.clearance = bestClearance;
    out.closestPoint = bestQ;
    out.alpha = bestAlpha;
    out.normal = bestNormal;
    out.obsId = bestObsId;
    out.obsType = bestObsType;
    out.rawDistance = bestRawD;
end

%% ============================================================
% Circle: analytic
%% ============================================================

function [rawD, q, alpha, normal] = segmentCircleFast(M, e, len2, obs)
    c = obs.center(:).';

    if len2 < 1e-14
        alpha = 0;
        q = M;
    else
        alpha = dot(c - M, e) / len2;
        alpha = clamp01(alpha);
        q = M + alpha * e;
    end

    diff = q - c;
    dist = norm(diff);

    rawD = dist - obs.radius;

    if dist < 1e-12
        normal = [1, 0];
    else
        normal = diff / dist;
    end
end

%% ============================================================
% Rotated rectangle: fast candidate-based SDF minimum
%% ============================================================

function [rawD, qWorld, alphaBest, normalWorld] = segmentRectFast(M, e, len2, obs)
    c = obs.center(:).';
    h = obs.halfSize(:).';

    if isfield(obs, 'yaw')
        yaw = obs.yaw;
    else
        yaw = 0;
    end

    cy = cos(yaw);
    sy = sin(yaw);

    R = [cy, -sy;
         sy,  cy];

    % World -> local
    ML = (R' * (M - c).').';
    eL = (R' * e.').';

    if len2 < 1e-14
        alphaBest = 0;
        qLocal = ML;
        [rawD, normalLocal] = rectSDFLocal(qLocal, h);
        qWorld = M;
        normalWorld = (R * normalLocal.').';
        return;
    end

    alphaList = rectCandidateAlphas(ML, eL, h);
    alphaList = alphaList(alphaList >= 0 & alphaList <= 1 & isfinite(alphaList));
    alphaList = unique(round(alphaList * 1e12) / 1e12);

    bestD = inf;
    alphaBest = 0;
    normalLocalBest = [1, 0];

    for i = 1:numel(alphaList)
        a = alphaList(i);
        qLocal = ML + a * eL;

        [d, nLocal] = rectSDFLocal(qLocal, h);

        if d < bestD
            bestD = d;
            alphaBest = a;
            normalLocalBest = nLocal;
        end
    end

    rawD = bestD;
    qWorld = M + alphaBest * e;
    normalWorld = (R * normalLocalBest.').';

    nrm = norm(normalWorld);
    if nrm < 1e-12
        normalWorld = [1, 0];
    else
        normalWorld = normalWorld / nrm;
    end
end

function alphaList = rectCandidateAlphas(M, e, h)
    % Candidate set for minimizing rectangle SDF along a segment.
    %
    % Includes:
    %   endpoints
    %   closest-to-center projection
    %   intersections with x = 0, y = 0
    %   intersections with rectangle side lines
    %   projections to four corners
    %   inside-SDF equality lines:
    %       s1*x - hx = s2*y - hy

    alphaList = zeros(1, 32);
    cnt = 0;

    cnt = cnt + 1; alphaList(cnt) = 0;
    cnt = cnt + 1; alphaList(cnt) = 1;

    len2 = dot(e, e);

    if len2 > 1e-14
        % Projection to local rectangle center
        cnt = cnt + 1;
        alphaList(cnt) = -dot(M, e) / len2;
    end

    % x = 0, y = 0
    if abs(e(1)) > 1e-14
        cnt = cnt + 1; alphaList(cnt) = -M(1) / e(1);
    end

    if abs(e(2)) > 1e-14
        cnt = cnt + 1; alphaList(cnt) = -M(2) / e(2);
    end

    % Side lines x = +/- hx, y = +/- hy
    sx = [-1, 1];
    sy = [-1, 1];

    for a = sx
        if abs(e(1)) > 1e-14
            cnt = cnt + 1;
            alphaList(cnt) = (a*h(1) - M(1)) / e(1);
        end
    end

    for b = sy
        if abs(e(2)) > 1e-14
            cnt = cnt + 1;
            alphaList(cnt) = (b*h(2) - M(2)) / e(2);
        end
    end

    % Corner projections
    corners = [
        -h(1), -h(2);
         h(1), -h(2);
         h(1),  h(2);
        -h(1),  h(2)
    ];

    if len2 > 1e-14
        for i = 1:4
            corner = corners(i,:);
            cnt = cnt + 1;
            alphaList(cnt) = dot(corner - M, e) / len2;
        end
    end

    % Equality lines for inside SDF:
    % |x|-hx = |y|-hy
    % s1*x - hx = s2*y - hy
    for s1 = [-1, 1]
        for s2 = [-1, 1]
            denom = s1*e(1) - s2*e(2);
            numer = h(1) - h(2) - s1*M(1) + s2*M(2);

            if abs(denom) > 1e-14
                cnt = cnt + 1;
                alphaList(cnt) = numer / denom;
            end
        end
    end

    alphaList = alphaList(1:cnt);

    % Add tiny neighborhood around candidates for robustness.
    delta = 1e-4;
    alphaList = [alphaList, alphaList - delta, alphaList + delta]; %#ok<AGROW>

    alphaList = max(0, min(1, alphaList));
end

function [d, g] = rectSDFLocal(q, h)
    % Signed distance from point q to axis-aligned rectangle centered at zero.
    % q, h are 1-by-2.

    a = abs(q) - h;
    outside = max(a, 0);
    outsideDist = norm(outside);

    if outsideDist > 1e-12
        d = outsideDist;
        g = signNonzero(q) .* outside / outsideDist;
    else
        % Inside or on boundary.
        % SDF = min(max(a_x, a_y), 0)
        [m, id] = max(a);
        d = min(m, 0);

        if id == 1
            g = [signScalar(q(1)), 0];
        else
            g = [0, signScalar(q(2))];
        end
    end
end

%% ============================================================
% Generic fallback for unknown SDF obstacles
%% ============================================================

function [rawD, qBest, alphaBest, normalBest] = segmentSDFSampleFast(M, e, len2, obs)
    if len2 < 1e-14
        alphaBest = 0;
        qBest = M;
        [rawD, normalBest] = queryObstaclePointSDF2D(qBest, obs);
        return;
    end

    % Very cheap fixed samples. Increase to 17 if accuracy is insufficient.
    aList = [0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1];

    bestD = inf;
    alphaBest = 0;
    normalBest = [1, 0];
    qBest = M;

    for i = 1:numel(aList)
        a = aList(i);
        q = M + a * e;

        [d, n] = queryObstaclePointSDF2D(q, obs);

        if d < bestD
            bestD = d;
            alphaBest = a;
            qBest = q;
            normalBest = n;
        end
    end

    rawD = bestD;
end

%% ============================================================
% Helpers
%% ============================================================

function y = clamp01(x)
    y = min(max(x, 0), 1);
end

function s = signScalar(x)
    if x >= 0
        s = 1;
    else
        s = -1;
    end
end

function s = signNonzero(x)
    s = ones(size(x));
    s(x < 0) = -1;
end
