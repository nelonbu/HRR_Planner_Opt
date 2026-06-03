clear; clc; close all;

%% ============================================================
%  2D path point optimization with length-based point pairs
%  Units: mm
%
%  Main features:
%  1. Initial path is straight line from start to goal
%  2. Point pairs are constructed by target length L_pair = 150 mm
%  3. Outer loop updates pairs, inner loop optimizes with fixed pairs
%  4. Pair segments must avoid obstacles
%  5. Adjacent point distance has lower and upper bounds
%  6. Turning angle constraint prevents sharp corners
%  7. True three-point curvature constraint prevents high curvature
%  8. Live visualization updates during optimization
% ============================================================

%% ===================== 1. Basic settings =====================
p_start = [0, 0];
p_goal  = [1000, 0];        % total path length around 1 m = 1000 mm

N = 80;                     % number of path points
L_pair = 150;               % target pair length, mm

alpha = linspace(0, 1, N)';
P0 = (1 - alpha) .* p_start + alpha .* p_goal;
P = P0;

% Add small initial perturbation to avoid symmetry trap
dir = p_goal - p_start;
dir = dir / norm(dir);
normal = [-dir(2), dir(1)];

perturbAmp = 15;            % mm
P(2:end-1, :) = P(2:end-1, :) + perturbAmp * sin(pi * alpha(2:end-1)) .* normal;

% Circle obstacles: [cx, cy, radius], units: mm
obstacles = [
    310,   0, 75;
    515, -100, 70;
    515, 100, 70;
    720,  25, 65
];

%% ===================== 2. Optimization parameters =====================
params.L_pair = L_pair;

% Collision / clearance
params.robotRadius = 5;     % line segment tube radius, mm
params.d_min       = 20;    % minimum clearance, mm
params.d_pref      = 60;    % preferred clearance, mm

% Cost weights
params.wObs        = 1200.0;
params.wClear      = 20.0;
params.wRef        = 0.02;
params.wLen        = 1.0;
params.wSmooth     = 50.0;
params.wPairLen    = 0.05;
params.wTotalLen   = 0.02;

% Total path length target
params.targetPathLength = 1000;       % mm

% Adjacent point distance bounds
params.avgInitSpacing = computePathLength(P0) / (N - 1);
params.minAdjacentDist = 0.65 * params.avgInitSpacing;
params.maxAdjacentDist = 2.00 * params.avgInitSpacing;

params.wAdjMin = 2000.0;
params.wAdjMax = 1200.0;

% Turning angle constraint
params.thetaMax = deg2rad(30);        % max turning angle, rad
params.wAngle   = 8.0e4;

% True three-point curvature constraint
params.kappaMaxTrue = 1 / 120;        % 1/mm, equivalent min radius = 120 mm
params.wCurvTrue    = 8.0e5;

% Numerical gradient for angle + true curvature penalties
% This is slower but robust and easier to verify.
params.useNumericalAngleCurvPenalty = true;
params.epsFD = 1e-3;                  % finite difference step, mm

% Sampling on every pair segment
params.samplePerSeg = 16;

% Trust region
params.rTrust = 180;                  % mm

% Outer-inner optimization
params.numOuter = 6;
params.innerIter = 400;

% Adam
params.lr       = 0.035;
params.beta1    = 0.9;
params.beta2    = 0.999;
params.epsAdam  = 1e-8;
params.gradClip = 500.0;

% Live visualization
params.enableLivePlot = true;
params.visualUpdateInterval = 40;

%% ===================== 3. Initialize visualization =====================
pairs = constructPairsByLength(P, L_pair);

if params.enableLivePlot
    hVis = initLiveVisualization(P0, P, pairs, obstacles, params);
end

%% ===================== 4. Outer-loop optimization =====================
totalIter = params.numOuter * params.innerIter;

Jhist = zeros(totalIter, 1);
minClearHist = zeros(totalIter, 1);
lenHist = zeros(totalIter, 1);
maxKappaHist = zeros(totalIter, 1);
maxAngleHist = zeros(totalIter, 1);
minAdjDistHist = zeros(totalIter, 1);
maxAdjDistHist = zeros(totalIter, 1);

globalIter = 0;

for outer = 1:params.numOuter
    % Reconstruct pairs using current optimized path
    pairs = constructPairsByLength(P, L_pair);

    fprintf('\nOuter %d / %d | pair number = %d\n', ...
        outer, params.numOuter, size(pairs, 1));

    % Reset Adam moments for each fixed-pair stage
    M = zeros(size(P));
    V = zeros(size(P));

    for iter = 1:params.innerIter
        globalIter = globalIter + 1;

        [J, G] = objectiveAndGradient(P, P0, pairs, obstacles, params);

        % Fix start and goal
        G(1, :) = 0;
        G(end, :) = 0;

        % Gradient clipping
        gnorm = norm(G(:));
        if gnorm > params.gradClip
            G = G * params.gradClip / (gnorm + 1e-12);
        end

        % Adam update
        M = params.beta1 * M + (1 - params.beta1) * G;
        V = params.beta2 * V + (1 - params.beta2) * (G.^2);

        Mhat = M / (1 - params.beta1^iter);
        Vhat = V / (1 - params.beta2^iter);

        P = P - params.lr * Mhat ./ (sqrt(Vhat) + params.epsAdam);

        % Fix endpoints
        P(1, :) = p_start;
        P(end, :) = p_goal;

        % Project to trust region
        P = projectToTrustRegion(P, P0, params.rTrust);

        % Record
        Jhist(globalIter) = J;
        minClearHist(globalIter) = evaluateMinClearance(P, pairs, obstacles, params.robotRadius, 50);
        lenHist(globalIter) = computePathLength(P);
        maxKappaHist(globalIter) = evaluateMaxTrueCurvature(P);
        maxAngleHist(globalIter) = evaluateMaxTurningAngle(P);
        minAdjDistHist(globalIter) = evaluateMinAdjacentDistance(P);
        maxAdjDistHist(globalIter) = evaluateMaxAdjacentDistance(P);

        % Live update
        if params.enableLivePlot && ...
           (mod(globalIter, params.visualUpdateInterval) == 0 || globalIter == 1)

            updateLiveVisualization( ...
                hVis, P, pairs, params, globalIter, ...
                Jhist(globalIter), ...
                minClearHist(globalIter), ...
                lenHist(globalIter), ...
                maxKappaHist(globalIter), ...
                maxAngleHist(globalIter), ...
                minAdjDistHist(globalIter), ...
                maxAdjDistHist(globalIter));
        end

        if mod(iter, 100) == 0
            fprintf(['  iter %4d | J = %.3f | minClear = %.2f mm | ', ...
                     'length = %.2f mm | maxKappa = %.5f 1/mm | ', ...
                     'maxAngle = %.2f deg | minAdj = %.2f mm | maxAdj = %.2f mm\n'], ...
                iter, J, minClearHist(globalIter), lenHist(globalIter), ...
                maxKappaHist(globalIter), rad2deg(maxAngleHist(globalIter)), ...
                minAdjDistHist(globalIter), maxAdjDistHist(globalIter));
        end
    end
end

Popt = P;
pairsFinal = constructPairsByLength(Popt, L_pair);

%% ===================== 5. Final check =====================
finalMinClear = evaluateMinClearance(Popt, pairsFinal, obstacles, params.robotRadius, 100);
finalLength = computePathLength(Popt);
finalMaxKappa = evaluateMaxTrueCurvature(Popt);
finalMaxAngle = evaluateMaxTurningAngle(Popt);
finalMinAdjDist = evaluateMinAdjacentDistance(Popt);
finalMaxAdjDist = evaluateMaxAdjacentDistance(Popt);

fprintf('\n================ Final result ================\n');
fprintf('Final minimum clearance: %.3f mm\n', finalMinClear);
fprintf('Required minimum clearance: %.3f mm\n', params.d_min);

fprintf('Final path length: %.3f mm\n', finalLength);
fprintf('Target path length: %.3f mm\n', params.targetPathLength);

fprintf('Final max true curvature: %.6f 1/mm\n', finalMaxKappa);
fprintf('Allowed max true curvature: %.6f 1/mm\n', params.kappaMaxTrue);
fprintf('Equivalent min radius: %.2f mm\n', 1 / params.kappaMaxTrue);

fprintf('Final max turning angle: %.3f deg\n', rad2deg(finalMaxAngle));
fprintf('Allowed max turning angle: %.3f deg\n', rad2deg(params.thetaMax));

fprintf('Final min adjacent distance: %.3f mm\n', finalMinAdjDist);
fprintf('Allowed min adjacent distance: %.3f mm\n', params.minAdjacentDist);

fprintf('Final max adjacent distance: %.3f mm\n', finalMaxAdjDist);
fprintf('Allowed max adjacent distance: %.3f mm\n', params.maxAdjacentDist);

fprintf('Final pair number: %d\n', size(pairsFinal, 1));

if finalMinClear >= params.d_min
    fprintf('Collision check: PASS under sampled checking.\n');
else
    fprintf('Collision check: WARNING, clearance violation exists.\n');
end

if finalMaxKappa <= params.kappaMaxTrue
    fprintf('Curvature check: PASS.\n');
else
    fprintf('Curvature check: WARNING, curvature is too high.\n');
end

if finalMaxAngle <= params.thetaMax
    fprintf('Turning angle check: PASS.\n');
else
    fprintf('Turning angle check: WARNING, turning angle is too large.\n');
end

if finalMinAdjDist >= params.minAdjacentDist
    fprintf('Adjacent min distance check: PASS.\n');
else
    fprintf('Adjacent min distance check: WARNING, adjacent points collapse.\n');
end

if finalMaxAdjDist <= params.maxAdjacentDist
    fprintf('Adjacent max distance check: PASS.\n');
else
    fprintf('Adjacent max distance check: WARNING, adjacent distance too large.\n');
end

%% ===================== 6. History plots =====================
figure('Name', 'Optimization history');

subplot(7,1,1);
plot(Jhist, 'LineWidth', 1.3);
grid on;
xlabel('Iteration');
ylabel('Objective');

subplot(7,1,2);
plot(minClearHist, 'LineWidth', 1.3);
hold on;
yline(params.d_min, 'r--', 'd_{min}');
grid on;
xlabel('Iteration');
ylabel('Min clearance / mm');

subplot(7,1,3);
plot(lenHist, 'LineWidth', 1.3);
hold on;
yline(params.targetPathLength, 'r--', 'target length');
grid on;
xlabel('Iteration');
ylabel('Path length / mm');

subplot(7,1,4);
plot(maxKappaHist, 'LineWidth', 1.3);
hold on;
yline(params.kappaMaxTrue, 'r--', '\kappa_{max}');
grid on;
xlabel('Iteration');
ylabel('Max curvature / 1/mm');

subplot(7,1,5);
plot(rad2deg(maxAngleHist), 'LineWidth', 1.3);
hold on;
yline(rad2deg(params.thetaMax), 'r--', '\theta_{max}');
grid on;
xlabel('Iteration');
ylabel('Max angle / deg');

subplot(7,1,6);
plot(minAdjDistHist, 'LineWidth', 1.3);
hold on;
yline(params.minAdjacentDist, 'r--', 'd_{adj,min}');
grid on;
xlabel('Iteration');
ylabel('Min adjacent / mm');

subplot(7,1,7);
plot(maxAdjDistHist, 'LineWidth', 1.3);
hold on;
yline(params.maxAdjacentDist, 'r--', 'd_{adj,max}');
grid on;
xlabel('Iteration');
ylabel('Max adjacent / mm');


%% ============================================================
%  Local functions
% ============================================================

function pairs = constructPairsByLength(P, L_pair)
    % For each point p_i, search forward along the path sequence.
    % Find the later point whose Euclidean distance to p_i is closest to L_pair.
    % If no later point reaches L_pair, use the endpoint p_N.

    N = size(P, 1);
    pairs = zeros(N-1, 2);
    cnt = 0;

    for i = 1:N-1
        jBest = N;
        found = false;

        for j = i+1:N
            dCurr = norm(P(j, :) - P(i, :));

            if dCurr >= L_pair
                found = true;

                if j == i + 1
                    jBest = j;
                else
                    dPrev = norm(P(j-1, :) - P(i, :));

                    if abs(dPrev - L_pair) <= abs(dCurr - L_pair)
                        jBest = j - 1;
                    else
                        jBest = j;
                    end
                end

                break;
            end
        end

        if ~found
            jBest = N;
        end

        if jBest > i
            cnt = cnt + 1;
            pairs(cnt, :) = [i, jBest];
        end
    end

    pairs = pairs(1:cnt, :);
end


function [J, G] = objectiveAndGradient(P, P0, pairs, obstacles, params)
    N = size(P, 1);
    G = zeros(size(P));
    J = 0;

    %% ---------- Obstacle and clearance cost on pair segments ----------
    Q = params.samplePerSeg;
    ts = linspace(0, 1, Q + 1);

    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        pi = P(i, :);
        pj = P(j, :);

        for q = 1:numel(ts)
            t = ts(q);
            x = (1 - t) * pi + t * pj;

            [phi, gradPhi] = circleUnionSDF2D(x, obstacles);

            clearance = phi - params.robotRadius;
            dJdx = [0, 0];

            % Hard minimum-clearance penalty
            gapObs = params.d_min - clearance;
            if gapObs > 0
                J = J + params.wObs * gapObs^2;
                dJdx = dJdx - 2 * params.wObs * gapObs * gradPhi;
            end

            % Soft preferred-clearance penalty
            gapClear = params.d_pref - clearance;
            if gapClear > 0
                J = J + params.wClear * gapClear^2;
                dJdx = dJdx - 2 * params.wClear * gapClear * gradPhi;
            end

            % Chain rule: x = (1-t) p_i + t p_j
            G(i, :) = G(i, :) + (1 - t) * dJdx;
            G(j, :) = G(j, :) + t * dJdx;
        end
    end

    %% ---------- Reference cost ----------
    D = P - P0;
    D(1, :) = 0;
    D(end, :) = 0;

    J = J + params.wRef * sum(D(:).^2);
    G = G + 2 * params.wRef * D;

    %% ---------- Adjacent length preservation ----------
    for i = 1:N-1
        d = P(i+1, :) - P(i, :);
        l = norm(d);

        d0 = P0(i+1, :) - P0(i, :);
        l0 = norm(d0);

        err = l - l0;
        J = J + params.wLen * err^2;

        if l > 1e-10
            g = 2 * params.wLen * err * d / l;
            G(i, :)   = G(i, :)   - g;
            G(i+1, :) = G(i+1, :) + g;
        end
    end

    %% ---------- Adjacent distance lower-bound penalty ----------
    for i = 1:N-1
        d = P(i+1, :) - P(i, :);
        l = norm(d);

        gap = params.minAdjacentDist - l;

        if gap > 0
            J = J + params.wAdjMin * gap^2;

            if l > 1e-10
                g = 2 * params.wAdjMin * gap * d / l;

                % gap = d_min - l
                % This pushes p_i and p_{i+1} apart.
                G(i, :)   = G(i, :)   + g;
                G(i+1, :) = G(i+1, :) - g;
            end
        end
    end

    %% ---------- Adjacent distance upper-bound penalty ----------
    for i = 1:N-1
        d = P(i+1, :) - P(i, :);
        l = norm(d);

        gap = l - params.maxAdjacentDist;

        if gap > 0
            J = J + params.wAdjMax * gap^2;

            if l > 1e-10
                g = 2 * params.wAdjMax * gap * d / l;

                % gap = l - d_max
                % This pulls p_i and p_{i+1} closer.
                G(i, :)   = G(i, :)   - g;
                G(i+1, :) = G(i+1, :) + g;
            end
        end
    end

    %% ---------- Smoothness cost ----------
    for i = 2:N-1
        e = P(i+1, :) - 2 * P(i, :) + P(i-1, :);

        J = J + params.wSmooth * sum(e.^2);

        G(i-1, :) = G(i-1, :) + 2 * params.wSmooth * e;
        G(i, :)   = G(i, :)   - 4 * params.wSmooth * e;
        G(i+1, :) = G(i+1, :) + 2 * params.wSmooth * e;
    end

    %% ---------- Pair length cost ----------
    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        d = P(j, :) - P(i, :);
        l = norm(d);

        err = l - params.L_pair;

        J = J + params.wPairLen * err^2;

        if l > 1e-10
            g = 2 * params.wPairLen * err * d / l;
            G(i, :) = G(i, :) - g;
            G(j, :) = G(j, :) + g;
        end
    end

    %% ---------- Total path length cost ----------
    totalLen = computePathLength(P);
    errTotal = totalLen - params.targetPathLength;

    J = J + params.wTotalLen * errTotal^2;

    for i = 1:N-1
        d = P(i+1, :) - P(i, :);
        l = norm(d);

        if l > 1e-10
            g = 2 * params.wTotalLen * errTotal * d / l;
            G(i, :)   = G(i, :)   - g;
            G(i+1, :) = G(i+1, :) + g;
        end
    end

    %% ---------- Numerical gradient for angle and true-curvature penalties ----------
    if params.useNumericalAngleCurvPenalty
        [Jextra, Gextra] = angleCurvaturePenaltyNumerical(P, params);
        J = J + Jextra;
        G = G + Gextra;
    end
end


function [Jextra, Gextra] = angleCurvaturePenaltyNumerical(P, params)
    epsFD = params.epsFD;

    Jextra = angleCurvaturePenaltyOnly(P, params);
    Gextra = zeros(size(P));

    % Only optimize internal points. Endpoints are fixed.
    for i = 2:size(P, 1)-1
        for dim = 1:2
            Pp = P;
            Pm = P;

            Pp(i, dim) = Pp(i, dim) + epsFD;
            Pm(i, dim) = Pm(i, dim) - epsFD;

            Jp = angleCurvaturePenaltyOnly(Pp, params);
            Jm = angleCurvaturePenaltyOnly(Pm, params);

            Gextra(i, dim) = (Jp - Jm) / (2 * epsFD);
        end
    end
end


function J = angleCurvaturePenaltyOnly(P, params)
    J = 0;

    for i = 2:size(P, 1)-1
        pPrev = P(i-1, :);
        pMid  = P(i, :);
        pNext = P(i+1, :);

        a = pMid - pPrev;
        b = pNext - pMid;

        la = norm(a);
        lb = norm(b);
        chord = norm(pNext - pPrev);

        if la < 1e-8 || lb < 1e-8 || chord < 1e-8
            continue;
        end

        % Turning angle
        c = dot(a, b) / (la * lb);
        c = max(-1, min(1, c));
        theta = acos(c);

        gapTheta = theta - params.thetaMax;

        if gapTheta > 0
            J = J + params.wAngle * gapTheta^2;
        end

        % True three-point curvature:
        % kappa = 4 * triangle_area / (side_a * side_b * side_c)
        crossVal = abs(a(1) * b(2) - a(2) * b(1));
        kappa = 2 * crossVal / (la * lb * chord);

        gapKappa = kappa - params.kappaMaxTrue;

        if gapKappa > 0
            J = J + params.wCurvTrue * gapKappa^2;
        end
    end
end


function [phi, gradPhi] = circleUnionSDF2D(x, obstacles)
    % SDF of union of circles.
    % phi > 0: outside obstacle
    % phi = 0: obstacle boundary
    % phi < 0: inside obstacle

    centers = obstacles(:, 1:2);
    radii = obstacles(:, 3);

    diff = x - centers;
    distToCenter = sqrt(sum(diff.^2, 2));

    phiAll = distToCenter - radii;
    [phi, idx] = min(phiAll);

    if distToCenter(idx) < 1e-10
        gradPhi = [1, 0];
    else
        gradPhi = diff(idx, :) / distToCenter(idx);
    end
end


function P = projectToTrustRegion(P, P0, rTrust)
    for i = 2:size(P, 1)-1
        d = P(i, :) - P0(i, :);
        nd = norm(d);

        if nd > rTrust
            P(i, :) = P0(i, :) + d / nd * rTrust;
        end
    end
end


function minClear = evaluateMinClearance(P, pairs, obstacles, robotRadius, denseQ)
    minClear = inf;
    ts = linspace(0, 1, denseQ + 1);

    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        for q = 1:numel(ts)
            t = ts(q);
            x = (1 - t) * P(i, :) + t * P(j, :);

            [phi, ~] = circleUnionSDF2D(x, obstacles);
            clearance = phi - robotRadius;

            minClear = min(minClear, clearance);
        end
    end
end


function maxKappa = evaluateMaxTrueCurvature(P)
    maxKappa = 0;

    for i = 2:size(P, 1)-1
        pPrev = P(i-1, :);
        pMid  = P(i, :);
        pNext = P(i+1, :);

        a = pMid - pPrev;
        b = pNext - pMid;

        la = norm(a);
        lb = norm(b);
        chord = norm(pNext - pPrev);

        if la < 1e-8 || lb < 1e-8 || chord < 1e-8
            continue;
        end

        crossVal = abs(a(1) * b(2) - a(2) * b(1));
        kappa = 2 * crossVal / (la * lb * chord);

        maxKappa = max(maxKappa, kappa);
    end
end


function maxAngle = evaluateMaxTurningAngle(P)
    maxAngle = 0;

    for i = 2:size(P, 1)-1
        a = P(i, :) - P(i-1, :);
        b = P(i+1, :) - P(i, :);

        la = norm(a);
        lb = norm(b);

        if la < 1e-8 || lb < 1e-8
            continue;
        end

        c = dot(a, b) / (la * lb);
        c = max(-1, min(1, c));

        theta = acos(c);
        maxAngle = max(maxAngle, theta);
    end
end


function minAdjDist = evaluateMinAdjacentDistance(P)
    d = diff(P, 1, 1);
    dist = sqrt(sum(d.^2, 2));
    minAdjDist = min(dist);
end


function maxAdjDist = evaluateMaxAdjacentDistance(P)
    d = diff(P, 1, 1);
    dist = sqrt(sum(d.^2, 2));
    maxAdjDist = max(dist);
end


function len = computePathLength(P)
    d = diff(P, 1, 1);
    len = sum(sqrt(sum(d.^2, 2)));
end


function h = initLiveVisualization(P0, P, pairs, obstacles, params)
    h.fig = figure('Name', 'Live path optimization');
    hold on; grid on; axis equal;

    drawObstacles(obstacles);

    [xPair, yPair] = pairSegmentsToXY(P, pairs);
    h.pairLine = plot(xPair, yPair, '-', ...
        'Color', [0.60, 0.60, 0.60], 'LineWidth', 0.8);

    h.initPath = plot(P0(:,1), P0(:,2), 'k--', 'LineWidth', 1.5);
    h.optPath = plot(P(:,1), P(:,2), 'b-', 'LineWidth', 2.2);

    h.initPts = plot(P0(:,1), P0(:,2), 'ko', 'MarkerSize', 3);
    h.optPts = plot(P(:,1), P(:,2), 'bo', 'MarkerSize', 3);

    h.startPt = plot(P0(1,1), P0(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h.goalPt  = plot(P0(end,1), P0(end,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

    margin = 180;
    xlim([min(P0(:,1))-margin, max(P0(:,1))+margin]);
    ylim([-300, 300]);

    xlabel('x / mm');
    ylabel('y / mm');

    h.title = title(sprintf('Live optimization | thetaMax = %.1f deg | kappaMax = %.5f 1/mm', ...
        rad2deg(params.thetaMax), params.kappaMaxTrue));

    legend('Obstacle', 'Pair segments', 'Initial path', 'Optimized path', ...
           'Initial points', 'Optimized points', 'Start', 'Goal', ...
           'Location', 'bestoutside');

    drawnow;
end


function updateLiveVisualization(h, P, pairs, params, iter, J, minClear, pathLen, maxKappa, maxAngle, minAdjDist, maxAdjDist)
    [xPair, yPair] = pairSegmentsToXY(P, pairs);

    set(h.pairLine, 'XData', xPair, 'YData', yPair);
    set(h.optPath, 'XData', P(:,1), 'YData', P(:,2));
    set(h.optPts, 'XData', P(:,1), 'YData', P(:,2));

    titleText = sprintf(['Iter %d | J = %.1f | clear = %.1f mm | len = %.1f mm | ', ...
                         'kappa = %.5f / %.5f | angle = %.1f / %.1f deg | ', ...
                         'adj = %.1f~%.1f mm'], ...
                         iter, J, minClear, pathLen, ...
                         maxKappa, params.kappaMaxTrue, ...
                         rad2deg(maxAngle), rad2deg(params.thetaMax), ...
                         minAdjDist, maxAdjDist);

    set(h.title, 'String', titleText);

    drawnow limitrate;
end


function [xAll, yAll] = pairSegmentsToXY(P, pairs)
    xAll = nan(3 * size(pairs, 1), 1);
    yAll = nan(3 * size(pairs, 1), 1);

    idx = 1;
    for e = 1:size(pairs, 1)
        i = pairs(e, 1);
        j = pairs(e, 2);

        xAll(idx) = P(i, 1);
        yAll(idx) = P(i, 2);

        xAll(idx + 1) = P(j, 1);
        yAll(idx + 1) = P(j, 2);

        xAll(idx + 2) = nan;
        yAll(idx + 2) = nan;

        idx = idx + 3;
    end
end


function drawObstacles(obstacles)
    theta = linspace(0, 2*pi, 150);

    for k = 1:size(obstacles, 1)
        cx = obstacles(k, 1);
        cy = obstacles(k, 2);
        r  = obstacles(k, 3);

        x = cx + r * cos(theta);
        y = cy + r * sin(theta);

        fill(x, y, [0.85, 0.85, 0.85], ...
            'EdgeColor', [0.25, 0.25, 0.25], ...
            'LineWidth', 1.2);
    end
end