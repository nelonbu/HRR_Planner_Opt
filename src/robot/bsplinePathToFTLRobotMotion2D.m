function motion = bsplinePathToFTLRobotMotion2D(P, params, robot, opts)
%BSPLINEPATHTOFTLROBOTMOTION2D Convert a B-spline path to FTL joint motion.
%
% The robot starts as a straight equal-link chain pointing along +x, with
% the leading tip located at the path start. At every simulation step the
% previous whole-body posture is first translated along entryDirection by
% advanceStep. Joints whose translated x coordinate has passed the path
% start are treated as entered joints. The non-entered prefix keeps the
% translated position, while the entered suffix is rebuilt on the B-spline
% by fixed-chord searches of length L.
%
% Outputs:
%   motion.joints        : nStep-by-numJoints-by-2 joint positions
%   motion.q             : nStep-by-numLinks, [base angle, relative hinges]
%   motion.jointAnglesRel : nStep-by-(numLinks-1) relative hinge angles
%   motion.enteredMask   : nStep-by-numJoints, true if joint lies on path

    if nargin < 4 || isempty(opts)
        opts = struct();
    end

    opts = setDefaults(opts, robot.linkLength);
    validateAdvanceStep(robot.linkLength, opts.advanceStep, opts.stepDivTol);

    degree = getField(params, 'degree', 3);
    if isfield(params, 'knot') && ~isempty(params.knot)
        knot = params.knot;
    else
        knot = makeClampedUniformKnot(size(P, 1), degree);
    end

    uGrid = linspace(0, 1, opts.pathSampleN).';
    path = evalBSplinePath2D(P, uGrid, degree, knot);
    sGrid = pathArcLength(path);
    totalLen = sGrid(end);

    if totalLen <= 0
        error('B-spline path has zero length.');
    end

    nJ = robot.numJoints;
    nL = robot.numLinks;
    startPt = path(1,:);
    goalPt = path(end,:);
    entryDir = normalizeVec(getField(opts, 'entryDirection', [1, 0]));
    initialJoints = makeInitialJoints(startPt, entryDir, robot.linkLength, nJ);

    maxSteps = opts.maxSteps;
    if isempty(maxSteps)
        maxSteps = ceil((totalLen + robot.numLinks * robot.linkLength) / opts.advanceStep) + 10;
    end

    joints = nan(maxSteps, nJ, 2);
    enteredMask = false(maxSteps, nJ);
    uJoint = nan(maxSteps, nJ);
    sJoint = nan(maxSteps, nJ);
    rootPositions = nan(maxSteps, 2);
    q = nan(maxSteps, nL);
    feedDistance = nan(maxSteps, 1);

    it = 1;
    feed = 0;
    J = initialJoints;
    U = nan(nJ, 1);
    S = nan(nJ, 1);
    entered = false(nJ, 1);
    U(nJ) = 0;
    S(nJ) = 0;
    entered(nJ) = true;

    [joints, enteredMask, uJoint, sJoint, rootPositions, q, feedDistance] = ...
        storeFrame(it, J, U, S, entered, feed, joints, enteredMask, uJoint, sJoint, ...
        rootPositions, q, feedDistance);

    stopReason = 'maxSteps';
    failed = false;
    failMessage = '';

    while it < maxSteps
        feed = feed + opts.advanceStep;
        Jpred = J + opts.advanceStep * entryDir;

        enteredRaw = entered | (Jpred(:,1) > startPt(1) + opts.enterTol);
        if ~any(enteredRaw)
            enteredRaw(nJ) = true;
        end

        firstEntered = find(enteredRaw, 1, 'first');
        enteredNew = false(nJ, 1);
        enteredNew(firstEntered:nJ) = true;

        Jnew = Jpred;
        Unew = nan(nJ, 1);
        Snew = nan(nJ, 1);

        if firstEntered == 1
            sFirst = nextArcPosition(S(1), Jpred(1,:), startPt, entryDir, ...
                opts.advanceStep, totalLen);
            uFirst = interpArcToU(sFirst, sGrid, uGrid);
            Jnew(1,:) = evalBSplinePath2D(P, uFirst, degree, knot);
            Unew(1) = uFirst;
            Snew(1) = sFirst;
        else
            lowerU = 0;
            if isfinite(U(firstEntered))
                lowerU = U(firstEntered);
            end
            [ok, uFirst, pFirst] = solveForwardChordFromPoint(P, degree, knot, ...
                lowerU, Jpred(firstEntered-1,:), robot.linkLength, opts);
            if ~ok
                failed = true;
                failMessage = sprintf('entry chord search failed at step %d, joint %d.', ...
                    it + 1, firstEntered);
                break;
            end
            Jnew(firstEntered,:) = pFirst;
            Unew(firstEntered) = uFirst;
            Snew(firstEntered) = interpUToArc(uFirst, sGrid, uGrid);
        end

        for jj = firstEntered+1:nJ
            lowerU = Unew(jj-1);
            if isfinite(U(jj))
                lowerU = max(lowerU, U(jj));
            end
            [ok, uNext, pNext] = solveForwardChordFromPoint(P, degree, knot, ...
                lowerU, Jnew(jj-1,:), robot.linkLength, opts);
            if ~ok
                failed = true;
                failMessage = sprintf('path chord search failed at step %d, joint %d.', ...
                    it + 1, jj);
                break;
            end
            Jnew(jj,:) = pNext;
            Unew(jj) = uNext;
            Snew(jj) = interpUToArc(uNext, sGrid, uGrid);
        end

        if failed
            break;
        end

        it = it + 1;
        J = Jnew;
        U = Unew;
        S = Snew;
        entered = enteredNew;

        [joints, enteredMask, uJoint, sJoint, rootPositions, q, feedDistance] = ...
            storeFrame(it, J, U, S, entered, feed, joints, enteredMask, uJoint, sJoint, ...
            rootPositions, q, feedDistance);

        if norm(J(end,:) - goalPt) <= opts.goalTol && strcmpi(opts.stopWhen, 'tip')
            stopReason = 'tipGoal';
            break;
        end
    end

    nStep = it;
    joints = joints(1:nStep,:,:);
    enteredMask = enteredMask(1:nStep,:);
    uJoint = uJoint(1:nStep,:);
    sJoint = sJoint(1:nStep,:);
    rootPositions = rootPositions(1:nStep,:);
    q = q(1:nStep,:);
    feedDistance = feedDistance(1:nStep);

    if failed
        stopReason = 'chordSearchFailed';
    end

    motion = struct();
    motion.joints = joints;
    motion.q = q;
    motion.jointAnglesRel = q(:,2:end);
    motion.rootPositions = rootPositions;
    motion.enteredMask = enteredMask;
    motion.uJoint = uJoint;
    motion.sJoint = sJoint;
    motion.sTip = sJoint(:,end);
    motion.feedDistance = feedDistance;
    motion.pathSample = path;
    motion.pathU = uGrid;
    motion.pathArcLength = sGrid;
    motion.totalPathLength = totalLen;
    motion.startPt = startPt;
    motion.goalPt = goalPt;
    motion.initialJoints = initialJoints;
    motion.stopReason = stopReason;
    motion.failed = failed;
    motion.failMessage = failMessage;
    motion.paramsUsed = params;
    motion.optsUsed = opts;
    motion.robot = robot;
end

function opts = setDefaults(opts, linkLength)
    if ~isfield(opts, 'advanceStep'); opts.advanceStep = linkLength / 5; end
    if ~isfield(opts, 'pathSampleN'); opts.pathSampleN = 1600; end
    if ~isfield(opts, 'chordSolveTol'); opts.chordSolveTol = 1e-5; end
    if ~isfield(opts, 'chordSolveMaxIter'); opts.chordSolveMaxIter = 40; end
    if ~isfield(opts, 'chordSearchSamples'); opts.chordSearchSamples = 48; end
    if ~isfield(opts, 'goalTol'); opts.goalTol = 1e-3; end
    if ~isfield(opts, 'stepDivTol'); opts.stepDivTol = 1e-9; end
    if ~isfield(opts, 'enterTol'); opts.enterTol = 1e-12; end
    if ~isfield(opts, 'stopWhen'); opts.stopWhen = 'tip'; end
    if ~isfield(opts, 'entryDirection'); opts.entryDirection = [1, 0]; end
    if ~isfield(opts, 'maxSteps'); opts.maxSteps = []; end
end

function validateAdvanceStep(linkLength, advanceStep, tol)
    ratio = linkLength / advanceStep;
    if abs(ratio - round(ratio)) > tol
        error('advanceStep must divide linkLength. Got linkLength/advanceStep = %.12g.', ratio);
    end
end

function [joints, enteredMask, uJoint, sJoint, rootPositions, q, feedDistance] = storeFrame(it, J, U, S, entered, feed, joints, enteredMask, uJoint, sJoint, rootPositions, q, feedDistance)

    angles = jointPositionsToAbsAngles(J);
    rel = absAnglesToRobotQ(angles);

    joints(it,:,:) = reshape(J, 1, size(J,1), 2);
    enteredMask(it,:) = entered.';
    uJoint(it,:) = U.';
    sJoint(it,:) = S.';
    rootPositions(it,:) = J(1,:);
    q(it,:) = rel;
    feedDistance(it) = feed;
end

function s = pathArcLength(path)
    ds = vecnorm(diff(path, 1, 1), 2, 2);
    s = [0; cumsum(ds)];
end

function u = interpArcToU(s, sGrid, uGrid)
    [sUnique, idx] = unique(sGrid, 'stable');
    uUnique = uGrid(idx);
    s = min(max(s, sUnique(1)), sUnique(end));
    u = interp1(sUnique, uUnique, s, 'linear', 'extrap');
end

function s = interpUToArc(u, sGrid, uGrid)
    u = min(max(u, uGrid(1)), uGrid(end));
    s = interp1(uGrid, sGrid, u, 'linear', 'extrap');
end

function J0 = makeInitialJoints(startPt, entryDir, linkLength, nJ)
    J0 = zeros(nJ, 2);
    for j = 1:nJ
        offset = (nJ - j) * linkLength;
        J0(j,:) = startPt - offset * entryDir;
    end
end

function sNext = nextArcPosition(sPrev, pPred, startPt, entryDir, advanceStep, totalLen)
    if isfinite(sPrev)
        sNext = sPrev + advanceStep;
    else
        sNext = dot(pPred - startPt, entryDir);
    end
    sNext = min(max(sNext, 0), totalLen);
end

function [ok, uNext, pNext] = solveForwardChordFromPoint(P, degree, knot, uLow, pAnchor, L, opts)
    ok = false;
    uNext = nan;
    pNext = [nan, nan];

    if ~isfinite(uLow)
        uLow = 0;
    end
    uLow = min(max(uLow, 0), 1);

    pLow = evalBSplinePath2D(P, uLow, degree, knot);
    fLow = norm(pLow - pAnchor) - L;
    if abs(fLow) <= opts.chordSolveTol
        ok = true;
        uNext = uLow;
        pNext = pLow;
        return;
    end

    us = linspace(uLow, 1, max(3, opts.chordSearchSamples));
    f = nan(size(us));
    for i = 1:numel(us)
        p = evalBSplinePath2D(P, us(i), degree, knot);
        f(i) = norm(p - pAnchor) - L;
    end

    idx = find(f(1:end-1) <= opts.chordSolveTol & f(2:end) >= -opts.chordSolveTol, 1, 'first');
    if isempty(idx)
        return;
    end

    lo = us(idx);
    hi = us(idx+1);
    for k = 1:opts.chordSolveMaxIter
        mid = 0.5 * (lo + hi);
        pMid = evalBSplinePath2D(P, mid, degree, knot);
        fMid = norm(pMid - pAnchor) - L;

        if fMid >= 0
            hi = mid;
        else
            lo = mid;
        end

        if abs(fMid) <= opts.chordSolveTol
            break;
        end
    end

    uNext = 0.5 * (lo + hi);
    pNext = evalBSplinePath2D(P, uNext, degree, knot);
    ok = isfinite(uNext) && all(isfinite(pNext));
end

function angles = jointPositionsToAbsAngles(J)
    d = diff(J, 1, 1);
    angles = atan2(d(:,2), d(:,1)).';
end

function q = absAnglesToRobotQ(angles)
    q = zeros(size(angles));
    if isempty(angles)
        return;
    end
    q(1) = angles(1);
    for i = 2:numel(angles)
        q(i) = wrapToPiLocal(angles(i) - angles(i-1));
    end
end

function y = wrapToPiLocal(x)
    y = mod(x + pi, 2*pi) - pi;
end

function v = normalizeVec(v)
    v = v(:).';
    n = norm(v);
    if n < 1e-12
        v = [1, 0];
    else
        v = v / n;
    end
end

function val = getField(s, name, defaultVal)
    if isstruct(s) && isfield(s, name)
        val = s.(name);
    else
        val = defaultVal;
    end
end




