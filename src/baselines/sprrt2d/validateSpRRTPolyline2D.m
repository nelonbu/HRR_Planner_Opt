function validation = validateSpRRTPolyline2D(path, obstacles, opts)
%VALIDATESPRRTPOLYLINE2D Recheck bounds, centerline collision and path turns.

    validation = struct();
    validation.valid = false;
    validation.collisionFree = false;
    validation.turnLimitSatisfied = false;
    validation.maxTurn = nan;
    validation.pathLength = nan;
    validation.message = 'Invalid path.';

    if ~isnumeric(path) || size(path,2) ~= 2 || ...
            size(path,1) < 2 || any(~isfinite(path(:)))
        return;
    end

    collisionOpts = struct( ...
        'bounds', opts.bounds, ...
        'collisionResolution', opts.collisionResolution, ...
        'inflateRadius', opts.inflateRadius);
    collisionFree = true;
    for i = 1:size(path,1)-1
        if ~isSegmentCollisionFree2D( ...
                path(i,:), path(i+1,:), obstacles, collisionOpts)
            collisionFree = false;
            break;
        end
    end

    turns = pathTurnAngles(path);
    if isempty(turns)
        maxTurn = 0;
    else
        maxTurn = max(turns);
    end
    turnLimitSatisfied = ...
        maxTurn <= opts.thetaMax + opts.angleTolerance;

    validation.collisionFree = collisionFree;
    validation.turnLimitSatisfied = turnLimitSatisfied;
    validation.maxTurn = maxTurn;
    validation.pathLength = sum(vecnorm(diff(path, 1, 1), 2, 2));
    validation.valid = collisionFree && turnLimitSatisfied;
    validation.message = ternary(validation.valid, 'valid', ...
        'Centerline collision or turn-limit violation.');
end

function turns = pathTurnAngles(path)
    if size(path,1) < 3
        turns = zeros(0,1);
        return;
    end
    vIn = diff(path(1:end-1,:), 1, 1);
    vOut = diff(path(2:end,:), 1, 1);
    denominator = vecnorm(vIn, 2, 2) .* vecnorm(vOut, 2, 2);
    cosine = sum(vIn .* vOut, 2) ./ max(denominator, eps);
    cosine = max(-1, min(1, cosine));
    turns = acos(cosine);
end

function value = ternary(condition, yesValue, noValue)
    if condition
        value = yesValue;
    else
        value = noValue;
    end
end
