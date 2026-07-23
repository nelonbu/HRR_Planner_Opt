function [pathOut, info] = shortcutPath2D(pathIn, obstacles, opts)
%SHORTCUTPATH2D Randomly shortcut a collision-free polyline.
%
% Key opts: numShortcut, collisionResolution, inflateRadius, bounds, seed.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'numShortcut'); opts.numShortcut = 100; end
    if ~isfield(opts, 'seed'); opts.seed = []; end

    if ~isempty(opts.seed)
        rng(opts.seed);
    end

    pathOut = pathIn;
    info = struct('numAttempt', 0, 'numAccepted', 0, ...
        'initialNodes', size(pathIn, 1), 'finalNodes', size(pathIn, 1));

    if size(pathOut, 1) <= 2
        return;
    end

    for k = 1:opts.numShortcut
        info.numAttempt = info.numAttempt + 1;
        n = size(pathOut, 1);
        if n <= 2
            break;
        end

        ids = sort(randperm(n, 2));
        i = ids(1);
        j = ids(2);
        if j <= i + 1
            continue;
        end

        if isSegmentCollisionFree2D(pathOut(i,:), pathOut(j,:), obstacles, opts)
            pathOut = [pathOut(1:i,:); pathOut(j:end,:)]; %#ok<AGROW>
            info.numAccepted = info.numAccepted + 1;
        end
    end

    info.finalNodes = size(pathOut, 1);
end
