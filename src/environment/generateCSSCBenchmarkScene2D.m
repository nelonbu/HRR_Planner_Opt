function [obstacles, envInfo] = generateCSSCBenchmarkScene2D(spec, cfg, envSeed)
%GENERATECSSCBENCHMARKSCENE2D Generate one common benchmark scene instance.
%   The same SPEC and ENVSEED always produce the same obstacles. Double-slit
%   gap centers, S-channel geometry, and random obstacles are seed-driven;
%   the current staggered-baffle geometry is deterministic.

    opts = struct();
    opts.bounds = cfg.bounds;
    opts.startPt = cfg.startPt;
    opts.goalPt = cfg.goalPt;
    opts.seed = envSeed;

    switch lower(char(spec.kind))
        case 'structured'
            switch lower(char(spec.type))
                case 'offsetdoubleslit'
                    opts.xWalls = spec.xWalls;
                    opts.gapHeight = spec.gapHeight;
                    opts.wallThickness = spec.wallThickness;
                    opts.gapCentersY = sampleDoubleSlitCenters( ...
                        spec.gapCenterRange, cfg.bounds, ...
                        opts.gapHeight, envSeed);

                case 'fourrectschannel'
                    opts.dGap = spec.dGap;
                    opts.W = spec.channelW;
                    opts.H = spec.channelH;
                    opts.yInOutRange = [-0.20, 0.20];

                case 'staggeredbaffles3'
                    spanY = cfg.bounds(2,2) - cfg.bounds(2,1);
                    halfHeight = 0.5 * (spanY - spec.passageWidth);
                    centers = zeros(3, 2);
                    centers(:,1) = spec.baffleXCenters(:);
                    for i = 1:3
                        if spec.bafflePattern(i) > 0
                            centers(i,2) = cfg.bounds(2,2) - halfHeight;
                        else
                            centers(i,2) = cfg.bounds(2,1) + halfHeight;
                        end
                    end
                    opts.centers = centers;
                    opts.halfSize = [0.5 * spec.baffleThickness, halfHeight];

                otherwise
                    error('Unsupported structured benchmark type: %s', ...
                        char(spec.type));
            end

            [obstacles, envInfo] = ...
                generateCSSCStructuredEnvironment2D(spec.type, opts);

        case 'random'
            opts.minGap = spec.minGap;
            opts.maxTry = spec.maxTry;

            switch lower(char(spec.type))
                case 'randommixed'
                    opts.nCircle = spec.nCircle;
                    opts.nRect = spec.nRect;
                    opts.radiusRange = spec.radiusRange;
                    opts.halfSizeXRange = spec.halfSizeXRange;
                    opts.halfSizeYRange = spec.halfSizeYRange;
                    opts.yawRange = spec.yawRange;
                    opts.keepoutStart = spec.keepoutStart;
                    opts.keepoutGoal = spec.keepoutGoal;

                otherwise
                    error('Unsupported random benchmark type: %s', ...
                        char(spec.type));
            end

            [obstacles, envInfo] = generateCSSCEnvironment2D(spec.type, opts);

        otherwise
            error('Unknown benchmark scene kind: %s', char(spec.kind));
    end

    envInfo.id = char(spec.id);
    envInfo.name = char(spec.name);
    envInfo.seed = envSeed;
end

function centers = sampleDoubleSlitCenters(centerRange, bounds, gapHeight, seed)
    feasibleRange = [bounds(2,1), bounds(2,2)] + ...
        [0.5, -0.5] * gapHeight;
    sampleRange = [max(centerRange(1), feasibleRange(1)), ...
        min(centerRange(2), feasibleRange(2))];
    if sampleRange(1) >= 0 || sampleRange(2) <= 0
        error('Double-slit center range must span zero after clipping.');
    end

    stream = RandStream('mt19937ar', 'Seed', seed);
    centers = [ ...
        sampleRange(1) + (0 - sampleRange(1)) * rand(stream), ...
        0 + (sampleRange(2) - 0) * rand(stream)];
end
