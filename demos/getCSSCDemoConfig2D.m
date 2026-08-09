function cfg = getCSSCDemoConfig2D()
%GETCSSCDEMOCONFIG2D Shared parameters for the common 2-D CSSC demos.
%
% Edit this file when changing the benchmark workspace, three difficulty
% levels, common RRT settings, or baseline time budgets. Figure styling and
% demo-specific debug controls should remain in their individual scripts.
%
% Typical use:
%   common = getCSSCDemoConfig2D();
%   cfg.bounds = common.bounds;
%   cfg.doubleSlit = common.doubleSlit;

    cfg = struct();
    cfg.profileName = "balanced-v1";

    %% Workspace and common path model
    cfg.bounds = [0, 1; -0.4, 0.4];
    cfg.startPt = [0.05, 0.0];
    cfg.goalPt = [0.95, 0.0];
    cfg.L = 0.15;
    cfg.dMin = 0.03;
    cfg.dPref = 1.3 * cfg.dMin;
    cfg.csscOptimizationClearanceBuffer = 0.002;
    cfg.degree = 3;

    %% Benchmark difficulty levels
    cfg.difficultyNames = ["Easy", "Normal", "Hard"];
    cfg.difficultyIds = ["easy", "normal", "hard"];
    cfg.numDifficultyLevels = numel(cfg.difficultyNames);
    cfg.sceneIds = [ ...
        "double_slit", ...
        "s_channel", ...
        "staggered_baffles", ...
        "random_mixed"];
    cfg.numSceneFamilies = numel(cfg.sceneIds);
    cfg.representativeDifficultyIndex = 2;

    % Double slit: smaller gap and wider center range are harder.
    cfg.doubleSlit.dGap = [0.18, 0.15, 0.12];
    cfg.doubleSlit.xWalls = [0.32, 0.68];
    cfg.doubleSlit.centerRange = [
        -0.10, 0.10
        -0.20, 0.20
        -0.30, 0.30
    ];
    cfg.doubleSlit.seed = [1301, 1308, 1315];
    cfg.doubleSlit.wallThickness = 0.08;

    % Four-rectangle S channel: smaller dGap is harder.
    cfg.sChannel.dGap = [0.20, 0.17, 0.14];
    cfg.sChannel.W = 0.70;
    cfg.sChannel.H = diff(cfg.bounds(2,:));
    cfg.sChannel.seed = [1789, 1593, 1791];
    cfg.sChannel.yInOutRange = [-0.20, 0.20];

    % Alternating baffles: passageWidth is the free vertical opening.
    cfg.baffles.passageWidth = [0.42, 0.40, 0.38];
    cfg.baffles.xCenters = [0.25, 0.50, 0.75];
    cfg.baffles.thickness = 0.05;
    cfg.baffles.pattern = [1, -1, 1];

    % Random mixed obstacles: count and minGap jointly set difficulty.
    cfg.randomMixed.nCircle = [5, 8, 10];
    cfg.randomMixed.nRect = [4, 6, 8];
    cfg.randomMixed.minGap = [0.035, 0.022, 0.012];
    cfg.randomMixed.seed = [2401, 2402, 2403];
    cfg.randomMixed.radiusRange = [0.022, 0.045];
    cfg.randomMixed.halfSizeXRange = [0.022, 0.052];
    cfg.randomMixed.halfSizeYRange = [0.025, 0.060];
    cfg.randomMixed.yawRange = [-pi/4, pi/4];
    cfg.randomMixed.keepoutStart = 0.07;
    cfg.randomMixed.keepoutGoal = 0.07;
    cfg.randomMixed.maxTry = 10000;

    cfg.envSeedBaseByFamily = [1301, 1789, 3301, 2401];
    cfg.maxEnvSeedTry = 20;

    %% Common point-path frontend settings
    cfg.stepSize = 0.030;
    cfg.goalBias = 0.16;
    cfg.goalTol = 0.035;
    cfg.maxIter = 4000;
    cfg.collisionResolution = 0.003;
    cfg.inflateRadius = 0.0;
    cfg.rrtConnectMaxConnectSteps = inf;
    cfg.csscFrontendMethod = 'rrt';

    cfg.rewireRadius = 0.12;
    cfg.minRewireRadius = 0.060;
    cfg.rewireGamma = 0.45;
    cfg.terminateRRTStarOnFirstSolution = false;
    cfg.maxRRTStarNoImproveIter = 1200;

    %% Common experiment and baseline settings
    % Common end-to-end planning budget. Final high-precision evaluation is
    % timed separately and is not charged to this planning budget.
    cfg.maxPlanningTimeSec = 10.0;
    cfg.numSeedsPerScene = 100;
    cfg.plannerSeedBase = 500000;
    cfg.shortcutSeedBase = 710000;
    cfg.numShortcut = 180;
    cfg.csscInitMaxCandidates = 6;
    cfg.csscInitTimeFraction = 0.25;
    cfg.csscInitSeedStride = 10000;
    cfg.csscInitAcceptMargin = 0.0;
    cfg.pointClearanceResolution = 0.0015;
    cfg.figureResolution = 300;

    cfg.rrtscMaxAttempts = 30;
    cfg.rrtscMaxTotalTime = cfg.maxPlanningTimeSec;
    cfg.rrtscCenterlineResolution = 0.001;
    cfg.rrtscCenterlineMinSamples = 800;
    cfg.rrtscChordNU = 240;
    cfg.rrtscMaxRefinement = 2;
    cfg.rrtscControlPointSpacingFactor = 0.35;

    cfg.sprrtNominalLinkCount = 6;
    cfg.sprrtMaxSegmentCount = 12;
    cfg.sprrtThetaMaxDeg = 40;
    cfg.sprrtEntranceBias = 0.05;
    cfg.sprrtMaxTimeSec = cfg.maxPlanningTimeSec;
    cfg.sprrtPathOptimizationMode = 'non-worsening-angle';
end
