function specs = makeCSSCBenchmarkSceneSpecs2D(cfg)
%MAKECSSCBENCHMARKSCENESPECS2D Common four-scene benchmark definitions.
%   SPECS = makeCSSCBenchmarkSceneSpecs2D(CFG) returns the scene parameters
%   shared by qualitative figures and baseline experiments. CFG must contain
%   bounds and may contain maxEnvSeedTry (default: 20).

    nSeed = getCfg(cfg, 'maxEnvSeedTry', 20);
    if ~isscalar(nSeed) || ~isfinite(nSeed) || nSeed < 1
        error('cfg.maxEnvSeedTry must be a positive scalar.');
    end
    nSeed = round(nSeed);

    specs = repmat(emptySceneSpec(), 4, 1);

    specs(1).id = "double_slit";
    specs(1).name = "Double Slit";
    specs(1).kind = "structured";
    specs(1).type = "offsetDoubleSlit";
    specs(1).envSeeds = 1:nSeed;
    specs(1).gapHeight = 0.08;
    specs(1).xWalls = [0.32, 0.68];
    specs(1).gapCenterRange = [-0.30, 0.30];
    specs(1).wallThickness = 0.08;

    specs(2).id = "s_channel";
    specs(2).name = "S Channel";
    specs(2).kind = "structured";
    specs(2).type = "fourRectSChannel";
    specs(2).envSeeds = [1593, 31, 1142, 1101:(1101 + nSeed - 1)];
    specs(2).dGap = 0.10;
    specs(2).channelW = 0.64;
    specs(2).channelH = cfg.bounds(2,2) - cfg.bounds(2,1);

    specs(3).id = "staggered_baffles";
    specs(3).name = "Staggered Baffles";
    specs(3).kind = "structured";
    specs(3).type = "staggeredBaffles3";
    specs(3).envSeeds = 1:nSeed;
    specs(3).passageWidth = 0.36;
    specs(3).baffleXCenters = [0.25, 0.50, 0.75];
    specs(3).baffleThickness = 0.05;
    specs(3).bafflePattern = [1, -1, 1];

    specs(4).id = "random_mixed";
    specs(4).name = "Random Mixed Obstacles";
    specs(4).kind = "random";
    specs(4).type = "randomMixed";
    specs(4).envSeeds = 2401:(2401 + nSeed);
    specs(4).nCircle = 8;
    specs(4).nRect = 6;
    specs(4).radiusRange = [0.022, 0.045];
    specs(4).halfSizeXRange = [0.022, 0.052];
    specs(4).halfSizeYRange = [0.025, 0.060];
    specs(4).yawRange = [-pi/4, pi/4];
    specs(4).minGap = 0.020;
    specs(4).keepoutStart = 0.07;
    specs(4).keepoutGoal = 0.07;
    specs(4).maxTry = 10000;
end

function spec = emptySceneSpec()
    spec = struct();
    spec.id = "";
    spec.name = "";
    spec.kind = "";
    spec.type = "";
    spec.envSeeds = 1;
    spec.gapHeight = nan;
    spec.xWalls = [nan, nan];
    spec.gapCentersY = [nan, nan];
    spec.gapCenterRange = [nan, nan];
    spec.wallThickness = nan;
    spec.dGap = nan;
    spec.channelW = nan;
    spec.channelH = nan;
    spec.passageWidth = nan;
    spec.baffleXCenters = [nan, nan, nan];
    spec.baffleThickness = nan;
    spec.bafflePattern = [nan, nan, nan];
    spec.nObs = nan;
    spec.nCircle = nan;
    spec.nRect = nan;
    spec.radiusRange = [nan, nan];
    spec.halfSizeXRange = [nan, nan];
    spec.halfSizeYRange = [nan, nan];
    spec.yawRange = [nan, nan];
    spec.minGap = nan;
    spec.keepoutStart = nan;
    spec.keepoutGoal = nan;
    spec.maxTry = nan;
end

function value = getCfg(cfg, fieldName, defaultValue)
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        value = cfg.(fieldName);
    else
        value = defaultValue;
    end
end
