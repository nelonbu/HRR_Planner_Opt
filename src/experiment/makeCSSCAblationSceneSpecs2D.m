function specs = makeCSSCAblationSceneSpecs2D(cfg)
%MAKECSSCABLATIONSCENESPECS2D Build the same 4-by-3 benchmark as simu1.

    base = makeCSSCBenchmarkSceneSpecs2D(struct( ...
        'bounds', cfg.bounds, 'maxEnvSeedTry', cfg.numTrials));
    nFamily = numel(base);
    nDifficulty = numel(cfg.difficultyIds);
    for family = 1:nFamily
        base(family).familyIndex = family;
        base(family).difficultyIndex = 0;
        base(family).difficultyId = "";
        base(family).difficultyName = "";
        base(family).difficultyMetric = "";
        base(family).difficultyValue = nan;
    end
    specs = repmat(base(1), nFamily * nDifficulty, 1);

    k = 0;
    for family = 1:nFamily
        for level = 1:nDifficulty
            k = k + 1;
            spec = base(family);
            spec.familyIndex = family;
            spec.difficultyIndex = level;
            spec.difficultyId = cfg.difficultyIds(level);
            spec.difficultyName = cfg.difficultyNames(level);
            spec.envSeeds = cfg.envSeedBaseByFamily(family) + ...
                (0:cfg.numTrials-1);

            switch family
                case 1
                    spec.gapHeight = cfg.doubleSlit.dGap(level);
                    spec.gapCenterRange = ...
                        cfg.doubleSlit.centerRange(level,:);
                    spec.difficultyMetric = "dGap";
                    spec.difficultyValue = spec.gapHeight;
                case 2
                    spec.dGap = cfg.sChannel.dGap(level);
                    spec.channelW = cfg.sChannel.W;
                    spec.channelH = diff(cfg.bounds(2,:));
                    spec.difficultyMetric = "dGap";
                    spec.difficultyValue = spec.dGap;
                case 3
                    spec.passageWidth = cfg.baffles.passageWidth(level);
                    spec.difficultyMetric = "passageWidth";
                    spec.difficultyValue = spec.passageWidth;
                case 4
                    spec.nCircle = cfg.randomMixed.nCircle(level);
                    spec.nRect = cfg.randomMixed.nRect(level);
                    spec.minGap = cfg.randomMixed.minGap(level);
                    spec.difficultyMetric = "numObstacles";
                    spec.difficultyValue = spec.nCircle + spec.nRect;
            end
            specs(k) = spec;
        end
    end

    if strlength(cfg.sceneId) > 0
        specs = specs(string({specs.id}) == cfg.sceneId);
    end
    if strlength(cfg.difficultyId) > 0
        specs = specs(string({specs.difficultyId}) == cfg.difficultyId);
    end
    if isempty(specs)
        error('No scene matches the requested simu2 selection.');
    end
end
