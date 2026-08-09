function tests = test_cssc_ablation2d
%TEST_CSSC_ABLATION2D Focused checks for the paired ablation extensions.
    tests = functiontests(localfunctions);
end

function testCoreVariantDefinition(testCase)
    variants = makeCSSCAblationVariants2D('core');
    verifyEqual(testCase, string({variants.id}), ...
        ["init_only", "point_sdf", "cssc_fd", "cssc_sag"]);
    verifyEqual(testCase, variants(4).objectiveMode, 'cssc-chord');
    verifyEqual(testCase, variants(4).returnPolicy, 'best-safe');
end

function testCenterlineSemiGradient(testCase)
    P = [0 0; 0.25 0.02; 0.50 0.04; 0.75 0.02; 1 0];
    Pref = P;
    obstacles = obsCircle2D([0.50, 0.12], 0.08);
    params = objectiveParams(P);
    [~, ~, grad] = objectiveCenterline2D_SemiGrad( ...
        P, Pref, obstacles, params);

    h = 1e-6;
    gradFD = zeros(size(P));
    for i = 2:size(P,1)-1
        for j = 1:2
            Pp = P; Pm = P;
            Pp(i,j) = Pp(i,j) + h;
            Pm(i,j) = Pm(i,j) - h;
            Jp = objectiveCenterline2D_Value(Pp, Pref, obstacles, params);
            Jm = objectiveCenterline2D_Value(Pm, Pref, obstacles, params);
            gradFD(i,j) = (Jp-Jm)/(2*h);
        end
    end
    relError = norm(grad(2:end-1,:) - gradFD(2:end-1,:), 'fro') / ...
        max(1, norm(gradFD(2:end-1,:), 'fro'));
    verifyLessThan(testCase, relError, 2e-3);
end

function testSceneMatrixDefinition(testCase)
    cfg = getCSSCDemoConfig2D();
    cfg.numTrials = 2;
    cfg.sceneId = "";
    cfg.difficultyId = "";
    specs = makeCSSCAblationSceneSpecs2D(cfg);
    verifyEqual(testCase, numel(specs), 12);
    verifyTrue(testCase, all(arrayfun(@(s) numel(s.envSeeds)==2, specs)));
end

function params = objectiveParams(P)
    params = struct();
    params.degree = 3;
    params.knot = makeClampedUniformKnot(size(P,1), params.degree);
    params.envOpts = struct('uRange', [0,1], 'nU', 35);
    params.dMin = 0.08;
    params.dPref = 0.12;
    params.wObs = 20;
    params.wClear = 2;
    params.wRef = 0;
    params.wSmooth = 0.1;
    params.wLength = 0.01;
    params.wTrust = 0;
    params.activeMode = 'all';
    params.activeTopK = 100;
    params.activeClearanceMargin = 1;
end
