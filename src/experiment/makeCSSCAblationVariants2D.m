function variants = makeCSSCAblationVariants2D(groupName)
%MAKECSSCABLATIONVARIANTS2D Define reproducible CSSC ablation variants.
%
% groupName: 'core' (default), 'stability', or 'all'. The full CSSC-SAG
% reference is included in every group.

    if nargin < 1 || isempty(groupName)
        groupName = 'core';
    end
    groupName = lower(char(string(groupName)));

    full = makeVariant( ...
        'cssc_sag', 'CSSC-SAG', 'reference', true, ...
        'cssc-chord', 'semi-analytic');

    core = [ ...
        makeVariant('init_only', 'B-spline Init', 'core', false, ...
            'cssc-chord', 'none')
        makeVariant('point_sdf', 'Point-SDF', 'core', true, ...
            'centerline', 'semi-analytic')
        makeVariant('cssc_fd', 'CSSC-FD', 'core', true, ...
            'cssc-chord', 'finite-diff')
        full
    ];

    singleInit = full;
    singleInit.id = 'single_init';
    singleInit.label = 'CSSC-SingleInit';
    singleInit.group = 'stability';
    singleInit.initPolicy = 'first-attempt';

    lastIter = full;
    lastIter.id = 'last_iter';
    lastIter.label = 'CSSC-LastIter';
    lastIter.group = 'stability';
    lastIter.returnPolicy = 'last-iterate';

    noPref = full;
    noPref.id = 'no_preferred_clearance';
    noPref.label = 'CSSC-NoPref';
    noPref.group = 'stability';
    noPref.wClearScale = 0;

    allActive = full;
    allActive.id = 'all_active';
    allActive.label = 'CSSC-AllActive';
    allActive.group = 'stability';
    allActive.activeMode = 'all';

    noBuffer = full;
    noBuffer.id = 'no_safety_buffer';
    noBuffer.label = 'CSSC-NoBuffer';
    noBuffer.group = 'stability';
    noBuffer.useSafetyBuffer = false;

    noPatience = full;
    noPatience.id = 'no_patience';
    noPatience.label = 'CSSC-NoPatience';
    noPatience.group = 'stability';
    noPatience.disablePatience = true;

    stability = [full; singleInit; lastIter; noPref; allActive; ...
        noBuffer; noPatience];

    switch groupName
        case 'core'
            variants = core;
        case 'stability'
            variants = stability;
        case 'all'
            variants = [core; stability(2:end)];
        otherwise
            error('Unknown CSSC ablation group: %s', groupName);
    end
end

function variant = makeVariant(id, label, group, runOptimizer, ...
        objectiveMode, gradMode)
    variant = struct();
    variant.id = id;
    variant.label = label;
    variant.group = group;
    variant.runOptimizer = logical(runOptimizer);
    variant.objectiveMode = objectiveMode;
    variant.gradMode = gradMode;
    variant.initPolicy = 'selected';
    variant.returnPolicy = 'best-safe';
    variant.activeMode = 'topk';
    variant.useSafetyBuffer = true;
    variant.wClearScale = 1;
    variant.disablePatience = false;
end
