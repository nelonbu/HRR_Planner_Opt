function result = makeCSSCExperimentResult( ...
        config, seed, obstacles, paths, optimizerInfo, metrics, timing, ...
        successFlags, stageMetrics, parameters, provenance)
%MAKECSSCEXPERIMENTRESULT Pack a stable result.mat structure.
%
% Required saved fields:
%   config, seed, obstacles, paths.Pinit, paths.Popt, paths.pathRRT,
%   optimizerInfo, highPrecisionMetrics, timing, successFlags
%
% Optional experiment-audit fields:
%   stageHighPrecisionMetrics : frontend/initial/final evaluations
%   parameters                : exact evaluation/optimization snapshots
%   provenance                : code, MATLAB, and hardware metadata
%
% The function accepts partial structs and fills missing top-level fields,
% so demos can save comparable result.mat files without duplicating packing
% code.

    if nargin < 1 || isempty(config); config = struct(); end
    if nargin < 2 || isempty(seed); seed = struct(); end
    if nargin < 3; obstacles = []; end
    if nargin < 4 || isempty(paths); paths = struct(); end
    if nargin < 5; optimizerInfo = []; end
    if nargin < 6 || isempty(metrics); metrics = struct(); end
    if nargin < 7 || isempty(timing); timing = struct(); end
    if nargin < 8 || isempty(successFlags); successFlags = struct(); end
    if nargin < 9 || isempty(stageMetrics); stageMetrics = struct(); end
    if nargin < 10 || isempty(parameters); parameters = struct(); end
    if nargin < 11 || isempty(provenance); provenance = struct(); end

    paths = setPathDefault(paths, 'Pinit', []);
    paths = setPathDefault(paths, 'Popt', []);
    paths = setPathDefault(paths, 'Pref', []);
    paths = setPathDefault(paths, 'pathRRT', []);
    paths = setPathDefault(paths, 'pathBSplineInit', []);
    paths = setPathDefault(paths, 'pathOptimized', []);

    if ~isfield(successFlags, 'plannerSuccess')
        successFlags.plannerSuccess = true;
    end
    if ~isfield(successFlags, 'highPrecisionSuccess')
        successFlags.highPrecisionSuccess = getMetricFlag(metrics, 'success');
    end
    if ~isfield(successFlags, 'dMinSatisfied')
        successFlags.dMinSatisfied = getMetricFlag(metrics, 'dMinSatisfied');
    end
    if ~isfield(successFlags, 'pointSuccess')
        successFlags.pointSuccess = getMetricFlag(metrics, 'pointSuccess');
    end

    result = struct();
    result.version = 'CSSC_stable_v1';
    result.createdAt = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    result.config = config;
    result.seed = seed;
    result.obstacles = obstacles;
    result.paths = paths;
    result.optimizerInfo = optimizerInfo;
    result.highPrecisionMetrics = metrics;
    result.stageHighPrecisionMetrics = stageMetrics;
    result.parameters = parameters;
    result.provenance = provenance;
    result.timing = timing;
    result.successFlags = successFlags;
end

function s = setPathDefault(s, name, value)
    if ~isfield(s, name)
        s.(name) = value;
    end
end

function tf = getMetricFlag(metrics, name)
    tf = false;
    if isstruct(metrics) && isfield(metrics, name)
        tf = logical(metrics.(name));
    end
end
