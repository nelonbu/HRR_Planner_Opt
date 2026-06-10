function output = prepareCSSCOutput(params)
%PREPARECSSCOUTPUT Create an output directory for a CSSC run.
%
% Temporary runs use:
%   results/tmp
%
% Formal runs use:
%   results/run_yyyymmdd_HHMMSS

    if nargin < 1 || ~isstruct(params)
        params = struct();
    end

    if ~isfield(params, 'output') || isempty(params.output)
        params.output = struct();
    end

    outputParams = params.output;
    if ~isfield(outputParams, 'enable'); outputParams.enable = true; end
    if ~isfield(outputParams, 'mode'); outputParams.mode = 'tmp'; end
    if ~isfield(outputParams, 'rootDir'); outputParams.rootDir = defaultResultsRoot(); end

    output = struct();
    output.enable = outputParams.enable;
    output.mode = outputParams.mode;
    output.rootDir = outputParams.rootDir;
    output.runName = '';
    output.dir = '';

    if ~output.enable
        return;
    end

    makeIfMissing(output.rootDir);

    switch lower(output.mode)
        case {'formal','run','official'}
            output.runName = ['run_' datestr(now, 'yyyymmdd_HHMMSS')];
            output.dir = fullfile(output.rootDir, output.runName);

        case {'tmp','temp','temporary'}
            output.runName = 'tmp';
            output.dir = fullfile(output.rootDir, 'tmp');

        otherwise
            error('Unknown params.output.mode: %s', output.mode);
    end

    makeIfMissing(output.dir);
end

function rootDir = defaultResultsRoot()
    try
        projectRoot = evalin('base', 'CSSC_PROJECT_ROOT');
        if ischar(projectRoot) || isstring(projectRoot)
            rootDir = fullfile(char(projectRoot), 'results');
            return;
        end
    catch
    end

    thisDir = fileparts(mfilename('fullpath'));
    srcDir = fileparts(thisDir);
    projectRoot = fileparts(srcDir);
    rootDir = fullfile(projectRoot, 'results');
end

function makeIfMissing(p)
    if ~exist(p, 'dir')
        mkdir(p);
    end
end
