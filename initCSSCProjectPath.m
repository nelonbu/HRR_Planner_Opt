function root = initCSSCProjectPath()
%INITCSSCPROJECTPATH Initialize MATLAB path for the CSSC-FTL project.
%
% Put this file at the project root and run:
%   root = initCSSCProjectPath;
%
% The function adds src/ and demos/ to the MATLAB path and creates
% a results/ folder if it does not exist.

    root = fileparts(mfilename('fullpath'));

    addpath(root);
    addIfExists(fullfile(root, 'src'));
    addIfExists(fullfile(root, 'demos'));

    makeIfMissing(fullfile(root, 'results'));
    makeIfMissing(fullfile(root, 'results', 'runs'));
    makeIfMissing(fullfile(root, 'results', 'figures'));

    assignin('base', 'CSSC_PROJECT_ROOT', root);

    fprintf('[CSSC] Project root initialized:\n');
    fprintf('       %s\n', root);
end

function addIfExists(p)
    if exist(p, 'dir')
        addpath(genpath(p));
        fprintf('[CSSC] addpath: %s\n', p);
    end
end

function makeIfMissing(p)
    if ~exist(p, 'dir')
        mkdir(p);
        fprintf('[CSSC] mkdir  : %s\n', p);
    end
end
