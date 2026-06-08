function root = initProjectPath()
%INITCSSCPROJECTPATH Initialize MATLAB path for the CSSC project.
%
% Put this file at the project root:
%
%   project_root/
%       initCSSCProjectPath.m
%       src/
%       results/
%
% Usage:
%   cd('your/project/root');
%   initCSSCProjectPath;
%
% This function:
%   1) detects project root as the folder containing this file;
%   2) adds src/ and common folders to MATLAB path;
%   3) creates results/ folders if missing;
%   4) stores CSSC_PROJECT_ROOT in the base workspace.

    root = fileparts(mfilename('fullpath'));

    addpath(root);

    addIfExists(fullfile(root, 'src'));
    addIfExists(fullfile(root, 'scripts'));
    addIfExists(fullfile(root, 'examples'));
    addIfExists(fullfile(root, 'utils'));
    addIfExists(fullfile(root, 'tests'));

    makeIfMissing(fullfile(root, 'results'));

    assignin('base', 'CSSC_PROJECT_ROOT', root);

    fprintf('[CSSC] Project root initialized:\n');
    fprintf('       %s\n', root);
    fprintf('[CSSC] Added source folders and created results folders.\n');
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
