function [path, info] = planCSSCFrontend2D( ...
        startPt, goalPt, obstacles, opts)
%PLANCSSCFRONTEND2D Select the point-path frontend used by CSSC.
%
% opts.method:
%   'rrt' (default)        : single-tree RRT.
%   'rrtconnect'           : bidirectional RRT-Connect.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'method') || isempty(opts.method)
        opts.method = 'rrt';
    end

    method = regexprep(lower(char(opts.method)), '[^a-z0-9*]', '');
    switch method
        case 'rrt'
            [path, info] = planRRT2D( ...
                startPt, goalPt, obstacles, opts);
            info.frontendMethod = 'rrt';

        case {'rrtconnect','connect'}
            [path, info] = planRRTConnect2D( ...
                startPt, goalPt, obstacles, opts);
            info.frontendMethod = 'rrtconnect';

        otherwise
            error('Unknown CSSC frontend method: %s', char(opts.method));
    end
end
