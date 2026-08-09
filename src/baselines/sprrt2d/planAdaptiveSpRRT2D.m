function [pathOptimized, info] = ...
        planAdaptiveSpRRT2D(startPt, goalPt, obstacles, opts)
%PLANADAPTIVESPRRT2D Formal entry point for Adaptive Sp-RRT-2D.
%
% The implementation retains planSpRRT2D as a compatibility entry point.
% Tree depth counts fixed-length leader-path segments and grows naturally
% up to maxSegmentCount; it does not count physical robot links.

    if nargin < 4
        opts = struct();
    end
    [pathOptimized, info] = ...
        planSpRRT2D(startPt, goalPt, obstacles, opts);
end
