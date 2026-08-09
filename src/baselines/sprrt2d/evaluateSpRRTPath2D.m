function metrics = evaluateSpRRTPath2D(path, obstacles, opts)
%EVALUATESPRRTPATH2D Evaluate an Adaptive Sp-RRT polyline without smoothing.
%
% The path remains a native polyline. Fixed-chord endpoints are obtained by
% exact segment-circle intersections, so no cubic smoothing or degree-1
% Newton solve changes the baseline output geometry.

    if nargin < 3
        opts = struct();
    end
    metrics = evaluatePolylineHighPrecision2D(path, obstacles, opts);
end
