function [Pinit, splineInfo] = polylineToBSplineInit2D(path, opts)
%POLYLINETOBSPLINEINIT2D Convert a polyline to B-spline control points.
%
% This first-stage initializer uses arc-length resampled polyline points as
% control points. It is meant to provide a reasonable initial path for the
% downstream CSSC optimizer rather than an exact spline fitting routine.
%
% Key opts: nCtrl, degree.

    if nargin < 2 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'degree'); opts.degree = 3; end

    if ~isfield(opts, 'nCtrl') || isempty(opts.nCtrl)
        approxLen = polylineLength(path);
        opts.nCtrl = max(opts.degree + 1, ceil(approxLen / 0.06) + 1);
    end

    Pinit = resamplePolyline2D(path, opts.nCtrl);
    Pinit(1,:) = path(1,:);
    Pinit(end,:) = path(end,:);

    splineInfo = struct();
    splineInfo.degree = opts.degree;
    splineInfo.nCtrl = size(Pinit, 1);
    splineInfo.knot = makeClampedUniformKnot(splineInfo.nCtrl, opts.degree);
    splineInfo.controlPolygonLength = polylineLength(Pinit);
end

function len = polylineLength(path)
    if size(path, 1) < 2
        len = 0;
    else
        len = sum(vecnorm(diff(path, 1, 1), 2, 2));
    end
end
