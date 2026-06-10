function knot = makeClampedUniformKnot(nCtrl, degree)
%MAKECLAMPEDUNIFORMKNOT Create a clamped uniform knot vector on [0,1].
%
% nCtrl  : number of B-spline control points
% degree : spline degree, e.g. 3 for cubic
%
% The knot vector length is nCtrl + degree + 1.

    if nCtrl < degree + 1
        error('nCtrl must be at least degree + 1.');
    end

    p = degree;
    n = nCtrl - 1;
    numInterior = n - p;

    if numInterior > 0
        tmp = linspace(0, 1, numInterior + 2);
        interior = tmp(2:end-1);
    else
        interior = [];
    end

    knot = [zeros(1, p+1), interior, ones(1, p+1)];
end
