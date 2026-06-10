function [pos, dpos, ddpos, basis, dbasis] = evalBSplinePath2D(P, w, degree, knot)
%EVALBSPLINEPATH2D Evaluate a 2D B-spline path and derivatives.
%
% Inputs:
%   P      : nCtrl-by-2 control points
%   w      : scalar or vector parameters in [0,1]
%   degree : B-spline degree
%   knot   : knot vector. If omitted, clamped uniform knots are used.
%
% Outputs:
%   pos    : numel(w)-by-2 path points
%   dpos   : first derivative wrt w
%   ddpos  : second derivative wrt w, computed by finite difference of basis derivative
%   basis  : numel(w)-by-nCtrl basis values
%   dbasis : numel(w)-by-nCtrl basis derivatives

    if nargin < 4 || isempty(knot)
        knot = makeClampedUniformKnot(size(P,1), degree);
    end

    w = w(:);
    nCtrl = size(P,1);
    nW = numel(w);

    basis = zeros(nW, nCtrl);
    dbasis = zeros(nW, nCtrl);

    for k = 1:nW
        basis(k,:) = bsplineBasisVector(w(k), nCtrl, degree, knot);
        dbasis(k,:) = bsplineBasisDerivVector(w(k), nCtrl, degree, knot);
    end

    pos = basis * P;
    dpos = dbasis * P;

    if nargout >= 3
        % Second derivative by differentiating dpos over w. This is only used
        % for diagnostic curvature costs/plots. For objective gradients, the
        % default demo keeps curvature disabled.
        if nW >= 3
            ddpos = gradient(dpos, w);
        else
            ddpos = zeros(size(dpos));
        end
    end
end
