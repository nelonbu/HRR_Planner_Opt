function [r, dr, d2r] = evalBSplinePath2D(Pctrl, w, opts)
%EVALBSPLINEPATH2D Evaluate a clamped uniform cubic B-spline path in 2D.
%
% Usage:
%   r = evalBSplinePath2D(Pctrl, w, opts)
%   [r, dr, d2r] = evalBSplinePath2D(Pctrl, w, opts)
%
% Inputs:
%   Pctrl : nCtrl-by-2 control points
%   w     : scalar or vector parameter in [0,1]
%   opts  : optional struct
%           opts.degree = 3 by default
%
% Outputs:
%   r     : numel(w)-by-2 path points
%   dr    : first derivative with respect to w
%   d2r   : second derivative with respect to w
%
% Notes:
%   This file does not require MATLAB toolboxes. It implements an open
%   clamped uniform B-spline with Cox-de Boor basis recursion.

    if nargin < 3
        opts = struct();
    end
    if ~isfield(opts, 'degree')
        opts.degree = 3;
    end

    p = opts.degree;
    nCtrl = size(Pctrl, 1);
    if size(Pctrl, 2) ~= 2
        error('Pctrl must be nCtrl-by-2.');
    end
    if nCtrl < p + 1
        error('Need at least degree+1 control points.');
    end

    w = w(:);
    w = min(1, max(0, w));

    U = makeClampedUniformKnot(nCtrl, p);
    r = evalBSplineGeneric(Pctrl, p, U, w);

    if nargout >= 2
        [P1, U1, p1] = derivativeCurve(Pctrl, U, p);
        dr = evalBSplineGeneric(P1, p1, U1, w);
    end

    if nargout >= 3
        [P2, U2, p2] = derivativeCurve(P1, U1, p1);
        d2r = evalBSplineGeneric(P2, p2, U2, w);
    end
end

%% ============================================================
% Local helpers
%% ============================================================

function U = makeClampedUniformKnot(nCtrl, p)
    nInternal = nCtrl - p - 1;
    if nInternal > 0
        internal = (1:nInternal) / (nInternal + 1);
    else
        internal = [];
    end
    U = [zeros(1, p + 1), internal, ones(1, p + 1)];
end

function C = evalBSplineGeneric(P, p, U, w)
    nCtrl = size(P, 1);
    C = zeros(numel(w), size(P, 2));

    for k = 1:numel(w)
        x = w(k);
        val = zeros(1, size(P, 2));
        for i = 1:nCtrl
            B = bsplineBasis(i, p, U, x, nCtrl);
            val = val + B * P(i, :);
        end
        C(k, :) = val;
    end
end

function B = bsplineBasis(i, p, U, x, nCtrl)
    if p == 0
        if (U(i) <= x && x < U(i + 1)) || (x == U(end) && i == nCtrl)
            B = 1.0;
        else
            B = 0.0;
        end
        return;
    end

    B = 0.0;

    denom1 = U(i + p) - U(i);
    if denom1 > 0
        B = B + (x - U(i)) / denom1 * bsplineBasis(i, p - 1, U, x, nCtrl);
    end

    denom2 = U(i + p + 1) - U(i + 1);
    if denom2 > 0
        B = B + (U(i + p + 1) - x) / denom2 * bsplineBasis(i + 1, p - 1, U, x, nCtrl);
    end
end

function [Pd, Ud, pd] = derivativeCurve(P, U, p)
    if p <= 0
        Pd = zeros(1, size(P, 2));
        Ud = [0 1];
        pd = 0;
        return;
    end

    nCtrl = size(P, 1);
    Pd = zeros(nCtrl - 1, size(P, 2));
    for i = 1:nCtrl - 1
        denom = U(i + p + 1) - U(i + 1);
        if denom <= 0
            Pd(i, :) = 0;
        else
            Pd(i, :) = p * (P(i + 1, :) - P(i, :)) / denom;
        end
    end

    Ud = U(2:end-1);
    pd = p - 1;
end
