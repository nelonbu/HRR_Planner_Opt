function val = bsplineBasis(i, p, u, knot, nCtrl)
%BSPLINEBASIS Cox-de Boor B-spline basis function value.
%
% i is a 1-based control point index.

    if i < 1 || i > nCtrl
        val = 0;
        return;
    end

    u = max(0, min(1, u));

    % Endpoint convention for clamped splines.
    if abs(u - 1.0) < 1e-14
        val = double(i == nCtrl);
        return;
    end

    if p == 0
        val = double(knot(i) <= u && u < knot(i+1));
        return;
    end

    denom1 = knot(i+p) - knot(i);
    denom2 = knot(i+p+1) - knot(i+1);

    term1 = 0;
    term2 = 0;

    if abs(denom1) > 1e-14
        term1 = (u - knot(i)) / denom1 * bsplineBasis(i, p-1, u, knot, nCtrl);
    end

    if abs(denom2) > 1e-14
        term2 = (knot(i+p+1) - u) / denom2 * bsplineBasis(i+1, p-1, u, knot, nCtrl);
    end

    val = term1 + term2;
end
