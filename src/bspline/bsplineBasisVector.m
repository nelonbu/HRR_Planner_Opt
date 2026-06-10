function B = bsplineBasisVector(u, nCtrl, degree, knot)
%BSPLINEBASISVECTOR Return all B-spline basis values at parameter u.

    B = zeros(1, nCtrl);
    u = max(0, min(1, u));

    for i = 1:nCtrl
        B(i) = bsplineBasis(i, degree, u, knot, nCtrl);
    end
end
