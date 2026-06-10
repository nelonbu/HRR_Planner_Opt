function dB = bsplineBasisDerivVector(u, nCtrl, degree, knot)
%BSPLINEBASISDERIVEVECTOR Return derivative of all basis functions at u.

    dB = zeros(1, nCtrl);
    u = max(0, min(1, u));

    if degree == 0
        return;
    end

    p = degree;

    for i = 1:nCtrl
        denom1 = knot(i+p) - knot(i);
        denom2 = knot(i+p+1) - knot(i+1);

        term1 = 0;
        term2 = 0;

        if abs(denom1) > 1e-14
            term1 = p / denom1 * bsplineBasis(i, p-1, u, knot, nCtrl);
        end

        if abs(denom2) > 1e-14
            term2 = p / denom2 * bsplineBasis(i+1, p-1, u, knot, nCtrl);
        end

        dB(i) = term1 - term2;
    end
end
