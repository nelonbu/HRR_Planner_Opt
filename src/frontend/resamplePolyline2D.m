function pathOut = resamplePolyline2D(pathIn, nSample)
%RESAMPLEPOLYLINE2D Resample a 2D polyline by arc length.

    if nSample < 2
        error('nSample must be at least 2.');
    end

    if size(pathIn, 1) == 1
        pathOut = repmat(pathIn, nSample, 1);
        return;
    end

    segLen = vecnorm(diff(pathIn, 1, 1), 2, 2);
    s = [0; cumsum(segLen)];
    totalLen = s(end);

    if totalLen < 1e-12
        pathOut = repmat(pathIn(1,:), nSample, 1);
        return;
    end

    [sUnique, idxUnique] = unique(s, 'stable');
    pathUnique = pathIn(idxUnique,:);

    sQuery = linspace(0, totalLen, nSample).';
    x = interp1(sUnique, pathUnique(:,1), sQuery, 'linear');
    y = interp1(sUnique, pathUnique(:,2), sQuery, 'linear');
    pathOut = [x, y];

    pathOut(1,:) = pathIn(1,:);
    pathOut(end,:) = pathIn(end,:);
end
