function env = fixedChordEnvelope_Vectorized(rFun, drFun, L, opts)
%FIXEDCHORDENVELOPE_VECTORIZED Vectorized fixed-length chord envelope.
%
% Fast version for small-medium B-spline curves (e.g., 13 control points)
% and moderate nU (~150). v(u) is predicted linearly: v = u + L / ||r'(u)||

    opts = setDefaultOpts(opts);

    % -------------------------------------------------------------
    % 1. u samples
    % -------------------------------------------------------------
    uList = linspace(opts.uRange(1), opts.uRange(2), opts.nU).';
    nU = numel(uList);

    % -------------------------------------------------------------
    % 2. Evaluate B-spline r(u) and derivative dr/du for all u
    % -------------------------------------------------------------
    MList  = rFun(uList);      % nU x 2
    ruList = drFun(uList);     % nU x 2

    % -------------------------------------------------------------
    % 3. Predict v(u) with linear approximation
    % v = u + L / ||r'(u)||
    % -------------------------------------------------------------
    ruNorm = sqrt(sum(ruList.^2,2));
    ruNorm(ruNorm<opts.minSpeed) = opts.minSpeed; % avoid division by zero
    vList = uList + L ./ ruNorm;
    vList = min(vList, opts.vSearchRange(2));

    % -------------------------------------------------------------
    % 4. Evaluate N = r(v)
    % -------------------------------------------------------------
    NList = rFun(vList);        % nU x 2
    rvList = drFun(vList);      % nU x 2

    % -------------------------------------------------------------
    % 5. Compute segment vector d = N - M, residuals, vp, dp
    % -------------------------------------------------------------
    d = NList - MList;                 % nU x 2
    residualList = sqrt(sum(d.^2,2)) - L;

    denomVp = sum(d .* rvList, 2);
    denomVp(abs(denomVp)<opts.tolDen) = NaN;
    vpList = sum(d .* ruList,2) ./ denomVp;
    dpList = rvList .* vpList - ruList;

    % -------------------------------------------------------------
    % 6. Compute lambda and G
    % lambda = cross2(ru,d)/cross2(d,dp)
    % -------------------------------------------------------------
    cross = @(a,b) a(:,1).*b(:,2)-a(:,2).*b(:,1);
    den = cross(d, dpList);
    den(abs(den)<opts.tolDen) = NaN;

    lambdaList = cross(ruList,d) ./ den;
    GList = MList + lambdaList .* d;

    % -------------------------------------------------------------
    % 7. Valid flags
    % -------------------------------------------------------------
    validLine = ~isnan(vpList);
    validSegment = lambdaList >= -opts.lambdaTol & lambdaList <= 1+opts.lambdaTol;

    % -------------------------------------------------------------
    % 8. Pack output
    % -------------------------------------------------------------
    env = struct();
    env.u = uList;
    env.v = vList;
    env.vp = vpList;
    env.M = MList;
    env.N = NList;
    env.G = GList;
    env.lambda = lambdaList;
    env.validLine = validLine;
    env.validSegment = validSegment;
    env.residual = residualList;
    env.opts = opts;

    env.stats = struct();
    env.stats.numNewtonSuccess = sum(validLine);
    env.stats.numFallbackUsed = 0;
    env.stats.avgNewtonIters = 0; % linear prediction, no Newton
end

function opts = setDefaultOpts(opts)
    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts,'uRange'); opts.uRange = [0,1]; end
    if ~isfield(opts,'vSearchRange'); opts.vSearchRange = [0,1]; end
    if ~isfield(opts,'nU'); opts.nU = 150; end
    if ~isfield(opts,'tolDen'); opts.tolDen = 1e-10; end
    if ~isfield(opts,'lambdaTol'); opts.lambdaTol = 1e-9; end
    if ~isfield(opts,'minSpeed'); opts.minSpeed = 1e-8; end
end