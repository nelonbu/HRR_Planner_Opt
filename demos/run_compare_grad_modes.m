%RUN_COMPARE_GRAD_MODES Optional quick comparison of finite-diff and semi-analytic modes.
clear; clc; close all;
initCSSCProjectPath;

% Build the same demo problem by executing the main demo setup manually.
obstacles = [
    obsCircle2D([2.2,  0.42], 0.38)
    obsCircle2D([3.2, -0.45], 0.42)
    obsRect2D(  [4.3,  0.48], [0.35, 0.28], 0.15)
    obsCircle2D([5.3, -0.45], 0.40)
    obsRect2D(  [6.4,  0.00], [0.35, 0.30], -0.2)
];

params = struct();
params.L = 0.90;
params.dMin = 0.03;
params.dPref = 0.22;
params.degree = 3;
startPt = [0 0]; goalPt = [8 0];
hCtrl = 0.75 * params.L;
nCtrl = max(params.degree + 1, ceil(norm(goalPt-startPt) / hCtrl) + 1);
x = linspace(startPt(1), goalPt(1), nCtrl).';
y = 0.12 * sin(2*pi*(x-x(1))/(x(end)-x(1)));
Pinit = [x y]; Pinit(1,:)=startPt; Pinit(end,:)=goalPt;
Pref = Pinit;
params.knot = makeClampedUniformKnot(size(Pinit,1), params.degree);
params.envOpts = struct('uRange',[0 1],'vSearchRange',[0 1],'nU',100,'nVGrid',120,'epsV',1e-8,'tolDen',1e-10);
params.wObs=600; params.wClear=1; params.wRef=0.01; params.wSmooth=0.5; params.wLength=0.02; params.wTrust=0;
params.activeTopK=20; params.activeClearanceMargin=0.06;
params.numIter=20; params.lr=0.008; params.fdStep=1e-4; params.gradClip=30; params.printInterval=5; params.saveInterval=10;
params.enableTimingDebug=true; params.timingPrintInterval=10; params.timingPrintWindow=5;

paramsSemi = params; paramsSemi.solver.gradMode = 'semi-analytic';
paramsFD = params; paramsFD.solver.gradMode = 'finite-diff'; paramsFD.numIter = 10;

fprintf('\n=== Semi-analytic ===\n');
[Psemi, infoSemi] = optimizeCSSC2D(Pinit, Pref, obstacles, paramsSemi);

fprintf('\n=== Finite difference ===\n');
[Pfd, infoFD] = optimizeCSSC2D(Pinit, Pref, obstacles, paramsFD);

figure('Color','w'); hold on; grid on;
plot(infoSemi.minClearHist, 'LineWidth', 1.8, 'DisplayName','semi minClear');
plot(infoFD.minClearHist, 'LineWidth', 1.8, 'DisplayName','FD minClear');
yline(params.dMin, 'r--', 'DisplayName','dMin');
legend; xlabel('iter'); ylabel('minClear'); title('Gradient mode comparison');

plotCSSCResult2D(Pinit, Psemi, obstacles, paramsSemi, infoSemi);
plotCSSCResult2D(Pinit, Pfd, obstacles, paramsFD, infoFD);
