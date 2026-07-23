clear; clc; close all;

%% Debug S-channel seeds: 1555 to 1570

try
    initCSSCProjectPath;
catch
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

seeds = 1585:1600;

opts = struct();
opts.bounds = [0, 1; -0.4, 0.4];
opts.startPt = [0.05, 0.0];
opts.goalPt = [0.95, 0.0];
opts.dGap = 0.10;
opts.W = 0.64;
opts.H = opts.bounds(2,2) - opts.bounds(2,1);
opts.yInOutRange = [-0.20, 0.20];

figure('Color','w', 'Name','S-channel seed grid', 'Position',[80 80 1100 760]);
tiledlayout(4, 4, 'Padding','compact', 'TileSpacing','compact');

for k = 1:numel(seeds)
    opts.seed = seeds(k);
    [obstacles, envInfo] = generateCSSCStructuredEnvironment2D('fourRectSChannel', opts);

    nexttile;
    hold on; axis equal; axis off;
    drawBounds(opts.bounds);
    drawObstacles(obstacles);
    plot(opts.startPt(1), opts.startPt(2), 'go', 'MarkerFaceColor','g', 'MarkerSize', 5);
    plot(opts.goalPt(1), opts.goalPt(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize', 6);
    xlim(opts.bounds(1,:));
    ylim(opts.bounds(2,:));
    title(sprintf('seed %d | L2 %.3f', seeds(k), envInfo.params.L2), ...
        'FontSize', 8, 'Interpreter','none');
end

function drawBounds(bounds)
    x = bounds(1,:);
    y = bounds(2,:);
    plot([x(1), x(2), x(2), x(1), x(1)], ...
         [y(1), y(1), y(2), y(2), y(1)], ...
         'k-', 'LineWidth', 0.8);
end

function drawObstacles(obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(obs.type)
            case 'circle'
                th = linspace(0, 2*pi, 80);
                fill(obs.center(1) + obs.radius*cos(th), ...
                     obs.center(2) + obs.radius*sin(th), ...
                     [0.75 0.77 0.80], ...
                     'EdgeColor',[0.15 0.15 0.15], ...
                     'LineWidth',0.8);

            case {'rect','rectangle','box'}
                drawRect(obs.center, obs.halfSize, obs.yaw);
        end
    end
end

function drawRect(c, h, yaw)
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    pts = [
        -h(1), -h(2)
         h(1), -h(2)
         h(1),  h(2)
        -h(1),  h(2)
        -h(1), -h(2)
    ];
    pts = (R * pts.').' + c;
    fill(pts(:,1), pts(:,2), [0.75 0.77 0.80], ...
        'EdgeColor',[0.15 0.15 0.15], 'LineWidth',0.8);
end
