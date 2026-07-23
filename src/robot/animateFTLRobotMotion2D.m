function out = animateFTLRobotMotion2D(motion, robot, obstacles, opts)
%ANIMATEFTLROBOTMOTION2D Render FTL robot motion as frames and optional MP4.
%
% Key opts:
%   outputDir, saveVideo, saveFramesImage, visible, fps, frameStride

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = setDefaults(opts);

    if ~isempty(opts.outputDir) && ~exist(opts.outputDir, 'dir')
        mkdir(opts.outputDir);
    end

    nStep = size(motion.joints, 1);
    frameIdx = 1:opts.frameStride:nStep;
    if frameIdx(end) ~= nStep
        frameIdx(end+1) = nStep; %#ok<AGROW>
    end

    fig = figure('Color','w', 'Name','FTL robot motion', ...
        'Visible', opts.visible, 'Position',[100 100 960 620]);
    ax = axes(fig);

    writer = [];
    if opts.saveVideo && ~isempty(opts.outputDir)
        videoPath = fullfile(opts.outputDir, 'ftl_robot_motion.mp4');
        writer = VideoWriter(videoPath, 'MPEG-4');
        writer.FrameRate = opts.fps;
        writer.Quality = opts.videoQuality;
        open(writer);
    else
        videoPath = "";
    end

    cleanupObj = onCleanup(@() closeWriter(writer)); %#ok<NASGU>

    for kk = 1:numel(frameIdx)
        it = frameIdx(kk);
        cla(ax);
        hold(ax, 'on');
        axis(ax, 'equal');
        grid(ax, 'on');
        xlabel(ax, 'x');
        ylabel(ax, 'y');
        title(ax, sprintf('FTL robot following | frame %d / %d', it, nStep));

        drawObstaclesLocal(ax, obstacles);
        plot(ax, motion.pathSample(:,1), motion.pathSample(:,2), 'k--', ...
            'LineWidth', 1.1, 'DisplayName','B-spline path');
        J = squeeze(motion.joints(it,:,:));
        plotFTLRobot2D(ax, J, robot);
        plot(ax, motion.startPt(1), motion.startPt(2), 'go', ...
            'MarkerFaceColor','g', 'MarkerSize',6);
        plot(ax, motion.goalPt(1), motion.goalPt(2), 'bs', ...
            'MarkerFaceColor','b', 'MarkerSize',6);

        applyMotionAxes(ax, motion, obstacles, robot);
        drawnow;

        if ~isempty(writer)
            writeVideo(writer, getframe(fig));
        end
    end

    if ~isempty(writer)
        close(writer);
        writer = [];
    end

    framesPath = "";
    if opts.saveFramesImage && ~isempty(opts.outputDir)
        framesPath = fullfile(opts.outputDir, 'ftl_robot_frames.jpg');
        exportFrameMontage(motion, robot, obstacles, opts, framesPath);
    end

    out = struct();
    out.figure = fig;
    out.videoPath = videoPath;
    out.framesPath = string(framesPath);
end

function opts = setDefaults(opts)
    if ~isfield(opts, 'outputDir'); opts.outputDir = ''; end
    if ~isfield(opts, 'saveVideo'); opts.saveVideo = true; end
    if ~isfield(opts, 'saveFramesImage'); opts.saveFramesImage = true; end
    if ~isfield(opts, 'visible'); opts.visible = 'on'; end
    if ~isfield(opts, 'fps'); opts.fps = 12; end
    if ~isfield(opts, 'videoQuality'); opts.videoQuality = 95; end
    if ~isfield(opts, 'frameStride'); opts.frameStride = 2; end
    if ~isfield(opts, 'numMontageFrames'); opts.numMontageFrames = 8; end
end

function exportFrameMontage(motion, robot, obstacles, opts, outPath)
    nStep = size(motion.joints, 1);
    ids = unique(round(linspace(1, nStep, opts.numMontageFrames)));

    fig = figure('Color','w', 'Name','FTL robot frames', ...
        'Visible', opts.visible, 'Position',[100 100 1200 420]);
    tiledlayout(fig, 2, ceil(numel(ids)/2), 'Padding','compact', 'TileSpacing','compact');

    for k = 1:numel(ids)
        ax = nexttile;
        hold(ax, 'on');
        axis(ax, 'equal');
        axis(ax, 'off');
        drawObstaclesLocal(ax, obstacles);
        plot(ax, motion.pathSample(:,1), motion.pathSample(:,2), 'k--', 'LineWidth', 0.8);
        J = squeeze(motion.joints(ids(k),:,:));
        plotFTLRobot2D(ax, J, robot);
        title(ax, sprintf('%d/%d', ids(k), nStep), 'FontSize',8);
        applyMotionAxes(ax, motion, obstacles, robot);
    end

    exportgraphics(fig, outPath, 'Resolution', 240, 'BackgroundColor','white');
    close(fig);
end

function applyMotionAxes(ax, motion, obstacles, robot)
    pts = motion.pathSample;
    Jall = reshape(motion.joints, [], 2);
    pts = [pts; Jall]; %#ok<AGROW>
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(obs.type)
            case 'circle'
                r = obs.radius + robot.linkLength * 0.2;
                pts = [pts; obs.center + [-r, -r]; obs.center + [r, r]]; %#ok<AGROW>
            case {'rect','rectangle','box'}
                h = obs.halfSize + robot.linkLength * 0.2;
                pts = [pts; obs.center - h; obs.center + h]; %#ok<AGROW>
            case 'polygon'
                pts = [pts; obs.vertices]; %#ok<AGROW>
        end
    end
    lo = min(pts, [], 1);
    hi = max(pts, [], 1);
    span = max(hi - lo);
    pad = max(0.05, 0.08 * span);
    xlim(ax, [lo(1)-pad, hi(1)+pad]);
    ylim(ax, [lo(2)-pad, hi(2)+pad]);
end

function drawObstaclesLocal(ax, obstacles)
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        switch lower(obs.type)
            case 'circle'
                th = linspace(0, 2*pi, 80);
                x = obs.center(1) + obs.radius*cos(th);
                y = obs.center(2) + obs.radius*sin(th);
                patch(ax, x, y, [0.80, 0.82, 0.84], ...
                    'FaceAlpha',0.75, 'EdgeColor',[0.15,0.15,0.15]);
            case {'rect','rectangle','box'}
                drawRect(ax, obs.center, obs.halfSize, obs.yaw);
            case 'polygon'
                V = [obs.vertices; obs.vertices(1,:)];
                patch(ax, V(:,1), V(:,2), [0.80, 0.82, 0.84], ...
                    'FaceAlpha',0.75, 'EdgeColor',[0.15,0.15,0.15]);
        end
    end
end

function drawRect(ax, c, h, yaw)
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    V = [
        -h(1), -h(2)
         h(1), -h(2)
         h(1),  h(2)
        -h(1),  h(2)
    ];
    V = V * R.' + c;
    patch(ax, V(:,1), V(:,2), [0.80, 0.82, 0.84], ...
        'FaceAlpha',0.75, 'EdgeColor',[0.15,0.15,0.15]);
end

function closeWriter(writer)
    if ~isempty(writer)
        try
            close(writer);
        catch
        end
    end
end
