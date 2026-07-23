function handles = plotFTLRobot2D(ax, joints, robot, opts)
%PLOTFTLROBOT2D Draw a planar equal-link robot as rectangular bodies.
%
% Each link body is shorter than the link length. It leaves a gap equal to
% jointRadius at hinge-connected ends. The distal end of the last link does
% not leave a gap by default because there is no following hinge.

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'showJointLabels'); opts.showJointLabels = false; end

    hold(ax, 'on');
    handles = struct();
    handles.links = gobjects(robot.numLinks, 1);
    handles.joints = gobjects(robot.numLinks + double(robot.drawTipJoint), 1);

    for i = 1:robot.numLinks
        p0 = joints(i,:);
        p1 = joints(i+1,:);
        handles.links(i) = drawLinkBody(ax, p0, p1, i, robot);
    end

    nJointCircles = robot.numLinks;
    if robot.drawTipJoint
        nJointCircles = robot.numLinks + 1;
    end

    for i = 1:nJointCircles
        if i == robot.numJoints
            color = robot.tipColor;
        else
            color = robot.jointColor;
        end
        handles.joints(i) = drawCirclePatch(ax, joints(i,:), robot.jointRadius, ...
            color, robot.jointEdgeColor);
        if opts.showJointLabels
            text(ax, joints(i,1), joints(i,2), sprintf('%d', i), ...
                'HorizontalAlignment','center', 'FontSize',7, 'Color','k');
        end
    end
end

function h = drawLinkBody(ax, p0, p1, idx, robot)
    e = p1 - p0;
    len = norm(e);
    if len < 1e-12
        h = gobjects(1);
        return;
    end

    dir = e / len;
    normal = [-dir(2), dir(1)];

    startGap = robot.jointRadius;
    if idx < robot.numLinks
        endGap = robot.jointRadius;
    else
        endGap = 0;
    end

    a = p0 + min(startGap, 0.45*len) * dir;
    b = p1 - min(endGap, 0.45*len) * dir;
    halfW = 0.5 * robot.bodyWidth;

    V = [
        a + halfW * normal
        b + halfW * normal
        b - halfW * normal
        a - halfW * normal
    ];

    h = patch(ax, V(:,1), V(:,2), robot.bodyColor, ...
        'FaceAlpha', robot.bodyFaceAlpha, ...
        'EdgeColor', robot.edgeColor, ...
        'LineWidth', 0.8);
end

function h = drawCirclePatch(ax, center, radius, faceColor, edgeColor)
    th = linspace(0, 2*pi, 40);
    x = center(1) + radius * cos(th);
    y = center(2) + radius * sin(th);
    h = patch(ax, x, y, faceColor, ...
        'FaceAlpha', 0.95, ...
        'EdgeColor', edgeColor, ...
        'LineWidth', 0.8);
end
