function robot = makeFTLRobot2D(linkLength, numLinks, opts)
%MAKEFTLROBOT2D Create a planar equal-link FTL robot model.
%
% The robot is represented as numLinks rigid links connected by hinges.
% Link length should match the CSSC fixed chord length L. Visualization
% draws each link as a rectangle and each internal hinge as a circle.
%
% Key opts:
%   bodyWidth, jointRadius, bodyColor, edgeColor, jointColor

    if nargin < 3 || isempty(opts)
        opts = struct();
    end

    robot = struct();
    robot.linkLength = linkLength;
    robot.numLinks = numLinks;
    robot.numJoints = numLinks + 1;

    robot.bodyWidth = getOpt(opts, 'bodyWidth', 0.045 * linkLength / 0.15);
    robot.jointRadius = getOpt(opts, 'jointRadius', 0.020 * linkLength / 0.15);
    robot.bodyColor = getOpt(opts, 'bodyColor', [0.25, 0.45, 0.88]);
    robot.bodyFaceAlpha = getOpt(opts, 'bodyFaceAlpha', 0.68);
    robot.edgeColor = getOpt(opts, 'edgeColor', [0.04, 0.08, 0.16]);
    robot.jointColor = getOpt(opts, 'jointColor', [0.96, 0.72, 0.16]);
    robot.jointEdgeColor = getOpt(opts, 'jointEdgeColor', [0.08, 0.08, 0.08]);
    robot.tipColor = getOpt(opts, 'tipColor', [0.05, 0.70, 0.30]);
    robot.drawTipJoint = getOpt(opts, 'drawTipJoint', false);
end

function val = getOpt(opts, name, defaultVal)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = defaultVal;
    end
end
