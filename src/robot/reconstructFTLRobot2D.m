function joints = reconstructFTLRobot2D(q, rootPosition, robot)
%RECONSTRUCTFTLROBOT2D Rebuild planar equal-link robot from joint parameters.
%
% q is [base angle, relative hinge angles]. rootPosition is the tail joint.

    q = q(:).';
    if numel(q) ~= robot.numLinks
        error('q must have numLinks elements.');
    end

    rootPosition = rootPosition(:).';
    joints = zeros(robot.numJoints, 2);
    joints(1,:) = rootPosition;

    theta = zeros(1, robot.numLinks);
    theta(1) = q(1);
    for i = 2:robot.numLinks
        theta(i) = theta(i-1) + q(i);
    end

    for i = 1:robot.numLinks
        dir = [cos(theta(i)), sin(theta(i))];
        joints(i+1,:) = joints(i,:) + robot.linkLength * dir;
    end
end
