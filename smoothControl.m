function [rr, w_des] = smoothControl(V,currentPos,targetPos)
% Executes algorithm from Park and Kuipers for Smooth Control
% Kinematic control law:
% rr = - (V/range) * ...
%  ( k2 * ( del - atan( -k1*theta_pose ) ) + ...
%   sin( del ) * ( 1. + ( k1 / ( 1 + (k1*theta_pose).^2 ) ) )
% )
%  k1 ~ 1 and k2 ~ 3 or so ...
% On input:
%  V: current robot forward speed (fps)
%  currentPos: 3-element vector of x, y, angle of robot on field
%  targetPos: x,y,psi value of location on a 2D field, with
%    psi the angle of orientation desired for the robot when
%    reaching that x,y position.
% On output:
%  Controller commands perturbation in left/right drive
%  controls to command desired angular rate.  Collective
%  velocity is (currently) untouched.
    % CONSTANTS --------------
    % wheelbase is distance between center of left and right wheels
    %wheelbase = 25. / 12.;
    % K-values from paper, but can be adjusted
    k2 = 3;
    k1 = 1;
    % EXECUTABLE CODE --------
    vv = V; % fps
    robotHeading = currentPos(3)*pi/180.; % now in radians
    % Check if we are going backwards!
    if V < 0
        vv = -V;
        robotHeading = robotHeading + pi;
        robotHeading = limitAngle( robotHeading );
    end
    % Compute range to target using differences in x, y field positions
    dx = targetPos(1) - currentPos(1);
    dy = targetPos(2) - currentPos(2);
    rr = sqrt( dx * dx + dy * dy );
    % Range angle is angle in field reference defining vector from robot to target
    r_angle = atan2( dy, dx ); % radians
    % Theta_T is from algorithm, which is relative angle of target
    %  orientation from range vector.
    thetaT = (targetPos(3)/57.3) - r_angle; % radians
    % This routine simply keeps this angle bounded between +/-pi
    thetaT = limitAngle( thetaT );
    % del_r is angle between current robot orientation and range vector
    del_r = robotHeading - r_angle; % radians
    del_r = limitAngle( del_r );
    % Now the desired angular rate:
    w_des = -(vv/rr) * ( ...
        k2 * ( del_r - atan( -k1 * thetaT ) ) + ...
        sin( del_r ) * ( 1 + ( k1 / ...
          ( 1 + (k1 * thetaT).^2 ) ) ) );
    % So we want to keep current speed while commanding this
    %   angular rate.
%     vLeft  = vv + (wheelbase/2.)*w_des;
%     vRight = vv - (wheelbase/2.)*w_des;
end

function oldAngle = limitAngle(oldAngle)
    % bounds angle between +/-pi
    while (oldAngle > pi )
        oldAngle = oldAngle - 2*pi;
    end
    while (oldAngle < -pi )
        oldAngle = oldAngle + 2*pi;
    end
end

