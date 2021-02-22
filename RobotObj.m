% RobotObj
% Robot class that provides a sprite for moving on the field and storage of
% its own "pose", or x,y,heading value.  Also includes current linear and
% angular velocity.
classdef RobotObj < handle
    % Inherits from handle to provide indirect references.
    
    properties
        pose = [0, 0, 0];  % x (ft), y (ft), and heading (deg)
        vv   = 0;          % velociity (fps)
        ahandle            % axes handle for drawing sprite
        hRobot             % patch handle for robot sprite
        hXform             % handle for hgtransform containing patch
    end
    
    methods
        %-------------------------------------
        function obj = RobotObj(ah)
            % Creator class stores location of graphics axes
            obj.ahandle = ah;
            % Geometry is a scaled box
            width = 37 / 12.;
            length = 35 /12.;
            height = 2.;
            % Cube coordinates
            theRobot.Vertices = [ ...
                1 1 1; ...
                1 -1 1; ...
                1 -1 -1; ...
                1 1 -1; ...
                -1 1 1; ...
                -1 -1 1; ...
                -1 -1 -1; ...
                -1 1 -1];
            % Offset vertical to all lie in -z direction
            theRobot.Vertices(:,3) = theRobot.Vertices(:,3) + 1.;
            % Connectivity
            theRobot.Faces = [ ...
                1 2 3; ...
                3 4 1; ...
                1 5 6; ...
                6 2 1; ...
                5 1 4; ...
                4 8 5; ...
                4 3 7; ...
                7 8 4; ...
                2 6 3; ...
                6 7 3; ...
                8 7 6; ...
                5 8 6 ];
            % Scale appropriately
            theRobot.Vertices = theRobot.Vertices * diag([length, width, height]/2.);
            % Blue Alliance!
            cv = [.3 .3 .9];
            theRobot.FaceColor = cv;
            theRobot.EdgeColor = cv.^2;
            % Create a transform container to hold this geometry
            obj.hXform = hgtransform('Parent',obj.ahandle);
            % Create a patch
            obj.hRobot = patch(theRobot,'Parent',obj.hXform,'Visible','off');
        end
        
        function showGeom(obj)
            obj.hRobot.Visible = 'on';
            Rz = makehgtform('zrotate', obj.pose(3)*pi/180);
            Sxy = makehgtform('translate', [obj.pose(1), obj.pose(2), 0]);
            obj.hXform.Matrix = Sxy*Rz;
        end            
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

