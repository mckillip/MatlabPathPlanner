classdef PathPlannerApp2_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        AutonomousCourseUIFigure    matlab.ui.Figure
        UITable                     matlab.ui.control.Table
        CourseSelectionButtonGroup  matlab.ui.container.ButtonGroup
        SlalomButton                matlab.ui.control.RadioButton
        BarrelRaceButton            matlab.ui.control.RadioButton
        BouncePathButton            matlab.ui.control.RadioButton
        SimulateButton              matlab.ui.control.Button
        SavetomyPathgifCheckBox     matlab.ui.control.CheckBox
        EStopButton                 matlab.ui.control.Button
        LapTimeEditFieldLabel       matlab.ui.control.Label
        LapTimeEditField            matlab.ui.control.NumericEditField
        GIFframeskipEditFieldLabel  matlab.ui.control.Label
        GIFframeskipEditField       matlab.ui.control.NumericEditField
        TabGroup                    matlab.ui.container.TabGroup
        PathTrajectoryTab           matlab.ui.container.Tab
        UIAxes                      matlab.ui.control.UIAxes
        YawratecmdTab               matlab.ui.container.Tab
        UIAxes2                     matlab.ui.control.UIAxes
        RadiusofCurvatureTab        matlab.ui.container.Tab
        UIAxes3                     matlab.ui.control.UIAxes
        AddleadingrowButton         matlab.ui.control.Button
        DeletepickedrowButton       matlab.ui.control.Button
        SavetomyPathcsvCheckBox     matlab.ui.control.CheckBox
    end


    % Properties for control of GUI behavior
    properties
        saveGIF                 % 0/1 flag for possibly saving to GIF
        gifName = {'slalomPath.gif','barrelPath.gif','bouncePath.gif'}
        pathNumber = 1          % 1,2,or 3 for three paths options
        eStopFlag  = 0          % Button sets to 1 to stop simulation
        Robot                   % Robot sprite object for moving
        numNodes = 0            % number of nodes in table
        lh                      % handles to markers on node points
        qh                      % handles to arrows on control points
        tableName = {'slalomPoints.txt', ...
                     'barrelPoints.txt', ...
                     'bouncePoints.txt'}
        lapTime   = 0.          % total time from start to end of path
        iskip     = 2           % frame skip for writing GIF image
        timeNow                 % current simulation time
        tVector                 % time vector from simulation
        omega_des               % commanded turn rate
        turnRadius              % desired turn radius (V/omega_des)
        curvature               % 1 / turn radius = omega_des / v
        pickedRow = 1           % user-selected row in table
        hTraj                   % handles to trajectory points
        saveCSV                 % flags for storing CSV of path
        fidCSV                  % file ID for storage of CSV path
    end
    
    methods (Access = private)
    
        function updateplot(app)
            % Delete old trajectory if we can
            try 
                delete( app.hTraj );
            catch ME
                size(ME);
            end
            % Get Table UI component data
            t = app.UITable.DisplayData;
            % Get number of specified nodes to track
            app.numNodes = length(t.X);
            
            % Redraw grid and markers
            % It was too hard to only redraw the markers as it
            % eliminated the background course map.
            switch app.pathNumber
                case 1
                    makeSlalomApp(app.UIAxes);                    
                case 2
                    makeBarrelRaceApp(app.UIAxes);
                case 3
                    makeBouncePathApp(app.UIAxes);
                otherwise
            end
            
            % Plot modified data
            try
                delete( app.lh );
                delete( app.qh );
            catch 
            end
            hold(app.UIAxes,'on');
            app.lh = line(app.UIAxes,t.X,t.Y,...
                'Color','red',...
                'LineStyle','none',...
                'Marker','o',...
                'LineWidth',2);
            app.qh = quiver(app.UIAxes, t.X, t.Y, ...
                cosd(t.H), sind(t.H), ...
                0.25,'LineWidth',2 );
            hold(app.UIAxes,'off');
            
            % Draw the robot at the first location
            app.Robot.pose=[t.X(1),t.Y(1),t.H(1)];
            app.Robot.showGeom();
            
        end
        
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Create robot sprite
            app.Robot = RobotObj(app.UIAxes);
            % Default to Slalom course
            courseSelection(app);

        end

        % Display data changed function: UITable
        function UITableDisplayDataChanged(app, event)
            % Update the plots when user sorts the columns of the table
            updateplot(app);
            % Save the updated table back to the file
            writetable(app.UITable.Data,app.tableName{app.pathNumber});
        end

        % Selection changed function: CourseSelectionButtonGroup
        function courseSelection(app, event)
            selectedButton = app.CourseSelectionButtonGroup.SelectedObject;
            switch selectedButton.Text
                case 'Slalom'
                    app.pathNumber = 1;
                   
                case 'BarrelRace'
                    app.pathNumber = 2;

                case 'BouncePath'
                    app.pathNumber = 3;

                otherwise
                    app.pathNumber = 1;

            end
            % Add data to the Table UI Component
            app.UITable.Data = readtable(app.tableName{app.pathNumber});
            % Plot the data
            updateplot(app);
        end

        % Button pushed function: SimulateButton
        function simulateRobot(app, event)
            app.eStopFlag = 0; % Don't stop before you start!
            
            t = app.UITable.DisplayData; % get table data
            app.numNodes = length(t.X);
            if app.numNodes <=0
                return
            end
            
            % Draw the robot at the first location
            app.Robot.pose=[t.X(1),t.Y(1),t.H(1)];
            app.Robot.showGeom();
            
            % Reset path markers and plot first one
            try
                delete( app.hTraj );
            catch ME
                size(ME);
            end
            app.hTraj(1) = line(app.UIAxes,t.X(1),t.Y(1),...
                'Marker','+','LineStyle','none');
            
            % Want a movie?
            if app.saveGIF
                gif('myPath.gif','frame',app.UIAxes);
            end
            
            % Want a CSV?
            if app.saveCSV
                % We make the output file name match the path
                fn = app.tableName{app.pathNumber};
                fn([14 15 16]) = ['c','s','v'];
                % open the file for saving
                app.fidCSV = fopen(fn,'w');
                fprintf(app.fidCSV,'%f, %f, %f, %f\n', ...
                    t.X(1),t.Y(1),t.H(1),t.V(1));
            end
            
            % Initialize counters and such
            range = inf;
            inode = 2; % pointer to next node in table
            i = 1;     % skip timer index for gif movie output
            % Pick a speed and a time step
            dt = 0.03333; % 30Hz, 2x default GIF playback speed
            
            % We have two time history plots: turn rate, and turn radius
            app.timeNow = 0.;
            app.tVector = nan;
            app.omega_des = nan;
            app.turnRadius = nan;
            app.curvature = nan;
            ph2 = plot(app.UIAxes2,app.tVector,app.omega_des);
%            ph3 = plot(app.UIAxes3,app.tVector,app.curvature);
            ph3 = semilogy(app.UIAxes3,app.tVector,app.turnRadius);
            
            
            % The loop: do each segment, and check for eSTOP
            while inode <= app.numNodes
                % Check for eStop
                if app.eStopFlag
                    break
                end
                % Get variables for computing angular rate command
                v = t.V(inode-1);
                targetPose = [t.X(inode),t.Y(inode),t.H(inode)];
                % Check range to see if we should go to next node
                if range > .4
                    % Yes, keep on truckin!
                    rPos = app.Robot.pose;
                    if v < 0
                        rPos(3) = rPos(3) + 180.; % the back is the front!
                        while rPos(3) > 180.
                            rPos(3) = rPos(3) - 360.;
                        end
                    end
                    % rPos is a possibly modified representation of the
                    % robot pose, as it has a 180deg flip if we are running
                    % in reverse.
                    [range,w_des] = smoothControl(abs(v),rPos,targetPose);
%                    disp([range, w_des]);
                else
                    % We are close enough to switch, so:
                    % * reset the range
                    % * set node pointer
                    inode = inode + 1;
                    range = inf;
                end
                % Integrate the kinematics over one time step
                xdot = v * cosd( app.Robot.pose(3) );
                ydot = v * sind( app.Robot.pose(3) );
                app.Robot.pose = app.Robot.pose + ...
                    dt * [xdot, ydot, w_des*(180./pi)] ;
                % Show the new position
                app.Robot.showGeom();
                app.hTraj(end+1) = line(app.UIAxes,...
                    app.Robot.pose(1),app.Robot.pose(2),...
                    'Marker','+','LineStyle','none');
                % Save a frame to the GIF if desired
                if app.saveGIF
                    % Graphics output (half time update rate to get 1/15 sec per frame)
                    if mod(i,app.iskip) == 0
                        gif
                    end
                end                
                % Want a CSV?
                if app.saveCSV
                    fprintf(app.fidCSV,'%f, %f, %f, %f\n', ...
                        app.Robot.pose(1), ...
                        app.Robot.pose(2), ...
                        mod(app.Robot.pose(3),360.), ...
                        v);
                end
                 % pause to generate graphics update
                pause(dt);
                i = i + 1;
                % Compute lap time
                app.lapTime = dt * i;
                app.LapTimeEditField.Value = app.lapTime;
                % Update each time history plot
                % Expand plot vectors
                app.tVector = [app.tVector, app.timeNow];
                app.omega_des = [app.omega_des, w_des];
                app.turnRadius = [app.turnRadius, abs(v/w_des)];
                app.curvature = [app.curvature, (w_des/(v+eps))];
                app.timeNow = app.timeNow + dt;
                ph2.XData = app.tVector;
                ph2.YData = app.omega_des;
                ph3.XData = app.tVector;
%                ph3.YData = app.curvature;
                ph3.YData = app.turnRadius;
            end % ends while loop over all nodes
            if app.saveCSV
                fclose(app.fidCSV);
            end
            
        end

        % Value changed function: SavetomyPathgifCheckBox
        function recordGIF(app, event)
            app.saveGIF = app.SavetomyPathgifCheckBox.Value;
        end

        % Button pushed function: EStopButton
        function eStop(app, event)
            app.eStopFlag = 1; % This will halt the simulation if it starts
        end

        % Value changed function: GIFframeskipEditField
        function newGIFskip(app, event)
            value = app.GIFframeskipEditField.Value;
            app.iskip = fix(value);
            app.GIFframeskipEditField.Value = app.iskip;
        end

        % Cell selection callback: UITable
        function UITableCellSelection(app, event)
            app.pickedRow = event.Indices(1);
        end

        % Button pushed function: AddleadingrowButton
        function AddleadingrowButtonPushed(app, event)
            Ddata = app.UITable.Data;
            if app.pickedRow > 1
                Ddata = [Ddata(1:(app.pickedRow-1),:); ...
                {0,0,0,0}; ...
                Ddata(app.pickedRow:end,:)];
            else
                Ddata = [{0,0,0,0};Ddata];
            end
            app.UITable.Data = Ddata;
            UITableDisplayDataChanged(app);
        end

        % Button pushed function: DeletepickedrowButton
        function DeletepickedrowButtonPushed(app, event)
            Ddata = app.UITable.Data;
            if app.pickedRow == size( Ddata, 1) % last
                Ddata = Ddata(1:(end-1),:);
            elseif app.pickedRow == 1 % first
                Ddata = Ddata(2:end,:);
            else 
                Ddata = [Ddata(1:(app.pickedRow-1),:); ...
                Ddata((app.pickedRow+1):end,:)];
            end
            app.UITable.Data = Ddata;
            UITableDisplayDataChanged(app);
        end

        % Value changed function: SavetomyPathcsvCheckBox
        function SavetomyPathcsvCheckBoxValueChanged(app, event)
            app.saveCSV = app.SavetomyPathcsvCheckBox.Value;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create AutonomousCourseUIFigure and hide until all components are created
            app.AutonomousCourseUIFigure = uifigure('Visible', 'off');
            app.AutonomousCourseUIFigure.Position = [100 100 733 548];
            app.AutonomousCourseUIFigure.Name = 'AutonomousCourse';

            % Create UITable
            app.UITable = uitable(app.AutonomousCourseUIFigure);
            app.UITable.ColumnName = {'X'; 'Y'; 'Heading'; 'Speed'};
            app.UITable.RowName = {};
            app.UITable.ColumnSortable = [false false false false];
            app.UITable.ColumnEditable = [true true true true];
            app.UITable.CellSelectionCallback = createCallbackFcn(app, @UITableCellSelection, true);
            app.UITable.DisplayDataChangedFcn = createCallbackFcn(app, @UITableDisplayDataChanged, true);
            app.UITable.Position = [39 37 670 125];

            % Create CourseSelectionButtonGroup
            app.CourseSelectionButtonGroup = uibuttongroup(app.AutonomousCourseUIFigure);
            app.CourseSelectionButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @courseSelection, true);
            app.CourseSelectionButtonGroup.Title = 'Course Selection';
            app.CourseSelectionButtonGroup.Position = [39 424 123 106];

            % Create SlalomButton
            app.SlalomButton = uiradiobutton(app.CourseSelectionButtonGroup);
            app.SlalomButton.Text = 'Slalom';
            app.SlalomButton.Position = [11 60 59 22];
            app.SlalomButton.Value = true;

            % Create BarrelRaceButton
            app.BarrelRaceButton = uiradiobutton(app.CourseSelectionButtonGroup);
            app.BarrelRaceButton.Text = 'BarrelRace';
            app.BarrelRaceButton.Position = [11 38 81 22];

            % Create BouncePathButton
            app.BouncePathButton = uiradiobutton(app.CourseSelectionButtonGroup);
            app.BouncePathButton.Text = 'BouncePath';
            app.BouncePathButton.Position = [11 16 88 22];

            % Create SimulateButton
            app.SimulateButton = uibutton(app.AutonomousCourseUIFigure, 'push');
            app.SimulateButton.ButtonPushedFcn = createCallbackFcn(app, @simulateRobot, true);
            app.SimulateButton.BackgroundColor = [0.5098 0.9294 0.3451];
            app.SimulateButton.Position = [42 395 118 22];
            app.SimulateButton.Text = 'Simulate!';

            % Create SavetomyPathgifCheckBox
            app.SavetomyPathgifCheckBox = uicheckbox(app.AutonomousCourseUIFigure);
            app.SavetomyPathgifCheckBox.ValueChangedFcn = createCallbackFcn(app, @recordGIF, true);
            app.SavetomyPathgifCheckBox.Text = 'Save to myPath.gif';
            app.SavetomyPathgifCheckBox.Position = [39 338 123 22];

            % Create EStopButton
            app.EStopButton = uibutton(app.AutonomousCourseUIFigure, 'push');
            app.EStopButton.ButtonPushedFcn = createCallbackFcn(app, @eStop, true);
            app.EStopButton.BackgroundColor = [0.9412 0.4706 0.4706];
            app.EStopButton.Position = [42 366 118 22];
            app.EStopButton.Text = 'E-Stop';

            % Create LapTimeEditFieldLabel
            app.LapTimeEditFieldLabel = uilabel(app.AutonomousCourseUIFigure);
            app.LapTimeEditFieldLabel.HorizontalAlignment = 'right';
            app.LapTimeEditFieldLabel.Position = [50 255 55 22];
            app.LapTimeEditFieldLabel.Text = 'Lap Time';

            % Create LapTimeEditField
            app.LapTimeEditField = uieditfield(app.AutonomousCourseUIFigure, 'numeric');
            app.LapTimeEditField.Limits = [0 Inf];
            app.LapTimeEditField.ValueDisplayFormat = '%7.2f';
            app.LapTimeEditField.Position = [108 255 54 22];

            % Create GIFframeskipEditFieldLabel
            app.GIFframeskipEditFieldLabel = uilabel(app.AutonomousCourseUIFigure);
            app.GIFframeskipEditFieldLabel.HorizontalAlignment = 'right';
            app.GIFframeskipEditFieldLabel.Position = [42 312 84 22];
            app.GIFframeskipEditFieldLabel.Text = 'GIF frame skip';

            % Create GIFframeskipEditField
            app.GIFframeskipEditField = uieditfield(app.AutonomousCourseUIFigure, 'numeric');
            app.GIFframeskipEditField.Limits = [1 10];
            app.GIFframeskipEditField.RoundFractionalValues = 'on';
            app.GIFframeskipEditField.ValueChangedFcn = createCallbackFcn(app, @newGIFskip, true);
            app.GIFframeskipEditField.Position = [133 312 29 22];
            app.GIFframeskipEditField.Value = 2;

            % Create TabGroup
            app.TabGroup = uitabgroup(app.AutonomousCourseUIFigure);
            app.TabGroup.Position = [197 199 513 332];

            % Create PathTrajectoryTab
            app.PathTrajectoryTab = uitab(app.TabGroup);
            app.PathTrajectoryTab.Title = 'Path Trajectory';

            % Create UIAxes
            app.UIAxes = uiaxes(app.PathTrajectoryTab);
            title(app.UIAxes, 'Course Diagram')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.Toolbar.Visible = 'off';
            app.UIAxes.XTick = [];
            app.UIAxes.YTick = [];
            app.UIAxes.Position = [10 1 502 298];

            % Create YawratecmdTab
            app.YawratecmdTab = uitab(app.TabGroup);
            app.YawratecmdTab.Title = 'Yaw rate cmd';

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.YawratecmdTab);
            title(app.UIAxes2, 'Turn Rate Cmd')
            xlabel(app.UIAxes2, 't (s)')
            ylabel(app.UIAxes2, 'rad/s')
            zlabel(app.UIAxes2, 'Z')
            app.UIAxes2.ZTick = [];
            app.UIAxes2.XGrid = 'on';
            app.UIAxes2.YGrid = 'on';
            app.UIAxes2.Position = [10 1 502 298];

            % Create RadiusofCurvatureTab
            app.RadiusofCurvatureTab = uitab(app.TabGroup);
            app.RadiusofCurvatureTab.Title = 'Radius of Curvature';

            % Create UIAxes3
            app.UIAxes3 = uiaxes(app.RadiusofCurvatureTab);
            title(app.UIAxes3, 'Turn Radius')
            xlabel(app.UIAxes3, 't (s)')
            ylabel(app.UIAxes3, 'ft')
            app.UIAxes3.XGrid = 'on';
            app.UIAxes3.YGrid = 'on';
            app.UIAxes3.Position = [10 1 502 298];

            % Create AddleadingrowButton
            app.AddleadingrowButton = uibutton(app.AutonomousCourseUIFigure, 'push');
            app.AddleadingrowButton.ButtonPushedFcn = createCallbackFcn(app, @AddleadingrowButtonPushed, true);
            app.AddleadingrowButton.Position = [42 227 118 22];
            app.AddleadingrowButton.Text = 'Add leading row';

            % Create DeletepickedrowButton
            app.DeletepickedrowButton = uibutton(app.AutonomousCourseUIFigure, 'push');
            app.DeletepickedrowButton.ButtonPushedFcn = createCallbackFcn(app, @DeletepickedrowButtonPushed, true);
            app.DeletepickedrowButton.Position = [42 199 118 22];
            app.DeletepickedrowButton.Text = 'Delete picked row';

            % Create SavetomyPathcsvCheckBox
            app.SavetomyPathcsvCheckBox = uicheckbox(app.AutonomousCourseUIFigure);
            app.SavetomyPathcsvCheckBox.ValueChangedFcn = createCallbackFcn(app, @SavetomyPathcsvCheckBoxValueChanged, true);
            app.SavetomyPathcsvCheckBox.Text = 'Save to myPath.csv';
            app.SavetomyPathcsvCheckBox.Position = [39 285 129 22];

            % Show the figure after all components are created
            app.AutonomousCourseUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = PathPlannerApp2_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.AutonomousCourseUIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.AutonomousCourseUIFigure)
        end
    end
end