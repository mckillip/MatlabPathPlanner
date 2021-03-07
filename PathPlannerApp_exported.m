classdef PathPlannerApp_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        AutonomousCourseUIFigure    matlab.ui.Figure
        UIAxes                      matlab.ui.control.UIAxes
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
    end
    
    methods (Access = private)
    
        function updateplot(app)
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
            obj.numNodes = length(t.X);
            if app.numNodes <=0
                return
            end
            
            % Draw the robot at the first location
            app.Robot.pose=[t.X(1),t.Y(1),t.H(1)];
            app.Robot.showGeom();
            
            % Want a movie?
            if app.saveGIF
                gif('myPath.gif','frame',app.UIAxes);
            end
            
            % Initialize counters and such
            range = inf;
            inode = 2; % pointer to next node in table
            i = 1;     % skip timer index for gif movie output
            % Pick a speed and a time step
            dt = 0.03333; % 30Hz, 2x default GIF playback speed
            
            % The loop: do each segment, and check for eSTOP
            while inode <= obj.numNodes
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
                    % We have a choice: MATLAB function or Java class!
%                    [range,w_des] = smoothControl(v,rPos,targetPose);
                    [range,w_des] = smoothControl2(v,rPos,targetPose);
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
                % Save a frame to the GIF if desired
                if app.saveGIF
                    % Graphics output (half time update rate to get 1/15 sec per frame)
                    if mod(i,app.iskip) == 0
                        gif
                    end
                end                
                % pause to generate graphics update
                pause(dt);
                i = i + 1;
                % Compute lap time
                app.lapTime = dt * i;
                app.LapTimeEditField.Value = app.lapTime;
            end % ends while loop over all nodes
            
            
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
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create AutonomousCourseUIFigure and hide until all components are created
            app.AutonomousCourseUIFigure = uifigure('Visible', 'off');
            app.AutonomousCourseUIFigure.Position = [100 100 733 548];
            app.AutonomousCourseUIFigure.Name = 'AutonomousCourse';

            % Create UIAxes
            app.UIAxes = uiaxes(app.AutonomousCourseUIFigure);
            title(app.UIAxes, 'Course Diagram')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.XTick = [];
            app.UIAxes.YTick = [];
            app.UIAxes.Position = [196 205 513 319];

            % Create UITable
            app.UITable = uitable(app.AutonomousCourseUIFigure);
            app.UITable.ColumnName = {'X'; 'Y'; 'Heading'; 'Speed'};
            app.UITable.RowName = {};
            app.UITable.ColumnSortable = [false false false false];
            app.UITable.ColumnEditable = [true true true true];
            app.UITable.DisplayDataChangedFcn = createCallbackFcn(app, @UITableDisplayDataChanged, true);
            app.UITable.Position = [36 37 673 125];

            % Create CourseSelectionButtonGroup
            app.CourseSelectionButtonGroup = uibuttongroup(app.AutonomousCourseUIFigure);
            app.CourseSelectionButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @courseSelection, true);
            app.CourseSelectionButtonGroup.Title = 'Course Selection';
            app.CourseSelectionButtonGroup.Position = [39 392 123 106];

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
            app.SimulateButton.Position = [42 362 118 22];
            app.SimulateButton.Text = 'Simulate!';

            % Create SavetomyPathgifCheckBox
            app.SavetomyPathgifCheckBox = uicheckbox(app.AutonomousCourseUIFigure);
            app.SavetomyPathgifCheckBox.ValueChangedFcn = createCallbackFcn(app, @recordGIF, true);
            app.SavetomyPathgifCheckBox.Text = 'Save to myPath.gif';
            app.SavetomyPathgifCheckBox.Position = [39 296 123 22];

            % Create EStopButton
            app.EStopButton = uibutton(app.AutonomousCourseUIFigure, 'push');
            app.EStopButton.ButtonPushedFcn = createCallbackFcn(app, @eStop, true);
            app.EStopButton.BackgroundColor = [0.9412 0.4706 0.4706];
            app.EStopButton.Position = [42 330 118 22];
            app.EStopButton.Text = 'E-Stop';

            % Create LapTimeEditFieldLabel
            app.LapTimeEditFieldLabel = uilabel(app.AutonomousCourseUIFigure);
            app.LapTimeEditFieldLabel.HorizontalAlignment = 'right';
            app.LapTimeEditFieldLabel.Position = [50 229 55 22];
            app.LapTimeEditFieldLabel.Text = 'Lap Time';

            % Create LapTimeEditField
            app.LapTimeEditField = uieditfield(app.AutonomousCourseUIFigure, 'numeric');
            app.LapTimeEditField.Limits = [0 Inf];
            app.LapTimeEditField.ValueDisplayFormat = '%7.2f';
            app.LapTimeEditField.Position = [108 229 54 22];

            % Create GIFframeskipEditFieldLabel
            app.GIFframeskipEditFieldLabel = uilabel(app.AutonomousCourseUIFigure);
            app.GIFframeskipEditFieldLabel.HorizontalAlignment = 'right';
            app.GIFframeskipEditFieldLabel.Position = [42 264 84 22];
            app.GIFframeskipEditFieldLabel.Text = 'GIF frame skip';

            % Create GIFframeskipEditField
            app.GIFframeskipEditField = uieditfield(app.AutonomousCourseUIFigure, 'numeric');
            app.GIFframeskipEditField.Limits = [1 10];
            app.GIFframeskipEditField.RoundFractionalValues = 'on';
            app.GIFframeskipEditField.ValueChangedFcn = createCallbackFcn(app, @newGIFskip, true);
            app.GIFframeskipEditField.Position = [133 264 29 22];
            app.GIFframeskipEditField.Value = 2;

            % Show the figure after all components are created
            app.AutonomousCourseUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = PathPlannerApp_exported

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