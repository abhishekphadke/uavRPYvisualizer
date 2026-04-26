classdef uavRPYvisualizer < matlab.apps.AppBase
    % Quaternion attitude demo (X-config) with stabilized "normal flight" behavior.
    % Adds:
    % - Scrollable controls (motor 4 not cut off)
    % - Front motors BLUE (1-2), rear motors GREEN (3-4)
    % - Red nose marker indicating FRONT direction
    % - Self-check that front/rear mapping matches geometry

    properties (Access = public)
        UIFigure matlab.ui.Figure
        MainGrid matlab.ui.container.GridLayout
        LeftPanel matlab.ui.container.Panel
        RightPanel matlab.ui.container.Panel
        LeftGrid matlab.ui.container.GridLayout

        UIAxes matlab.ui.control.UIAxes

        StartButton matlab.ui.control.Button
        PauseButton matlab.ui.control.Button
        ResetButton matlab.ui.control.Button

        DemoRollButton matlab.ui.control.Button
        DemoPitchButton matlab.ui.control.Button
        DemoYawButton matlab.ui.control.Button
        DemoStopButton matlab.ui.control.Button

        DtField matlab.ui.control.NumericEditField

        WSliders matlab.ui.control.Slider
        WFields  matlab.ui.control.NumericEditField

        RollLabel matlab.ui.control.Label
        PitchLabel matlab.ui.control.Label
        YawLabel matlab.ui.control.Label
        StatusLabel matlab.ui.control.Label

        HelpText matlab.ui.control.TextArea

        TimerObj timer
    end

    properties (Access = private)
        p

        q  (1,4) double
        wB (3,1) double

        DemoMode string = "none"
        DemoT0 uint64 = uint64(0)

        QuadXform matlab.graphics.primitive.Transform
        RotorXform matlab.graphics.primitive.Transform
        RotorPatch matlab.graphics.primitive.Patch
        RotorSpinAngle (4,1) double

        NoseXform matlab.graphics.primitive.Transform
        NoseSurf  

        IsRunning logical = false
    end

    methods (Access = public)
        function app = uavRPYvisualizer()
            app.startup();
        end

        function delete(app)
            app.safeStopTimer();
            try
                if ~isempty(app.TimerObj) && isvalid(app.TimerObj)
                    delete(app.TimerObj);
                end
            catch
            end
            try
                if ~isempty(app.UIFigure) && isvalid(app.UIFigure)
                    delete(app.UIFigure);
                end
            catch
            end
        end
    end

    methods (Access = private)

        function startup(app)
            app.p = app.quadParams();

            app.createUI();
            app.buildQuadGraphics();
            app.resetState();
          
            app.TimerObj = timer( ...
                'ExecutionMode','fixedRate', ...
                'BusyMode','drop', ...
                'Period', 0.02, ...
                'TimerFcn', @(~,~)app.timerTickSafe());
        end

        function p = quadParams(app) %#ok<MANU>
            p.L  = 0.25;
            p.I  = diag([0.02 0.02 0.04]);

            p.wMin = 0;
            p.wMax = 1200;
            p.dtMin = 0.001;
            p.dtMax = 0.05;

            p.z0 = 0.20;

            p.yawSign = [ +1; -1; +1; -1 ];

            % Bounded "normal flight" attitude
            p.maxRollDeg  = 20;
            p.maxPitchDeg = 20;
            p.maxYawDeg   = 35;

            p.cmdGainRoll  = 0.00008;
            p.cmdGainPitch = 0.00008;
            p.cmdGainYaw   = 0.00003;

            p.Kp = diag([2.0  2.0  1.2]);
            p.Kd = diag([0.25 0.25 0.20]);

            p.tauMax  = [1.0; 1.0; 0.6];
            p.wMaxBody = [5; 5; 3];

            p.rotorR = 0.08;
            p.armW   = 0.02;
            p.armT   = 0.01;
            p.hubDims = [0.07 0.07 0.03];

            % Nose marker
            p.noseOffset = 0.12; % along FRONT direction in body frame
            p.noseRadius = 0.015;
            p.noseZ      = 0.03;
        end

        function createUI(app)
            app.UIFigure = uifigure('Name','UAV Roll-Pitch-Yaw Visualizer', ...
                'Position',[100 100 1250 720]);

            app.MainGrid = uigridlayout(app.UIFigure,[1 2]);
            app.MainGrid.ColumnWidth = {460,'1x'};
            app.MainGrid.Padding = [10 10 10 10];
            app.MainGrid.ColumnSpacing = 10;

            app.LeftPanel = uipanel(app.MainGrid,'Title','Controls');
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            app.LeftPanel.Scrollable = 'on';

            app.RightPanel = uipanel(app.MainGrid,'Title','Visualization');
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            app.LeftGrid = uigridlayout(app.LeftPanel,[19 3]);
            app.LeftGrid.RowHeight = { ...
                32,32,32,8, ...
                24,30,8, ...
                24, ...
                24,24,24,24, ...
                8, ...
                24,24,24, ...
                24, ...
                140, ...
                10};
            app.LeftGrid.ColumnWidth = {190,'1x',90};
            app.LeftGrid.Padding = [10 10 10 10];
            app.LeftGrid.RowSpacing = 8;

            axGrid = uigridlayout(app.RightPanel,[1 1]);
            axGrid.Padding = [8 8 8 8];

            app.UIAxes = uiaxes(axGrid);
            title(app.UIAxes,'UAV state');
            xlabel(app.UIAxes,'X'); ylabel(app.UIAxes,'Y'); zlabel(app.UIAxes,'Z');
            grid(app.UIAxes,'on'); axis(app.UIAxes,'equal');
            view(app.UIAxes,35,20);
            hold(app.UIAxes,'on');
            camlight(app.UIAxes,'headlight');
            material(app.UIAxes,'dull');

            app.StartButton = uibutton(app.LeftGrid,'Text','Start', 'ButtonPushedFcn',@(~,~)app.onStart());
            app.StartButton.Layout.Row = 1; app.StartButton.Layout.Column = 1;

            app.PauseButton = uibutton(app.LeftGrid,'Text','Pause', 'ButtonPushedFcn',@(~,~)app.onPause());
            app.PauseButton.Layout.Row = 1; app.PauseButton.Layout.Column = 2;

            app.ResetButton = uibutton(app.LeftGrid,'Text','Reset', 'ButtonPushedFcn',@(~,~)app.onReset());
            app.ResetButton.Layout.Row = 1; app.ResetButton.Layout.Column = 3;

            app.DemoRollButton = uibutton(app.LeftGrid,'Text','Demo: Roll',  'ButtonPushedFcn',@(~,~)app.onDemo("roll"));
            app.DemoRollButton.Layout.Row = 2; app.DemoRollButton.Layout.Column = 1;

            app.DemoPitchButton = uibutton(app.LeftGrid,'Text','Demo: Pitch', 'ButtonPushedFcn',@(~,~)app.onDemo("pitch"));
            app.DemoPitchButton.Layout.Row = 2; app.DemoPitchButton.Layout.Column = 2;

            app.DemoYawButton = uibutton(app.LeftGrid,'Text','Demo: Yaw',   'ButtonPushedFcn',@(~,~)app.onDemo("yaw"));
            app.DemoYawButton.Layout.Row = 2; app.DemoYawButton.Layout.Column = 3;

            app.DemoStopButton = uibutton(app.LeftGrid,'Text','Stop Demo (Manual Control)', ...
                'ButtonPushedFcn',@(~,~)app.onDemo("none"));
            app.DemoStopButton.Layout.Row = 3; app.DemoStopButton.Layout.Column = [1 3];

            dtLbl = uilabel(app.LeftGrid,'Text','Time step dt (s):','FontWeight','bold');
            dtLbl.Layout.Row = 5; dtLbl.Layout.Column = [1 3];

            app.DtField = uieditfield(app.LeftGrid,'numeric','Value',0.01,'Limits',[app.p.dtMin app.p.dtMax]);
            app.DtField.Layout.Row = 6; app.DtField.Layout.Column = 1;

            dtHint = uilabel(app.LeftGrid,'Text','Recommended: 0.01','FontAngle','italic');
            dtHint.Layout.Row = 6; dtHint.Layout.Column = [2 3];

            mh = uilabel(app.LeftGrid,'Text','Motor Speeds ωᵢ (rad/s)','FontWeight','bold');
            mh.Layout.Row = 8; mh.Layout.Column = [1 3];

            app.WSliders = matlab.ui.control.Slider.empty(4,0);
            app.WFields  = matlab.ui.control.NumericEditField.empty(4,0);

            motorNames = { ...
                'Motor 1: Front-Left (CW) [BLUE]', ...
                'Motor 2: Front-Right (CCW) [BLUE]', ...
                'Motor 3: Rear-Right (CW) [GREEN]', ...
                'Motor 4: Rear-Left (CCW) [GREEN]'};

            for i = 1:4
                row = 8 + i;
                ml = uilabel(app.LeftGrid,'Text',motorNames{i});
                ml.Layout.Row = row; ml.Layout.Column = 1;

                app.WSliders(i) = uislider(app.LeftGrid, ...
                    'Limits',[app.p.wMin app.p.wMax], ...
                    'MajorTicks',0:200:1200, ...
                    'Value',700, ...
                    'ValueChangingFcn',@(s,e)app.onSliderChanging(i,e), ...
                    'ValueChangedFcn',@(~,~)app.onSliderChanged(i));
                app.WSliders(i).Layout.Row = row; app.WSliders(i).Layout.Column = 2;

                app.WFields(i) = uieditfield(app.LeftGrid,'numeric', ...
                    'Limits',[app.p.wMin app.p.wMax], ...
                    'Value',700, ...
                    'ValueChangedFcn',@(~,~)app.onFieldChanged(i));
                app.WFields(i).Layout.Row = row; app.WFields(i).Layout.Column = 3;
            end

            app.RollLabel  = uilabel(app.LeftGrid,'Text','Roll  φ: 0.0 deg','FontWeight','bold');
            app.RollLabel.Layout.Row = 14; app.RollLabel.Layout.Column = [1 3];

            app.PitchLabel = uilabel(app.LeftGrid,'Text','Pitch θ: 0.0 deg','FontWeight','bold');
            app.PitchLabel.Layout.Row = 15; app.PitchLabel.Layout.Column = [1 3];

            app.YawLabel   = uilabel(app.LeftGrid,'Text','Yaw   ψ: 0.0 deg','FontWeight','bold');
            app.YawLabel.Layout.Row = 16; app.YawLabel.Layout.Column = [1 3];

            app.StatusLabel = uilabel(app.LeftGrid,'Text','Status: Stopped','FontWeight','bold');
            app.StatusLabel.Layout.Row = 17; app.StatusLabel.Layout.Column = [1 3];

            app.HelpText = uitextarea(app.LeftGrid,'Editable','off', ...
                'Value', { ...
                    'Color legend: FRONT motors (1–2) are BLUE; REAR motors (3–4) are GREEN.', ...
                    'Red nose marker indicates the FRONT direction.', ...
                    'Constant altitude: UAV rendered at fixed z = 0.20 m.', ...
                    'Bounded attitude: Roll/Pitch limited to ±20° (no flips).', ...
                    'Use Demo buttons for clean examples.'});
            app.HelpText.Layout.Row = 18; app.HelpText.Layout.Column = [1 3];
        end

        function buildQuadGraphics(app)
            ax = app.UIAxes;
            app.drawGroundPlane(ax);

            app.QuadXform = hgtransform('Parent',ax);

            [V1,F1] = app.makeBox([0 0 0],[2*app.p.L app.p.armW app.p.armT], +45);
            [V2,F2] = app.makeBox([0 0 0],[2*app.p.L app.p.armW app.p.armT], -45);
            patch('Parent',app.QuadXform,'Vertices',V1,'Faces',F1,'FaceColor',[0.2 0.2 0.2],'EdgeColor','none');
            patch('Parent',app.QuadXform,'Vertices',V2,'Faces',F2,'FaceColor',[0.2 0.2 0.2],'EdgeColor','none');

            [Vh,Fh] = app.makeBox([0 0 0.015],app.p.hubDims,0);
            patch('Parent',app.QuadXform,'Vertices',Vh,'Faces',Fh,'FaceColor',[0.1 0.1 0.1],'EdgeColor','none');

            % Rotor positions in BODY frame (X forward, Y right, Z up for display)
            L = app.p.L/sqrt(2);

            % Motor order: 1 FL, 2 FR, 3 RR, 4 RL
            rPos = [ ...
            +L, +L, -L, -L;   % x (front +, rear -)
            -L, +L, +L, -L;   % y (left -, right +)
             0,  0,  0,  0];  % z

         

           

            app.RotorXform = matlab.graphics.primitive.Transform.empty(4,0);
            app.RotorPatch = matlab.graphics.primitive.Patch.empty(4,0);
            app.RotorSpinAngle = zeros(4,1);

            for i = 1:4
                app.RotorXform(i) = hgtransform('Parent',app.QuadXform);
                app.RotorXform(i).Matrix = makehgtform('translate', rPos(1,i), rPos(2,i), rPos(3,i)+0.01);

                [Xd,Yd,Zd] = app.makeDisk(app.p.rotorR, 40);

                if i <= 2
                    c = [0.00 0.45 0.90]; % BLUE = front motors (1,2)
                else
                    c = [0.00 0.65 0.25]; % GREEN = rear motors (3,4)
                end

                app.RotorPatch(i) = patch('Parent',app.RotorXform(i), ...
                    'XData',Xd,'YData',Yd,'ZData',Zd, ...
                    'FaceColor',c,'FaceAlpha',0.60,'EdgeColor','none');
            end

            % Nose marker: create on AXES then reparent to hgtransform (robust fix)
            app.NoseXform = hgtransform('Parent',app.QuadXform);
           
            % Nose marker at FRONT (+X)
            nosePosB = [app.p.noseOffset; 0; app.p.noseZ];
            app.NoseXform.Matrix = makehgtform('translate', nosePosB(1), nosePosB(2), nosePosB(3));

          

            [sx,sy,sz] = sphere(18);
            X = app.p.noseRadius*sx;
            Y = app.p.noseRadius*sy;
            Z = app.p.noseRadius*sz;

            app.NoseSurf = surf(app.UIAxes, X, Y, Z, 'FaceColor',[0.85 0.05 0.05], 'EdgeColor','none');
            app.NoseSurf.Parent = app.NoseXform;

            lim = 0.6;
            xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[0 lim]);
            view(ax,35,20);
        end

       

        function drawGroundPlane(app, ax) %#ok<MANU>
            g = 0.6;
            step = 0.1;
            xs = -g:step:g;

            for x = xs
                plot3(ax,[x x],[-g g],[0 0],'Color',[0.85 0.85 0.85]);
            end
            for y = xs
                plot3(ax,[-g g],[y y],[0 0],'Color',[0.85 0.85 0.85]);
            end
        end

        function [V,F] = makeBox(app, center, dims, yawDeg) %#ok<MANU>
            dx=dims(1); dy=dims(2); dz=dims(3);
            cx=center(1); cy=center(2); cz=center(3);
            x=dx/2; y=dy/2; z=dz/2;

            V = [ ...
                -x -y -z;
                 x -y -z;
                 x  y -z;
                -x  y -z;
                -x -y  z;
                 x -y  z;
                 x  y  z;
                -x  y  z ];

            if yawDeg ~= 0
                a = deg2rad(yawDeg);
                Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
                V = (Rz*V')';
            end
            V = V + [cx cy cz];

            F = [ ...
                1 2 3 4;
                5 6 7 8;
                1 2 6 5;
                2 3 7 6;
                3 4 8 7;
                4 1 5 8 ];
        end

        function [X,Y,Z] = makeDisk(app, R, N) %#ok<MANU>
            th = linspace(0,2*pi,N);
            X = [0, R*cos(th)];
            Y = [0, R*sin(th)];
            Z = zeros(size(X));
        end

    end

    %==========================
    % Timer / Dynamics / Updates
    %==========================
    methods (Access = private)

        function onStart(app)
            if isempty(app.TimerObj) || ~isvalid(app.TimerObj), return; end
            dt = app.DtField.Value;
            app.TimerObj.Period = max(0.01, min(0.05, dt));
            app.IsRunning = true;
            app.StatusLabel.Text = "Status: Running";
            start(app.TimerObj);
        end

        function onPause(app)
            app.safeStopTimer();
            app.IsRunning = false;
            app.StatusLabel.Text = "Status: Paused";
        end

        function onReset(app)
            app.safeStopTimer();
            app.DemoMode = "none";
            app.resetState();
            app.IsRunning = false;
            app.StatusLabel.Text = "Status: Stopped";
        end

        function onDemo(app, mode)
            app.DemoMode = string(mode);
            if app.DemoMode == "none", return; end
            app.DemoT0 = tic;
            app.applyDemoPattern(0);
        end

        function onSliderChanging(app, i, evt)
            app.WFields(i).Value = evt.Value;
        end
        function onSliderChanged(app, i)
            app.WFields(i).Value = app.WSliders(i).Value;
        end
        function onFieldChanged(app, i)
            app.WSliders(i).Value = app.WFields(i).Value;
        end

        function resetState(app)
            app.q  = [1 0 0 0];
            app.wB = [0;0;0];
            app.RotorSpinAngle(:) = 0;
            app.updateGraphics();
            app.updateReadouts();
        end

        function safeStopTimer(app)
            try
                if ~isempty(app.TimerObj) && isvalid(app.TimerObj)
                    stop(app.TimerObj);
                end
            catch
            end
        end

        function timerTickSafe(app)
            try
                app.timerTick();
            catch ME
                app.safeStopTimer();
                app.IsRunning = false;
                app.StatusLabel.Text = "Status: Error (timer stopped)";
                fprintf(2,'Timer error: %s\n', ME.message);
            end
        end

        function timerTick(app)
            dt = app.DtField.Value;

            if app.DemoMode ~= "none"
                t = toc(app.DemoT0);
                app.applyDemoPattern(t);
            end

            w = zeros(4,1);
            for i = 1:4
                w(i) = app.WSliders(i).Value;
            end

            tauB = app.motorSpeedsToStabilizedTorque(w);
            app.stepAttitudeRK2(dt, tauB);
            app.stepRotorSpin(dt, w);

            app.updateGraphics();
            app.updateReadouts();
        end

        function applyDemoPattern(app, t)
            base = 700;
            d = 140;
            ramp = min(1, t/0.6);

            w = base * ones(4,1);
            switch app.DemoMode
                case "roll"
                    w([2 3]) = base + ramp*d;
                    w([1 4]) = base - ramp*d;
                case "pitch"
                    w([3 4]) = base + ramp*d;
                    w([1 2]) = base - ramp*d;
                case "yaw"
                    w([1 3]) = base + ramp*d;
                    w([2 4]) = base - ramp*d;
                otherwise
                    return;
            end

            w = max(app.p.wMin, min(app.p.wMax, w));
            for i = 1:4
                app.WSliders(i).Value = w(i);
                app.WFields(i).Value  = w(i);
            end
        end

        function tauB = motorSpeedsToStabilizedTorque(app, w)
            s = w(:).^2;

            %rollSig  = (s(2)+s(3)) - (s(1)+s(4));
            %pitchSig = (s(3)+s(4)) - (s(1)+s(2));
            %yawSig   = (s(1)+s(3)) - (s(2)+s(4));

            pitchSig = (s(1)+s(2)) - (s(3)+s(4)); % front - rear  (rear↑ => negative => nose-down)
            rollSig = (s(1)+s(4)) - (s(2)+s(3)); % left - right
           %yawSig = (s(2)+s(4)) - (s(1)+s(3)); % CW - CCW
            yawSig = (s(1)+s(3)) - (s(2)+s(4)); % CCW - CW




            phiCmd   = app.p.cmdGainRoll  * rollSig;
            thetaCmd = app.p.cmdGainPitch * pitchSig;
            psiCmd   = app.p.cmdGainYaw   * yawSig;

            phiCmd   = max(-deg2rad(app.p.maxRollDeg),  min(deg2rad(app.p.maxRollDeg),  phiCmd));
            thetaCmd = max(-deg2rad(app.p.maxPitchDeg), min(deg2rad(app.p.maxPitchDeg), thetaCmd));
            psiCmd   = max(-deg2rad(app.p.maxYawDeg),   min(deg2rad(app.p.maxYawDeg),   psiCmd));

            [phi,theta,psi] = app.quatToEulerZYX(app.q);

            e = [phiCmd - phi;
                 thetaCmd - theta;
                 app.wrapToPi(psiCmd - psi)];

            tau = app.p.Kp * e - app.p.Kd * app.wB;
            tau = max(-app.p.tauMax, min(app.p.tauMax, tau));
            tauB = tau;
        end

        function stepAttitudeRK2(app, dt, tauB)
            I = app.p.I;

            w0 = app.wB;
            wdot0 = I \ (tauB - cross(w0, I*w0));
            wmid  = w0 + 0.5*dt*wdot0;

            wdotm = I \ (tauB - cross(wmid, I*wmid));
            w1    = w0 + dt*wdotm;

            w1 = max(-app.p.wMaxBody, min(app.p.wMaxBody, w1));

            q0 = app.q(:).';

            qdot0 = 0.5 * (app.omegaMat(w0) * q0.').';
            qmid  = q0 + 0.5*dt*qdot0;

            qdotm = 0.5 * (app.omegaMat(w1) * qmid.').';
            q1    = q0 + dt*qdotm;

            n = norm(q1);
            if ~isfinite(n) || n < 1e-12
                q1 = [1 0 0 0];
                w1 = [0;0;0];
            else
                q1 = q1 / n;
            end

            app.wB = w1;
            app.q  = q1;
        end

        function Omega = omegaMat(app, w) %#ok<MANU>
            Omega = [ 0    -w(1) -w(2) -w(3);
                      w(1)  0     w(3) -w(2);
                      w(2) -w(3)  0     w(1);
                      w(3)  w(2) -w(1)  0    ];
        end

        function stepRotorSpin(app, dt, w)
            scale = 0.02;
            for i = 1:4
                app.RotorSpinAngle(i) = app.RotorSpinAngle(i) + app.p.yawSign(i)*scale*w(i)*dt;
            end
        end

        function updateGraphics(app)
            R = app.quatToRotm(app.q);

            M = diag([1 1 -1]);
            Rw = M * R * M;
            if any(~isfinite(Rw(:))), return; end

            pW = [0;0;app.p.z0];
            app.QuadXform.Matrix = [Rw pW; 0 0 0 1];

   

         % Rotor positions in BODY frame (X forward, Y right, Z up)
             L = app.p.L/sqrt(2);

        % Motor order: 1 FL, 2 FR, 3 RR, 4 RL
             rPos = [ ...
            +L, +L, -L, -L;   % x: front +, rear -
            -L, +L, +L, -L;   % y: left -, right +
             0,  0,  0,  0];  % z


            for i = 1:4
                T = makehgtform('translate', rPos(1,i), rPos(2,i), rPos(3,i)+0.01);
                S = makehgtform('zrotate', app.RotorSpinAngle(i));
                app.RotorXform(i).Matrix = T*S;
            end

            drawnow limitrate nocallbacks
        end

        function updateReadouts(app)
            [phi,theta,psi] = app.quatToEulerZYX(app.q);
            app.RollLabel.Text  = sprintf('Roll  φ: %.1f deg', rad2deg(phi));
            app.PitchLabel.Text = sprintf('Pitch θ: %.1f deg', rad2deg(theta));
            app.YawLabel.Text   = sprintf('Yaw   ψ: %.1f deg', rad2deg(psi));
        end

        function R = quatToRotm(app, q) %#ok<MANU>
            qw=q(1); qx=q(2); qy=q(3); qz=q(4);
            R = [ ...
                1-2*(qy^2+qz^2),   2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy);
                2*(qx*qy + qw*qz), 1-2*(qx^2+qz^2),   2*(qy*qz - qw*qx);
                2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx^2+qy^2) ];
        end

        function [phi,theta,psi] = quatToEulerZYX(app, q) %#ok<MANU>
            R = app.quatToRotm(q);
            psi   = atan2(R(2,1), R(1,1));
            theta = -asin(max(-1,min(1,R(3,1))));
            phi   = atan2(R(3,2), R(3,3));
        end

        function a = wrapToPi(app, a) %#ok<MANU>
            a = mod(a + pi, 2*pi) - pi;
        end

    end
end
