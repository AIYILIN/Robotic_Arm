classdef arm_app < matlab.apps.AppBase

    properties (Access = public)
        % 基础UI组件
        UIFigure         matlab.ui.Figure
        % ... [原有组件声明保持不变] ...
        
        % 新增运动控制组件
        JointPanel       matlab.ui.container.Panel
        JointSliders     matlab.ui.control.Slider[6]
        JointDisplays    matlab.ui.control.NumericEditField[6]
        
        CartesianPanel   matlab.ui.container.Panel  
        PosFields        matlab.ui.control.NumericEditField[3]
        RotFields        matlab.ui.control.NumericEditField[3]
        
        % 可视化组件
        VisualTabGroup   matlab.ui.container.TabGroup
        JointPlotTab     matlab.ui.container.Tab
        TrajectoryTab    matlab.ui.container.Tab
        JointAxes        matlab.ui.control.UIAxes
        TrajectoryAxes   matlab.ui.control.UIAxes
        
        % 控制按钮
        FKButton         matlab.ui.control.Button
        IKButton         matlab.ui.control.Button
        TeachButton      matlab.ui.control.StateButton
    end

    properties (Access = private)
        % 机械臂模型
        RobotModel
        ToolOffset = 168.51; % TCP偏移量
        
        % 串口连接
        SerialPort
        IsConnected = false
        
        % 数据记录
        JointHistory
        PoseHistory
        MaxHistory = 500; % 最大记录点数
        
        % 定时器
        TeachTimer
    end

    methods (Access = private)
        
        function setupRobotModel(app)
            % 初始化D-H参数
            L(1) = Link('d',0,    'a',0,    'alpha',0,     'modified');
            L(2) = Link('d',0,    'a',0,    'alpha',pi/2,  'modified');
            L(3) = Link('d',0,    'a',199.54,'alpha',0,     'modified');
            L(4) = Link('d',171.53,'a',95.91,'alpha',pi/2,  'modified');
            L(5) = Link('d',0,    'a',0,    'alpha',-pi/2, 'modified');
            L(6) = Link('d',0,    'a',0,    'alpha',pi/2,  'modified');
            
            app.RobotModel = SerialLink(L, 'name', 'Arm');
            app.RobotModel.tool = transl(0,0,app.ToolOffset);
        end
        
        function updateJointPlots(app)
            % 更新关节角度曲线
            cla(app.JointAxes);
            t = 1:size(app.JointHistory,1);
            for i = 1:6
                subplot(3,2,i, 'Parent', app.JointAxes);
                plot(t, app.JointHistory(:,i));
                title(['Joint ', num2str(i)]);
            end
        end
        
        function updateTrajectoryPlot(app)
            % 更新末端轨迹
            cla(app.TrajectoryAxes);
            plot3(app.TrajectoryAxes, ...
                app.PoseHistory(:,1), app.PoseHistory(:,2), app.PoseHistory(:,3));
            view(app.TrajectoryAxes, 3);
            grid(app.TrajectoryAxes, 'on');
        end
        
        function sendJointCommand(app, angles)
            % 构造通信协议
            header = uint8([0xAA 0x55]);
            data = typecast(single(angles), 'uint8');
            crc = crc16(data);
            packet = [header data crc];
            
            % 串口发送
            if app.IsConnected
                write(app.SerialPort, packet, 'uint8');
            end
        end
        
        function processRealTimeTeaching(app)
            % 示教模式定时回调
            currentAngles = [app.JointSliders.Value];
            T = app.RobotModel.fkine(deg2rad(currentAngles));
            
            % 记录数据
            app.JointHistory = [app.JointHistory; currentAngles];
            app.PoseHistory = [app.PoseHistory; T.t'];
            
            % 更新显示
            updateJointPlots(app);
            updateTrajectoryPlot(app);
            
            % 发送指令
            sendJointCommand(app, currentAngles);
        end
    end

    % 回调函数
    methods (Access = private)
        
        function onConnectButtonPushed(app)
            % 串口连接回调
            port = app.DropDown.Value;
            baud = str2double(app.DropDown_2.Value);
            
            try
                app.SerialPort = serialport(port, baud);
                configureTerminator(app.SerialPort, "CR");
                app.IsConnected = true;
                app.Lamp.Color = 'green';
            catch
                uialert(app.UIFigure, '串口连接失败', '错误');
            end
        end
        
        function onJointSliderMoved(app, src)
            % 关节滑块回调
            jointNum = find(app.JointSliders == src);
            app.JointDisplays(jointNum).Value = src.Value;
            
            % 实时模式处理
            if app.TeachButton.Value
                processRealTimeTeaching(app);
            end
        end
        
        function onFKButtonPushed(app)
            % 正运动学计算
            q = [app.JointSliders.Value];
            T = app.RobotModel.fkine(deg2rad(q));
            
            % 更新笛卡尔显示
            app.PosFields(1).Value = T.t(1);
            app.PosFields(2).Value = T.t(2);
            app.PosFields(3).Value = T.t(3);
            
            % 更新姿态显示
            eul = rotm2eul(T.R, 'XYZ');
            app.RotFields(1).Value = rad2deg(eul(1));
            app.RotFields(2).Value = rad2deg(eul(2));
            app.RotFields(3).Value = rad2deg(eul(3));
        end
        
        function onIKButtonPushed(app)
            % 逆运动学计算
            pos = [app.PosFields.Value];
            eul = deg2rad([app.RotFields.Value]);
            R = eul2rotm(eul, 'XYZ');
            T = [R, pos'; 0 0 0 1];
            
            try
                q = app.RobotModel.ikine(T, 'tol', 1e-6);
                q_deg = rad2deg(q);
                
                % 更新关节显示
                for i = 1:6
                    app.JointSliders(i).Value = q_deg(i);
                    app.JointDisplays(i).Value = q_deg(i);
                end
            catch
                uialert(app.UIFigure, '逆运动学无解', '计算错误');
            end
        end
        
        function onTeachButtonValueChanged(app)
            % 示教模式切换
            if app.TeachButton.Value
                % 启动定时器
                app.TeachTimer = timer(...
                    'ExecutionMode', 'fixedRate', ...
                    'Period', 0.1, ...
                    'TimerFcn', @(~,~)processRealTimeTeaching(app));
                start(app.TeachTimer);
            else
                % 停止定时器
                stop(app.TeachTimer);
                delete(app.TeachTimer);
            end
        end
    end

    % 应用初始化
    methods (Access = private)
        
        function createComponents(app)
            % 创建基础组件
            createComponents@arm_app_base(app);  % 继承原有组件
            
            % 创建机械臂控制面板
            createJointControlPanel(app);
            createCartesianControlPanel(app);
            createVisualizationTabs(app);
            setupRobotModel(app);
        end
        
        function createJointControlPanel(app)
            app.JointPanel = uipanel(app.UIFigure);
            app.JointPanel.Title = '关节空间控制';
            app.JointPanel.Position = [250 450 400 150];
            
            % 创建6个关节滑块
            for i = 1:6
                app.JointSliders(i) = uislider(app.JointPanel);
                app.JointSliders(i).Limits = [-180 180];
                app.JointSliders(i).Position = [20 130-20*i 350 3];
                app.JointSliders(i).ValueChangedFcn = @app.onJointSliderMoved;
                
                app.JointDisplays(i) = uieditfield(app.JointPanel, 'numeric');
                app.JointDisplays(i).Position = [380 120-20*i 50 22];
            end
        end
        
        function createCartesianControlPanel(app)
            app.CartesianPanel = uipanel(app.UIFigure);
            app.CartesianPanel.Title = '笛卡尔空间控制';
            app.CartesianPanel.Position = [250 300 400 140];
            
            labels = {'X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)'};
            for i = 1:6
                uilabel(app.CartesianPanel, 'Text', labels{i},...
                    'Position', [10 100-20*i 60 20]);
                
                if i <= 3
                    app.PosFields(i) = uieditfield(app.CartesianPanel, 'numeric');
                    app.PosFields(i).Position = [80 100-20*i 100 22];
                else
                    app.RotFields(i-3) = uieditfield(app.CartesianPanel, 'numeric');
                    app.RotFields(i-3).Position = [80 100-20*i 100 22];
                end
            end
            
            app.FKButton = uibutton(app.CartesianPanel, 'push',...
                'Text', '正运动学', 'Position', [200 80 80 22],...
                'ButtonPushedFcn', @app.onFKButtonPushed);
            
            app.IKButton = uibutton(app.CartesianPanel, 'push',...
                'Text', '逆运动学', 'Position', [200 40 80 22],...
                'ButtonPushedFcn', @app.onIKButtonPushed);
        end
        
        function createVisualizationTabs(app)
            app.VisualTabGroup = uitabgroup(app.UIFigure, 'Position', [670 50 320 500]);
            
            app.JointPlotTab = uitab(app.VisualTabGroup, 'Title', '关节曲线');
            app.JointAxes = uiaxes(app.JointPlotTab, 'Position', [10 10 300 400]);
            
            app.TrajectoryTab = uitab(app.VisualTabGroup, 'Title', '末端轨迹');
            app.TrajectoryAxes = uiaxes(app.TrajectoryTab, 'Position', [10 10 300 400]);
            
            app.TeachButton = uibutton(app.UIFigure, 'state',...
                'Text', '示教模式', 'Position', [250 250 100 22],...
                'ValueChangedFcn', @app.onTeachButtonValueChanged);
        end
    end
end