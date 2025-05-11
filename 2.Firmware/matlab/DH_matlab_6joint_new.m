%% 利用标准D-H法建立多轴机器人
clear;
clc;
% DH参数定义（重点检查L2的a参数是否正确）
L1 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', 0,     'modified');   
L2 = Link('offset',pi/2,  'd', 0,      'a', 0,      'alpha', pi/2,  'modified');  
L3 = Link('offset',0,     'd', 0,      'a', 199.54, 'alpha', 0,     'modified');
L4 = Link('offset',0,     'd', 171.53, 'a', 95.91,  'alpha', pi/2,  'modified');
L5 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', -pi/2, 'modified');
L6 = Link('offset',0,     'd', 168.51,      'a',0,       'alpha', pi/2,  'modified');
% 168.51
robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','aiyilin');

%% 机械臂变换矩阵验证（各关节零位）
disp('======= 各关节零位齐次变换矩阵 =======');
T1 = robot.links(1).A(0);
T2 = robot.links(2).A(0);
T3 = robot.links(3).A(0);
T4 = robot.links(4).A(0);
T5 = robot.links(5).A(0);
T6 = robot.links(6).A(0);
disp('T1:'); disp(T1);
disp('T2:'); disp(T2);
disp('T3:'); disp(T3);
disp('T4:'); disp(T4);
disp('T5:'); disp(T5);
disp('T6:'); disp(T6);

%% 正运动学求解
disp('======= 正运动学计算 =======');
% q_deg = [20, 40, 60, 80, 100, 120];       % 输入：各关节角度(度)
q_deg = [10, 20, 30, 40, 50, 60];       % 输入：各关节角度(度)
% q_deg = [0, 0, 0, 0, 0, 0];       % 输入：各关节角度(度)
q_rad = deg2rad(q_deg);                 % 转换为弧度
T = robot.fkine(q_rad);                 % 正运动学计算
disp('末端位姿矩阵：');
disp(T.T);                              % 显示齐次变换矩阵

% 分解显示位置和姿态
disp('末端位置（mm）:');
disp(T.t');
disp('末端旋转矩阵：');
disp(T.R);
disp('XYZ欧拉角(yaw-pitch-roll)：');
disp(rotm2eul(T.R,"XYZ")*180/pi);


%% 逆运动学求解示例
disp('======= 逆运动学计算 =======');
% 用正向运动学结果作为目标位姿
T_target = T;  

% 设置逆解参数
ik_options = {
    'mask', [1 1 1 1 1 1],...  % 控制哪些自由度被考虑[tx ty tz rx ry rz]
    'ilimit', 1000,...         % 最大迭代次数
    'tol', 1e-6,...            % 容差
    'quiet'...                 % 不显示迭代过程
};

% 执行逆解（需要提供初始猜测）
q0_guess = zeros(1,6);         % 初始猜测角度（通常取零位）                 
q_ik = robot.ikine(T_target, q0_guess, ik_options{:});

% 显示逆解结果
disp('逆解关节角度（弧度）:');
disp(q_ik);
disp('逆解关节角度（度）:');
disp(rad2deg(q_ik));

%% 对比正逆解结果误差
disp('======= 结果验证 =======');
T_ik = robot.fkine(q_ik);      % 用逆解结果重新计算位姿
position_error = norm(T.t - T_ik.t);  % 位置误差
orientation_error = norm(T.R - T_ik.R);% 姿态误差
disp(['位置误差：', num2str(position_error), ' mm']);
disp(['姿态误差：', num2str(orientation_error)]);

%% 人机交互界面
robot.display();               % 展示DH参数
disp('正在启动示教界面...');
teach(robot);                  % 调出示教滑块