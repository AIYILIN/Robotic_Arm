%%利用标准D-H法建立多轴机器人
clear;
clc;
L1 = Link('d', 0,       'a', 0,       'alpha', pi/2,'offset',0);    %Link 类函数;offset建立初始的偏转角
L2 = Link('d', 16.982,  'a', 202.596, 'alpha', 0,   'offset',pi/2);
L3 = Link('d', -33.7,   'a', 211.366, 'alpha', 0,   'offset',0);
L4 = Link('d', 27.7,    'a', 215.389, 'alpha', pi/2,'offset',0);
% L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2,'offset',0);
% L6 = Link('d', 10, 'a', 0, 'alpha', 0,   'offset',0);

L1.qlim = [-pi/2,pi];%利用qlim设置每个关节的旋转角度范围
robot=SerialLink([L1,L2,L3,L4],'name','aiyilin');   %SerialLink 类函数
T1 = robot.links(1).A(0);
T2 = robot.links(2).A(0);
T3 = robot.links(3).A(0);
T4 = robot.links(4).A(0);
disp('T1:'); disp(T1);
disp('T2:'); disp(T2);
disp('T3:'); disp(T3);
disp('T4:'); disp(T4);

%% 普通机器人的示教展示

robot.display();%展示出机器人的信息
teach(robot);%调出示教滑块

% MATLAB验证代码
target = [200 200 300]; % 同C代码中的目标
q_ik = robot.ikine(transl(target), 'q0', [0 0 0 0], 'mask', [1 1 1 0 0 0]);
disp(q_ik/3.1415926*180);

 
%% 展示当六个关节角为000000时对应的姿态

% theta=[0 0 0 0 ];
% robot.plot(theta);   
% p_1=robot.fkine(theta);

%% 机器人的正解函数

% theta1=[pi/4,-pi/3,pi/6,pi/4];
% robot.plot(theta1);
% p0=robot.fkine(theta);
% p1=robot.fkine(theta1);

% 机器人的逆解

% Pos_x=180;Pos_y=180;Pos_z=180;
%  p = [1 0 0 Pos_x;
%       0 1 0 Pos_y;
%       0 0 1 Pos_z;
%       0 0 0 1];%已知空间中的位姿q
% mask = [1 1 1 0 0 0];
% q=ikine(robot,p,'mask',mask);   %ikine逆解函数，根据末端位姿p，求解出关节角q
% robot.plot(q);%输出机器人模型，后面的三个角为输出时的theta姿态
% disp(q);
