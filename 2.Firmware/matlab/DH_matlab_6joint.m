%%利用标准D-H法建立多轴机器人
clear;
clc;
L1 = Link('offset',0,     'd', 500,        'a', 500,   'alpha', -pi/2);   
L2 = Link('offset',-pi/2, 'd', 0,          'a', 1000,  'alpha', 0);
L3 = Link('offset',0,     'd', 0,          'a', 500,   'alpha',  -pi/2);
L4 = Link('offset',0,     'd', 1000,       'a', 0,     'alpha', pi/2);
L5 = Link('offset',0,     'd', 0,          'a', 0,     'alpha', -pi/2);
L6 = Link('offset',0,     'd', 1000,       'a', 0,     'alpha', 0);

% L1 = Link('offset',0,     'd', 500,        'a', 500,   'alpha', -pi/2);   
% L2 = Link('offset',-pi/2, 'd', 0,          'a', 1000,  'alpha', 0);
% L3 = Link('offset',0,     'd', 0,          'a', 500,   'alpha',  -pi/2);
% L4 = Link('offset',0,     'd', 1000,       'a', 0,     'alpha', pi/2);
% L5 = Link('offset',0,     'd', 0,          'a', 0,     'alpha', -pi/2);
% L6 = Link('offset',0,     'd', 1000,       'a', 0,     'alpha', 0);

L1.qlim = [-pi,pi];%利用qlim设置每个关节的旋转角度范围
robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','aiyilin');   %SerialLink 类函数
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


%% 普通机器人的示教展示

robot.display();%展示出机器人的信息
teach(robot);%调出示教滑块
% 
% % MATLAB验证代码
% target = [200 200 300]; % 同C代码中的目标
% q_ik = robot.ikine(transl(target), 'q0', [0 0 0 0], 'mask', [1 1 1 0 0 0]);
% disp(q_ik/3.1415926*180);

 
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
