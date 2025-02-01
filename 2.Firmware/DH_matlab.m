%%利用标准D-H法建立多轴机器人
clear;
clc;
L1 = Link('d', -59.4, 'a', 0,       'alpha', pi/2,'offset',0);    %Link 类函数;offset建立初始的偏转角
L2 = Link('d', 46.3,  'a', 202.596, 'alpha', 0,   'offset',0);
L3 = Link('d', -50,   'a', 211.345, 'alpha', 0,   'offset',0);
L4 = Link('d', 0,     'a', 215.389, 'alpha', pi/2,'offset',90);
% L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2,'offset',0);
% L6 = Link('d', 10, 'a', 0, 'alpha', 0,   'offset',0);

L1.qlim = [-pi/2,pi];%利用qlim设置每个关节的旋转角度范围
robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','S725');   %SerialLink 类函数

%% 普通机器人的示教展示

robot.display();%展示出机器人的信息
teach(robot);%调出示教滑块
