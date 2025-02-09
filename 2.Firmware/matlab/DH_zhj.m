%% MATLAB验证代码（与C++参数对应）
clear;
clc;

L_BASE = 500;
D_BASE = 500;
L_ARM = 1000;
D_ELBOW = 200;
L_FORE = 500;
L_WRIST = 500;

L(1) = Link('alpha', -pi/2,  'a', D_BASE ,  'd',  L_BASE, 'offset',     0);
L(2) = Link('alpha', 0,      'a', L_ARM,    'd',       0, 'offset',  -pi/2);
L(3) = Link('alpha', pi/2,   'a', 0,        'd',  D_ELBOW, 'offset',  pi/2);
L(4) = Link('alpha', -pi/2,  'a', 0,        'd',  L_FORE, 'offset',     0);
L(5) = Link('alpha', pi/2,   'a', 0,        'd',       0, 'offset',     0);
L(6) = Link('alpha', 0,      'a', 0 ,       'd', L_WRIST, 'offset',     0);
robot = SerialLink(L, 'name', 'DH6-Arm');

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
robot.display();%展示出机器人的信息
robot.teach();