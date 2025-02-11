%% 利用标准D-H法建立多轴机器人
clear;
clc;
L1 = Link('offset',0,     'd', 50,        'a', 50,   'alpha', -pi/2);   
L2 = Link('offset',-pi/2, 'd', 0,          'a', 100,  'alpha', 0);
L3 = Link('offset',0,     'd', 0,          'a', 50,   'alpha',  -pi/2);
L4 = Link('offset',0,     'd', 100,       'a', 0,     'alpha', pi/2);
L5 = Link('offset',0,     'd', 0,          'a', 0,     'alpha', -pi/2);
L6 = Link('offset',0,     'd', 100,       'a', 0,     'alpha', 0);

robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','aiyilin');

%% 计算所有欧拉角表示形式
q = [pi/6, pi/6, pi/6, pi/6, pi/6, pi/6]; % 关节角度（弧度）
T = robot.fkine(q);                        % 计算变换矩阵

% 提取位置（单位：毫米）
position_mm = T.t; 

% 定义所有欧拉角序列类型
euler_sequences = {'ZYX', 'ZYZ', 'XYZ', 'ZXY', 'ZXZ',...
                   'YXZ', 'YXY', 'YZX', 'YZY', 'XYX',...
                   'XZY', 'XZX'};

% 计算并显示所有欧拉角
disp('末端位置(mm):');
disp(position_mm');

for i = 1:length(euler_sequences)
    seq = euler_sequences{i};
    eul_deg = rotm2eul(T.R, seq) * 180/pi;
    
    fprintf('%s 欧拉角 (度):\n', seq);
    disp(eul_deg);
end


% %% 增强版示教界面（显示姿态信息）
% teach(robot, 'eul', 'zyx'); % 在示教界面显示ZYX欧拉角