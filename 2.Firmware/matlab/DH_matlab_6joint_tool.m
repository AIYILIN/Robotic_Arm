%% 利用标准D-H法建立多轴机器人（含工具坐标系）
clear;
clc;

% DH参数定义
L1 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', 0,     'modified');   
L2 = Link('offset',pi/2,  'd', 0,      'a', 0,      'alpha', pi/2,  'modified');  
L3 = Link('offset',0,     'd', 0,      'a', 199.54, 'alpha', 0,     'modified');
L4 = Link('offset',0,     'd', 171.53, 'a', 95.91,  'alpha', pi/2,  'modified');
L5 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', -pi/2, 'modified');
L6 = Link('offset',0,     'd', 0,      'a',0,       'alpha', pi/2,  'modified');

robot = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'aiyilin');

%% 添加工具坐标系（TCP偏移）
tool_offset = 168.51; % z轴方向偏移
robot.tool = transl(0, 0, tool_offset);

%% 正运动学验证
disp('======= 正运动学计算 =======');
% q_deg = [10, 20, 30, 40, 50, 60];
q_deg = [0, 0, 0, 0, 0, 0];
q_rad = deg2rad(q_deg);

% 保存原始工具坐标系
original_tool = robot.tool;

% 计算腕部中心位置（临时移除工具偏移）
robot.tool = eye(4);
T_wrist = robot.fkine(q_rad);

% 恢复工具坐标系
robot.tool = original_tool;

% 计算工具末端位置（包含工具偏移）
T_tool = robot.fkine(q_rad);

% 显示腕部中心信息
disp('腕部中心位置(x,y,z):');
disp(T_wrist.t');
disp('腕部中心欧拉角（XYZ, 度）(yaw-pitch-roll):');
disp(rotm2eul(T_wrist.R, 'XYZ') * 180/pi);

% 显示工具末端信息
disp('工具末端位置(x,y,z):');
disp(T_tool.t');
disp('工具末端欧拉角（XYZ, 度）(yaw-pitch-roll):');
disp(rotm2eul(T_tool.R, 'XYZ') * 180/pi);


%% 逆运动学：输入工具末端位姿
disp('======= 逆运动学计算 =======');
% target_pos = [200, 200, 150]';       % 目标位置 [x, y, z]
% target_eul_deg = [30, 40, 50]; % 目标欧拉角(yaw-pitch-roll) [X, Y, Z]（度）

target_pos = [340.0400  ,  0.0000 , 295.4500]';       % 目标位置 [x, y, z]
target_eul_deg = [ 0   , 90  , 180]; % 目标欧拉角(yaw-pitch-roll) [X, Y, Z]（度）

% disp('target_eul_deg:');
% disp(target_eul_deg);

% 设置逆解参数
ik_options = {
    'mask', [1 1 1 1 1 1],...  % 控制哪些自由度被考虑[tx ty tz rx ry rz]
    'ilimit', 1000,...         % 最大迭代次数
    'tol', 1e-6,...            % 容差
    'quiet'...                 % 不显示迭代过程
};

% 构造目标位姿的齐次变换矩阵（修正旋转顺序）
target_eul_rad = deg2rad(target_eul_deg);
% disp('target_eul_rad:');
% disp(target_eul_rad);

R_target = eul2rotm(target_eul_rad, 'XYZ');
% disp('R_target:');
% disp(R_target);

T_target = [R_target, target_pos; 0 0 0 1];

% disp('T_target:');
% disp(T_target);

% 执行逆解（需要提供初始猜测）
q0_guess = zeros(1,6);         % 初始猜测角度（通常取零位）                 
q_ik = robot.ikine(T_target, q0_guess, ik_options{:});


% 显示逆解结果
disp('逆解关节角度（弧度）:');
disp(q_ik);
disp('逆解关节角度（度）:');
disp(rad2deg(q_ik));


%% 示教界面（显示工具坐标系）
teach(robot, q_ik); % 以欧拉角形式显示




% 
% %% 动态逆运动学：变换欧拉角中的某个分量
% target_pos = [343, -250, 158]';  % 保持目标位置固定
% base_eul_deg = [0, 40, 50];     % 基准欧拉角 [X,Y,Z]
% 
% % 定义要遍历的参数
% varying_angle_index = 1;        % 选择要变化的欧拉角分量 (1=X, 2=Y, 3=Z)
% angle_range = 0:5:120;           % 角度变化范围 (0°到90°，步长5°)
% 
% % 初始化存储矩阵
% q_ik_sequence = zeros(length(angle_range), 6);  % 存储关节角度
% error_flags = zeros(length(angle_range), 1);     % 错误标记
% 
% % 循环计算逆解
% for i = 1:length(angle_range)
%     % 更新目标欧拉角
%     modified_eul_deg = base_eul_deg;
%     modified_eul_deg(varying_angle_index) = angle_range(i);
% 
%     % 构造目标变换矩阵
%     R_target = eul2rotm(deg2rad(modified_eul_deg), 'XYZ');
%     T_target = [R_target, target_pos; 0 0 0 1];
% 
%     % 求解逆运动学
%     try
%         q_ik = robot.ikine(T_target, 'q0', q0_guess, ik_options{:});
% 
%         % 验证解的有效性
%         T_achieved = robot.fkine(q_ik);
%         pos_error = norm(T_achieved.t - target_pos);
%         if pos_error > 1e-3
%             error('Position error exceeds tolerance');
%         end
% 
%         % 存储有效解
%         q_ik_sequence(i, :) = q_ik;
%         q0_guess = q_ik;  % 使用当前解作为下次初始猜测
% 
%     catch ME
%         warning('无法求解角度 %.1f°: %s', angle_range(i), ME.message);
%         error_flags(i) = 1;
%     end
% end
% 
% %% 结果可视化
% % 绘制关节角度变化曲线
% figure('Name','关节角度变化趋势');
% for j = 1:6
%     subplot(2,3,j);
%     plot(angle_range(~error_flags), q_ik_sequence(~error_flags, j));
%     xlabel('变化角度 (deg)');
%     ylabel(['关节 ', num2str(j), ' 角度 (rad)']);
%     title(['关节 ', num2str(j)]);
%     grid on;
% end
% 
% 
% 
% 
% 
% %% 保存关节角度到Excel
% % 生成带时间戳的唯一文件名
% filename = sprintf('JointAngles_%s.xlsx', datestr(now, 'yyyymmdd_HHMMSS'));
% 
% % 创建数据表格（角度值+关节角度）
% valid_indices = ~error_flags; % 获取有效解的索引
% 
% % 将弧度转换为度数
% q_ik_deg = rad2deg(q_ik_sequence(valid_indices, :));
% 
% % 构建数据矩阵
% data_matrix = [angle_range(valid_indices)', q_ik_deg]; 
% 
% % 定义列标题
% headers = {'角度变化值(deg)', '关节1(deg)', '关节2(deg)', '关节3(deg)',...
%            '关节4(deg)', '关节5(deg)', '关节6(deg)'};
% 
% % 创建表格对象
% data_table = array2table(data_matrix, 'VariableNames', headers);
% 
% % 写入Excel文件
% writetable(data_table, filename);
% 
% % 显示保存路径
% fprintf('数据已保存至文件: %s\\%s\n', pwd, filename);
% 
% 
% % 动态演示运动轨迹
% figure('Name','动态运动演示');
% robot.plot(q_ik_sequence(1, :));  % 初始位置
% for i = 2:length(angle_range)
%     if ~error_flags(i)
%         robot.plot(q_ik_sequence(i, :));
%         pause(0.1);  % 控制动画速度
%     end
% end