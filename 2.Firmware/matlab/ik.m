% 步骤1：定义目标位姿（位置和欧拉角）
position = [-30.983, -5.463, 380.556]; % 单位：mm
eul_deg = [29.5, 2.8, -81.8];% 欧拉角（yaw-Z, pitch-Y, roll-X）单位：度

% 步骤2：将欧拉角转换为旋转矩阵（XYZ顺序）
eul_rad = deg2rad(eul_deg);
R = eul2rotm(eul_rad, 'XYZ');         % 生成3x3旋转矩阵
disp(R)
% 构建齐次变换矩阵
coord = eye(4);
coord(1:3, 1:3) = R;
coord(1:3, 4) = position';

% 步骤3：定义DH参数（d, a, alpha）
d = [0, 0, 0, 171.53, 0, 0];         % 单位：mm
a = [0, 0, 199.54, 95.91, 0, 0];     % 单位：mm
alpha_deg = [0, 90, 0, 90, -90, 90]; % 单位：度
theta_deg = [0, 90, 0, 0, 0, 0];     % 单位：度
alpha_rad = deg2rad(alpha_deg);      % 转换为弧度

% 步骤4：计算关节角度
T = calculate_theta(coord, d, a, alpha_rad, theta_deg);
% disp(T);

% 步骤5：将结果转换为度数并显示
T_deg = rad2deg(T);
disp('共解出8组解（度）：');

% 自定义格式化输出
for i = 1:size(T_deg,1)
    % 生成解编号
    header = sprintf('解%-2d：', i); 
    
    % 格式化每个关节角度（固定宽度9字符，保留4位小数）
    formatted_angles = arrayfun(@(x) sprintf('% 9.4f', x), T_deg(i,:), 'UniformOutput', false);
    
    % 组合输出行
    output_line = [header, strjoin(formatted_angles, '   ')];
    
    % 根据解号调整缩进
    if i < 10
        output_line = [' ', output_line];  % 单数解号增加缩进
    end
    
    disp(output_line);
end