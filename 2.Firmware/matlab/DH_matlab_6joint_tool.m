%% 主程序入口
clear; clc;

%% DH参数定义，机械臂创建
L1 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', 0,     'modified');   
L2 = Link('offset',pi/2,  'd', 0,      'a', 0,      'alpha', pi/2,  'modified');  
L3 = Link('offset',0,     'd', 0,      'a', 199.54, 'alpha', 0,     'modified');
L4 = Link('offset',0,     'd', 171.53, 'a', 95.91,  'alpha', pi/2,  'modified');
L5 = Link('offset',0,     'd', 0,      'a', 0,      'alpha', -pi/2, 'modified');
L6 = Link('offset',0,     'd', 0,      'a',0,       'alpha', pi/2,  'modified');
robot = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'aiyilin');

% 添加工具坐标系（TCP偏移）
tool_offset = 168.51; % z轴方向偏移
robot.tool = transl(0, 0, tool_offset)

%% 参数
% 正运动六个关节的角度
q_deg = [10, 20, 30, 40, 50, 60];
% q_deg = [0, 0, 0, 0, 0, 0];

% 逆运动的位置 [x, y, z]，欧拉角(yaw-pitch-roll) [X, Y, Z]（度）
% target_pos = [340.0400  ,  0.0000 , 295.4500]';       
% target_eul_deg = [ 0   , 90  , 180];                  

target_pos = [-22.6078  -88.2414  527.0931]';       
target_eul_deg = [ 29.4619    2.8488  -81.7780]; 

% 设置逆解参数
ik_options = {
    'mask', [1 1 1 1 1 1],...  % 控制哪些自由度被考虑[tx ty tz rx ry rz]
    'ilimit', 1000,...         % 最大迭代次数
    'tol', 1e-6,...            % 容差
    'quiet'...                 % 不显示迭代过程
};

q0_guess = zeros(1,6); % 初始猜测角度（通常取零位）   

% 动态逆运动学，变换欧拉角
eular_base_pos = [343, -250, 158]';  % 保持目标位置固定
eular_base_eul_deg = [0, 40, 50];     % 基准欧拉角 [X,Y,Z]
eular_varying_angle_index = 1;        % 选择要变化的欧拉角分量 (1=X, 2=Y, 3=Z)
eular_angle_range = 0:5:120;           % 角度变化范围 (0°到90°，步长5°)

% 动态逆运动学，变换位姿
% 每一行表示一种位姿状态，分别是x, y, z（mm），yaw，pitch，roll（度），有几行表示连续变换几次，按照顺序来，第一行第二行这样变换
change_array = [
 250 , 200,  200 , 0, 90, -180;
 250 ,   0,  200 , 0, 90, -180;
 250 ,-200,  200 , 0, 90, -180;];
pos_base_pos = [340 ,0, 295]';  % 基准位置
pos_base_eul_deg = [0, 90, -180];     % 基准欧拉角
pos_pos_range = 1;              % 位置变化步进
pos_angle_range = 1;           % 角度变化步进
%% 主函数
arm_fk(robot, q_deg);%正运动
arm_ik(robot, target_pos, target_eul_deg, ik_options, q0_guess);%逆运动
% teach(robot); % 示教器
% arm_vary_euler_component(robot,eular_base_pos, eular_base_eul_deg, eular_varying_angle_index, eular_angle_range, ik_options, q0_guess);
arm_vary_pos_component(robot, pos_base_pos, pos_base_eul_deg, change_array, ik_options, q0_guess, pos_pos_range, pos_angle_range);

%% 函数封装：正运动学计算
function [T_wrist,T_tool] = arm_fk(robot, q_deg)

    disp('======= 正运动学计算 =======');
    q_rad = deg2rad(q_deg);%角度转换弧度 
    original_tool = robot.tool;% 保存原始工具坐标系
    robot.tool = eye(4); % 计算腕部中心位置（临时移除工具偏移）
    T_wrist = robot.fkine(q_rad);
    robot.tool = original_tool;% 恢复工具坐标系 
    T_tool = robot.fkine(q_rad);% 计算工具末端位置（包含工具偏移）
    
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

end

%% 逆运动学：输入工具末端位姿
function [q_ik] = arm_ik(robot, target_pos, target_eul_deg, ik_options, q0_guess)

    disp('======= 逆运动学计算 =======');
    target_eul_rad = deg2rad(target_eul_deg);
    R_target = eul2rotm(target_eul_rad, 'XYZ');
    T_target = [R_target, target_pos; 0 0 0 1];% 构造目标位姿的齐次变换矩阵
                  
    q_ik = robot.ikine(T_target, q0_guess, ik_options{:});% 执行逆解（需要提供初始猜测）
    
    disp('逆解关节角度（弧度）:');
    disp(q_ik);
    disp('逆解关节角度（度）:');
    disp(rad2deg(q_ik));

end

%% 逆运动学：动态变换欧拉角
function [q_ik_sequence, error_flags] = arm_vary_euler_component(robot, eular_base_pos, eular_base_eul_deg, eular_varying_angle_index, eular_angle_range, ik_options, q0_guess)

    q_ik_sequence = zeros(length(eular_angle_range), 6);  % 存储关节角度
    error_flags = zeros(length(eular_angle_range), 1);     % 错误标记
    
    for i = 1:length(eular_angle_range)% 循环计算逆解
        modified_eul_deg = eular_base_eul_deg;% 更新目标欧拉角
        modified_eul_deg(eular_varying_angle_index) = eular_angle_range(i);
   
        R_target = eul2rotm(deg2rad(modified_eul_deg), 'XYZ');% 构造目标变换矩阵
        T_target = [R_target, eular_base_pos; 0 0 0 1];
    
        try     % 求解逆运动学
            q_ik = robot.ikine(T_target, 'q0', q0_guess, ik_options{:});
       
            T_achieved = robot.fkine(q_ik);% 验证解的有效性
            pos_error = norm(T_achieved.t - eular_base_pos);
            if pos_error > 1e-3
                error('Position error exceeds tolerance');
            end

            q_ik_sequence(i, :) = q_ik;% 存储有效解
            q0_guess = q_ik;  % 使用当前解作为下次初始猜测
    
        catch ME
            warning('无法求解角度 %.1f°: %s', eular_angle_range(i), ME.message);
            error_flags(i) = 1;
        end
    end

    % 结果可视化
    figure('Name','关节角度变化趋势');
    for j = 1:6
        subplot(2,3,j);
        plot(eular_angle_range(~error_flags), q_ik_sequence(~error_flags, j));
        xlabel('变化角度 (deg)');
        ylabel(['关节 ', num2str(j), ' 角度 (rad)']);
        title(['关节 ', num2str(j)]);
        grid on;
    end
   
    % 创建数据表格（角度值+关节角度）
    filename = sprintf('vary_eular_%s.xlsx', datestr(now, 'yyyymmdd_HHMMSS'));% 生成带时间戳的唯一文件名
    valid_indices = ~error_flags; % 获取有效解的索引
    q_ik_deg = rad2deg(q_ik_sequence(valid_indices, :));% 将弧度转换为度数
    data_matrix = [eular_angle_range(valid_indices)', q_ik_deg]; % 构建数据矩阵
    
    % 定义列标题
    headers = {'角度变化值(deg)', '关节1(deg)', '关节2(deg)', '关节3(deg)',...
               '关节4(deg)', '关节5(deg)', '关节6(deg)'};  
    data_table = array2table(data_matrix, 'VariableNames', headers);% 创建表格对象
    writetable(data_table, filename);% 写入Excel文件
    fprintf('数据已保存至文件: %s\\%s\n', pwd, filename);% 显示保存路径
    
    % 动态演示运动轨迹
    figure('Name','动态运动演示');
    robot.plot(q_ik_sequence(1, :));  % 初始位置
    for i = 2:length(eular_angle_range)
        if ~error_flags(i)
            robot.plot(q_ik_sequence(i, :));
            pause(0.1);  % 控制动画速度
        end
    end

end


%% 逆运动学：动态变换位姿（带中间插值步进）
function [q_ik_sequence, error_flags] = arm_vary_pos_component(robot, pos_base_pos, pos_base_eul_deg, change_array, ik_options, q0_guess, pos_pos_range, pos_angle_range)
    
% 步骤1：生成完整路径（包含基准位置和所有中间插值点）
    full_path = []; 
    current_pos = pos_base_pos';       % 当前位姿：位置
    current_eul = pos_base_eul_deg;     % 当前位姿：欧拉角
    
    % 将基准位置作为第一个点
    full_path = [full_path; [current_pos, current_eul]]; 
    
    % 遍历change_array的每一行，生成插值路径
    for i = 1:size(change_array,1)
        target_pos = change_array(i,1:3);     % 目标位置 [x,y,z]
        target_eul = change_array(i,4:6);    % 目标欧拉角 [yaw,pitch,roll]
        
        % 计算位置和角度的总变化量
        delta_pos = target_pos - current_pos;
        delta_eul = target_eul - current_eul;
        
        % 计算需要插值的步数（取位置和角度中变化最大的维度）
        num_pos_steps = ceil(norm(delta_pos) / pos_pos_range);
        num_angle_steps = ceil(max(abs(delta_eul)) / pos_angle_range);
        num_steps = max(num_pos_steps, num_angle_steps);
        
        % 线性插值生成中间点
        interp_pos = linspaceVectors(current_pos, target_pos, num_steps);
        interp_eul = linspaceAngles(current_eul, target_eul, num_steps);
        
        % 将中间点加入完整路径（跳过第一个点避免重复）
        full_path = [full_path; [interp_pos(2:end,:), interp_eul(2:end,:)]];
        
        % 更新当前位姿
        current_pos = target_pos;
        current_eul = target_eul;
    end
    
    % 步骤2：逆运动学求解所有路径点
    num_points = size(full_path,1);
    q_ik_sequence = zeros(num_points,6);
    error_flags = zeros(num_points,1);
    
    for i = 1:num_points
        target_pos = full_path(i,1:3)';
        target_eul = full_path(i,4:6);
        
        % 构造目标变换矩阵
        R_target = eul2rotm(deg2rad(target_eul), 'XYZ');
        T_target = [R_target, target_pos; 0 0 0 1];
        
        % 求解逆运动学
        try
            q_ik = robot.ikine(T_target, 'q0', q0_guess, ik_options{:});
            
            % 验证解
            T_achieved = robot.fkine(q_ik);
            pos_error = norm(T_achieved.t - target_pos);
            if pos_error > 1e-3
                error('Position error exceeds tolerance');
            end
            
            q_ik_sequence(i,:) = q_ik;
            q0_guess = q_ik; % 更新初始猜测
            
        catch ME
            warning('路径点 %d 求解失败: %s', i, ME.message);
            error_flags(i) = 1;
        end
    end
    
    % 步骤3：结果可视化
    % 关节角度变化曲线
    figure('Name','关节角度变化趋势');
    time_steps = 1:num_points;
    valid_steps = ~error_flags;
    for j = 1:6
        subplot(2,3,j);
        plot(time_steps(valid_steps), q_ik_sequence(valid_steps, j), 'o-');
        xlabel('路径点序号');
        ylabel(['关节 ', num2str(j), ' 角度 (rad)']);
        grid on;
    end
    
    % 步骤4：Excel保存（仅关节角度）
    filename = sprintf('joint_angles_%s.xlsx', datestr(now, 'yyyymmdd_HHMMSS'));
    
    % 提取有效数据
    valid_steps = find(~error_flags);          % 有效步骤序号
    valid_angles_deg = rad2deg(q_ik_sequence(~error_flags, :)); % 转为角度
    
    % 构建数据表格 (步骤序号 + 六个关节角度)
    data_matrix = [valid_steps, valid_angles_deg]; 
    
    % 列标题定义
    headers = {'Step', 'Joint1(deg)', 'Joint2(deg)', 'Joint3(deg)',...
               'Joint4(deg)', 'Joint5(deg)', 'Joint6(deg)'};
    
    % 写入Excel
    data_table = array2table(data_matrix, 'VariableNames', headers);
    writetable(data_table, filename);
    disp(['关节角度数据已保存至: ' fullfile(pwd, filename)]);
    
    % 步骤5：动态演示（带插值步进）
    figure('Name','动态位姿变换演示');
    robot.plot(q_ik_sequence(1,:));
    for i = 2:num_points
        if ~error_flags(i)
            robot.plot(q_ik_sequence(i,:));
            pause(0.001); % 调整动画速度
        end
    end
end

%% 辅助函数：线性插值向量（处理位置）
function interp = linspaceVectors(start, finish, num_steps)
    t = linspace(0,1,num_steps)';
    interp = (1 - t) .* start + t .* finish;
end

%% 辅助函数：角度插值（处理欧拉角环绕）
function interp = linspaceAngles(start_deg, finish_deg, num_steps)
    start_rad = deg2rad(start_deg);
    finish_rad = deg2rad(finish_deg);
    
    % 处理角度环绕（例如从350°到10°应插值为-20°而不是+340°）
    diff = finish_rad - start_rad;
    diff = mod(diff + pi, 2*pi) - pi; % 保证在[-pi, pi]范围内
    
    t = linspace(0,1,num_steps)';
    interp_rad = start_rad + t .* diff;
    interp = rad2deg(interp_rad);
end
%%