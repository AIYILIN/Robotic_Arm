function solutions = inverse_kinematics(T_target)
    % 输入：目标位姿 T_target (4x4)
    % 输出：所有可能的关节角度解（Nx6矩阵，单位弧度）

    % --- 提取目标位置和姿态 ---
    p_target = T_target(1:3, 4);
    R_target = T_target(1:3, 1:3);

    % --- 腕点位置计算（此处简化为p_target，因L6无偏移）---
    p_wrist = p_target;

    % --- 求解θ1 ---
    theta1 = [atan2(p_wrist(2), p_wrist(1)), atan2(p_wrist(2), p_wrist(1)) + pi];

    % --- 对每个θ1求解θ2、θ3 ---
    solutions = [];
    for theta1_val = theta1
        % 投影到平面
        r = sqrt(p_wrist(1)^2 + p_wrist(2)^2);
        z = p_wrist(3);

        % 几何参数
        L2 = 199.54;
        L3 = sqrt(95.91^2 + 171.53^2);

        % 余弦定理解θ3
        D = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
        theta3_val = [acos(D), -acos(D)]; % 两种可能的肘部构型

        for theta3 = theta3_val
            % 解θ2
            theta2 = atan2(z, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3));
            
            % 存储前三轴解
            theta_front = [theta1_val, theta2, theta3];
            
            % --- 求解后三轴θ4、θ5、θ6 ---
            % 计算R_3^0
            T1 = L1.A(theta_front(1));
            T2 = L2.A(theta_front(2));
            T3 = L3.A(theta_front(3));
            R_3_0 = (T1 * T2 * T3).R;

            % 计算R_6^3 = (R_3^0)^T * R_target
            R_6_3 = R_3_0.' * R_target;

            % Z-Y-Z欧拉角分解
            [theta4, theta5, theta6] = eul_zyz(R_6_3);

            % 存储所有可能的后三轴解
            theta_back = [theta4, theta5, theta6;
                          theta4 + pi, -theta5, theta6 + pi]; % 腕部翻转

            % 合并解
            for k = 1:size(theta_back, 1)
                sol = [theta_front, theta_back(k, :)];
                solutions = [solutions; sol];
            end
        end
    end

    % 去除超出关节限位的解
    solutions = filter_joint_limits(solutions);
end

function [theta4, theta5, theta6] = eul_zyz(R)
    % Z-Y-Z欧拉角分解
    theta5 = atan2(sqrt(R(3,1)^2 + R(3,2)^2), R(3,3));
    theta4 = atan2(R(2,3), R(1,3));
    theta6 = atan2(R(3,2), -R(3,1));
    
    % 处理奇异情况（theta5=0或pi）
    if abs(theta5) < eps
        theta4 = 0;
        theta6 = atan2(R(1,2), R(1,1));
    elseif abs(theta5 - pi) < eps
        theta4 = 0;
        theta6 = -atan2(R(1,2), R(1,1));
    end
end


% 目标位姿（示例）
T_target = robot.fkine([deg2rad(10), deg2rad(20), deg2rad(30), deg2rad(40), deg2rad(50), deg2rad(60)]).T;

% 计算所有逆解
solutions = inverse_kinematics(T_target);

% 显示解
disp('所有逆解（弧度）:');
disp(solutions);
disp('对应的角度（度）:');
disp(rad2deg(solutions));