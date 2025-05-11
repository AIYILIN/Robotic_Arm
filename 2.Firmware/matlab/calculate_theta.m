% 主函数：计算6自由度机械臂逆运动学解
% 输入：
%   coord - 目标位姿矩阵（4x4齐次变换矩阵）
%   d     - DH参数中的d值数组（1x6）
%   a     - DH参数中的a值数组（1x6）
%   alpha - DH参数中的alpha值数组（1x6，单位弧度）
% 输出：
%   T     - 所有可能的关节角度解（Nx6矩阵，单位弧度）
function T = calculate_theta(coord, d, a, alpha, theta)
    % 提取目标位置坐标（x,y,z）
    p = [coord(1,4) coord(2,4) coord(3,4)];
    
    % 步骤1：计算关节3的角度（可能有两个解）
    theta3 = calculate_theta3(p, d, a, alpha);
    
    % 步骤2：计算与theta3相关的中间参数f1和f2
    % f1/f2用于后续关节2的计算，分别对应theta3的两个解
    % f1对应theta3(1), f2对应theta3(2)
    f1(1) = a(4) * cos(theta3(1)) + d(4) * sin(alpha(4)) * sin(theta3(1)) + a(3);
    f1(2) = a(4) * cos(alpha(3)) * sin(theta3(1)) - d(4) * sin(alpha(4)) * cos(alpha(3)) * cos(theta3(1)) ...
          - d(4) * sin(alpha(3)) * cos(alpha(4)) - d(3) * sin(alpha(3));
    
    % 步骤3：计算关节2的角度（每个theta3解对应两个theta2解）
    theta2_1 = calculate_theta2(p(3), f1, alpha);
    
    % 对第二个theta3解重复上述过程
    f2(1) = a(4) * cos(theta3(2)) + d(4) * sin(alpha(4)) * sin(theta3(2)) + a(3);
    f2(2) = a(4) * cos(alpha(3)) * sin(theta3(2)) - d(4) * sin(alpha(4)) * cos(alpha(3)) * cos(theta3(2)) ...
          - d(4) * sin(alpha(3)) * cos(alpha(4)) - d(3) * sin(alpha(3));
    theta2_2 = calculate_theta2(p(3), f2, alpha);

    % 计算f1和f2的第三个分量（用于关节1计算）
    f1(3) = a(4) * sin(alpha(3)) * sin(theta3(1)) - d(4) * sin(alpha(4)) * sin(alpha(3)) * cos(theta3(1)) ...
          + d(4) * cos(alpha(3)) * cos(alpha(4)) + d(3) * cos(alpha(3));
    f2(3) = a(4) * sin(alpha(3)) * sin(theta3(2)) - d(4) * sin(alpha(4)) * sin(alpha(3)) * cos(theta3(2)) ...
          + d(4) * cos(alpha(3)) * cos(alpha(4)) + d(3) * cos(alpha(3));

    % 步骤4：计算关节1的角度（每个组合对应一个解）
    theta1 = calculate_theta1(p, f1, theta2_1(1), alpha);
    T1(1,:) = [theta1 theta2_1(1) theta3(1)];
    theta1 = calculate_theta1(p, f1, theta2_1(2), alpha);
    T1(2,:) = [theta1 theta2_1(2) theta3(1)];
    theta1 = calculate_theta1(p, f2, theta2_2(1), alpha);
    T1(3,:) = [theta1 theta2_2(1) theta3(2)];
    theta1 = calculate_theta1(p, f2, theta2_2(2), alpha);
    T1(4,:) = [theta1 theta2_2(2) theta3(2)];

    % 步骤5：计算最后三个关节角度（4-6关节）
    R06 = [coord(1,1:3); coord(2,1:3); coord(3,1:3)];  % 提取目标旋转矩阵
    j = 1;
    for i = 1:4
        % 对每个前三个关节的解，计算后三个关节
        
        disp('print');
        disp(i);
        [T2, num] = calculate_theta456(T1(i,1), T1(i,2), T1(i,3), R06, alpha);
        if num == 1
            T(j,:) = [T1(i,:) T2(1,:)];
            j = j + 1;
        else
            % 当存在两种后三轴解时，分别存储
            T(j,:) = [T1(i,:) T2(1,:)];
            T(j+1,:) = [T1(i,:) T2(2,:)];
            j = j + 2;
        end
    end

    % 应用全局关节偏移修正
    for i = 1:size(T,1)
        % 对每个关节进行独立偏移
        for joint = 1:6
            T(i,joint) = T(i,joint) - deg2rad(theta(joint));
            
            % 角度归一化处理（保持[-π, π]）
            T(i,joint) = mod(T(i,joint) + pi, 2*pi) - pi;
            
            % 消除微小数值误差（例如将-π转换为π）
            if abs(T(i,joint) - pi) < 1e-6
                T(i,joint) = pi;
            elseif abs(T(i,joint) + pi) < 1e-6
                T(i,joint) = -pi;
            end
        end
    end

end

% 关节3角度计算函数
% 原理：基于球面交点法求解theta3
% 输入参数：
%   p - 目标位置坐标[x,y,z]
% 返回：theta3的两个可能解（弧度）
function T = calculate_theta3(p, d, a, alpha)
    r = p(1)^2 + p(2)^2 + p(3)^2;  % 目标点模长平方
    m = 2 * a(3) * d(4) * sin(alpha(4));  % 方程系数
    n = 2 * a(3) * a(4);                 % 方程系数
    e = r - (a(4)^2 + d(4)^2 + d(3)^2 + a(3)^2 + 2 * d(4) * d(3) * cos(alpha(4)));

    % 解三角方程 m*cosθ3 + n*sinθ3 = e
    T1 = equation_cal(m, n, e);
    
    % 将解转换为角度形式
    sin_theta3(1) = T1(1,1);
    cos_theta3(1) = T1(1,2);
    sin_theta3(2) = T1(2,1);
    cos_theta3(2) = T1(2,2);
    T = [atan2(sin_theta3(1), cos_theta3(1)), atan2(sin_theta3(2), cos_theta3(2))];
end

% 关节2角度计算函数
% 原理：通过几何关系建立方程求解
% 输入参数：
%   g3 - 目标z坐标
%   f  - 来自前一步计算的中间参数
% 返回：theta2的两个可能解（弧度）
function T = calculate_theta2(g3, f, alpha)
    e = g3;  % 方程常数项
    m = f(1) * sin(alpha(2));  % 方程系数
    n = f(2) * sin(alpha(2));  % 方程系数
    
    % 解方程 m*cosθ2 + n*sinθ2 = e
    T1 = equation_cal(m, n, e);
    
    % 结果转换
    sin_theta2 = [T1(1,1), T1(2,1)];
    cos_theta2 = [T1(1,2), T1(2,2)];
    T = [atan2(sin_theta2(1), cos_theta2(1)), atan2(sin_theta2(2), cos_theta2(2))];
end

% 通用三角方程求解器
% 解方程：m*cosθ + n*sinθ = e
% 返回：2组可能的[sinθ, cosθ]解
function T = equation_cal(m, n, e)
    discriminant = m^2 + n^2 - e^2;  % 判别式
    sqrt_disc = sqrt(discriminant);
    
    % 第一组解
    T(1,1) = (e - (n*(e*n + m*sqrt_disc))/(m^2 + n^2))/m;  % sinθ1
    T(1,2) = (e*n + m*sqrt_disc)/(m^2 + n^2);             % cosθ1
    
    % 第二组解
    T(2,1) = (e - (n*(e*n - m*sqrt_disc))/(m^2 + n^2))/m;  % sinθ2
    T(2,2) = (e*n - m*sqrt_disc)/(m^2 + n^2);             % cosθ2
end

% 关节1角度计算函数
% 原理：通过平面几何关系求解
function T = calculate_theta1(p, f, theta2, alpha)
    % 计算中间参数
    g(1) = cos(theta2)*f(1) - sin(theta2)*f(2);
    g(2) = -sin(alpha(2))*f(3);
    
    % 建立方程并求解
    denominator = g(1)^2 + g(2)^2;
    sin_theta1 = (g(1)*p(2) - g(2)*p(1)) / denominator;
    cos_theta1 = (g(1)*p(1) + g(2)*p(2)) / denominator;
    
    T = atan2(sin_theta1, cos_theta1);  % 四象限反正切
end

% 后三轴（关节4-6）角度计算函数
% 原理：通过旋转矩阵分解求解腕部关节角度
% 返回：每个前三个关节解对应的后三个关节解（可能1或2组）
function [T, num] = calculate_theta456(theta1, theta2, theta3, R06, alpha)
    % 计算前三关节的复合旋转矩阵
    R01 = DH_rotation(theta1, alpha(1));
    disp('R01:')
    disp(R01);
    R12 = DH_rotation(theta2, alpha(2));
    disp('R12:')
    disp(R12);
    R23 = DH_rotation(theta3, alpha(3));
    disp('R23:')
    disp(R23);
    
    % 计算腕部坐标系相对旋转
    R34 = [1 0 0; 0 cos(alpha(4)) -sin(alpha(4)); 0 sin(alpha(4)) cos(alpha(4))];
    disp('R34:')
    disp(R34);
    R46 = R34' * R23' * R12' * R01' * R06;
    disp('R46:')
    disp(R46);
    
    % 奇异位置判断
    if abs(R46(3,3) - 1) < 1e-8      % 腕部奇异位置1
        theta5 = 0;
        theta4 = 0;
        theta6 = atan2(-R46(1,2), R46(1,1));
        T = [theta4, theta5, theta6];
        num = 1;
    elseif abs(R46(3,3) + 1) < 1e-8  % 腕部奇异位置2
        theta5 = pi;
        theta4 = 0;
        theta6 = atan2(R46(1,2), -R46(1,1));
        T = [theta4, theta5, theta6];
        num = 1;
    else                             % 常规情况，两个解
        % 第一组解
        theta5 = atan2(sqrt(R46(3,1)^2 + R46(3,2)^2), R46(3,3));
        theta4 = atan2(R46(2,3)/sin(theta5), R46(1,3)/sin(theta5));
        theta6 = atan2(R46(3,2)/sin(theta5), -R46(3,1)/sin(theta5));
        
        % 第二组解（θ5取负）
        theta5_neg = -theta5;
        theta4_neg = atan2(R46(2,3)/sin(theta5_neg), R46(1,3)/sin(theta5_neg));
        theta6_neg = atan2(R46(3,2)/sin(theta5_neg), -R46(3,1)/sin(theta5_neg));
        
        T = [theta4, theta5, theta6; 
             theta4_neg, theta5_neg, theta6_neg];
        num = 2;
    end
end

% DH参数旋转矩阵生成函数（辅助函数）
function R = DH_rotation(theta, alpha)
    R = [cos(theta)           , -sin(theta)          , 0;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha)];
end