

% 创建Link对象（modified DH）
L1 = Link('offset',0,     'd', 0,          'a', 0,      'alpha', 0     ,'modified');   
L2 = Link('offset',pi/2,  'd', 0,          'a', 0,      'alpha', pi/2  ,'modified');
L3 = Link('offset',0,     'd', 0,          'a', 199.54, 'alpha', 0     ,'modified');
L4 = Link('offset',0,     'd', 171.53,     'a', 95.91,  'alpha', pi/2  ,'modified');
L5 = Link('offset',0,     'd', 0,          'a', 0,      'alpha', -pi/2 ,'modified');
L6 = Link('offset',0,     'd', 168.51,     'a', 0,      'alpha', pi/2  ,'modified');

% 构建机械臂
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6-DOF Robot');

% 生成蒙特卡洛点云数据
joint_limits = [
    -pi        , pi;       % Joint 1
    -pi/2      , pi/2;     % Joint 2
    -pi/5 * 4  , pi/5 * 4; % Joint 3
    -pi        , pi;       % Joint 4
    -pi/5 * 4  , pi/5 * 4; % Joint 5
    -pi        , pi;       % Joint 6
];

N = 20000;
q = zeros(N, 6);
for i = 1:6
    q(:,i) = (joint_limits(i,2) - joint_limits(i,1)) * rand(N,1) + joint_limits(i,1);
end

% 计算末端坐标
points = zeros(N,3);
for i = 1:N
    T = robot.fkine(q(i,:)); 
    points(i,:) = T.t(1:3)'; 
end

figure;
% 绘制点云
scatter3(points(:,1), points(:,2), points(:,3), 1, 'b.', 'MarkerFaceAlpha', 0.1);
hold on; 

% --- 关键修复：获取三维坐标轴范围 ---
xlim = get(gca, 'XLim');
ylim = get(gca, 'YLim');
zlim = get(gca, 'ZLim');
workspace_limits = [xlim, ylim, zlim]; % 组合成 Xmin Xmax Ymin Ymax Zmin Zmax

% 绘制机械臂模型（设置完整的工作空间范围）
q_plot = [0, 0, 0, 0, 0, 0];
robot.plot(q_plot, ...
    'workspace', workspace_limits, ...  % 指定全部三维范围
    'noarrow', 'noname', ...
    'linkcolor', 'r', 'jointcolor', 'k', 'basecolor', 'g');

hold off;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('机械臂工作空间点云');
axis equal;
grid on;
rotate3d on;