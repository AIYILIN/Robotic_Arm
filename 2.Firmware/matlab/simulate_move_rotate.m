initial_theta = [0 pi/2 0 0 0 0];
d=[0 0 0 139 0 0];
alpha=[0 pi/2 0 pi/2 -pi/2 pi/2];
%alpha=[0 pi/2 0 pi/2 -pi/2 pi/2];
a = [0 0 300 0 0 0];

L(1) = Link([0	0		0	0],    'modified');
L(2) = Link([pi/2	0		0	pi/2], 'modified');
L(3) = Link([0	0		300	0],    'modified');
L(4) = Link([0	139		0	pi/2],    'modified');
L(5) = Link([0	0		0	-pi/2], 'modified');
L(6) = Link([0	0		0	pi/2], 'modified');
Six_Link = SerialLink(L,'name','Six_Link');
%Six_Link.plot(initial_theta);
%Six_Link.teach
Six_Link.fkine([0 pi/2 0 0 0 0])

T_first = forward_kinematics(6, initial_theta, d, a, alpha);

%x方向移动
%T_last = [T_first(:,1) T_first(:,2) T_first(:,3) [T_first(1,4)+20 T_first(2:4,4)']'];

%y方向移动
T_last = [T_first(:,1) T_first(:,2) T_first(:,3) [T_first(1,4) T_first(2,4)+10 T_first(3:4,4)']'];

%z方向移动
%T_last = [T_first(:,1) T_first(:,2) T_first(:,3) [T_first(1,4) T_first(2,4) T_first(3,4)+10 T_first(4,4)]'];

%绕x旋转pi/2
%out_pose = [1 0 0;0 cos(pi/2) -sin(pi/2);0 sin(pi/2) cos(pi/2)] * T_first(1:3,1:3);
%T_last = [out_pose(1,:) T_first(1,4);out_pose(2,:) T_first(2,4);out_pose(3,:) T_first(3,4);T_first(4,:)];

%绕y旋转pi/2
%out_pose = [cos(pi/2)  0 sin(pi/2);0 1 0;-sin(pi/2) 0 cos(pi/2)] * T_first(1:3,1:3);
%T_last = [out_pose(1,:) T_first(1,4);out_pose(2,:) T_first(2,4);out_pose(3,:) T_first(3,4);T_first(4,:)];

%绕z旋转pi/2
%out_pose = [cos(pi/2) -sin(pi/2) 0;sin(pi/2) cos(pi/2) 0;0 0 1] * T_first(1:3,1:3);
%T_last = [out_pose(1,:) T_first(1,4);out_pose(2,:) T_first(2,4);out_pose(3,:) T_first(3,4);T_first(4,:)];

theta = calculate_theta(T_last, d, a, alpha);
theta_jiaodu = theta * 180 / pi; 

%Six_Link.ikine(T_last) 