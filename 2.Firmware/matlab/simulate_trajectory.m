initial_theta = [0 pi/2 0 0 0 0];
d=[0 0 0 139 0 0];
alpha=[0 pi/2 0 pi/2 -pi/2 pi/2];
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
%Six_Link.fkine([0 pi/2 0 0 0 0])

T_first = forward_kinematics(6, initial_theta, d, a, alpha);
T_last = [T_first(:,1) T_first(:,2) T_first(:,3) [159 0 300 1]'];%-300 0 139 1

via_point = calculate_linear_trajectory(T_first(1:3, 4), T_last(1:3, 4));
[row, col] = size(via_point);

now_theta = initial_theta;
for i=1:row
    coord = [T_first(:,1) T_first(:,2) T_first(:,3) [via_point(i, :) 1]'];
    theta = calculate_theta(coord, d, a, alpha);
    next_theta = select_theta(now_theta, theta);
    Six_Link.plot(next_theta);
    hold on
    grid on

    T_via = forward_kinematics(6, next_theta, d, a, alpha);
    bx = T_via(1,4);
    by = T_via(2,4);
    bz = T_via(3,4);
    plot3(bx,by,bz,'*','LineWidth',1);
    hold on

    now_theta = next_theta;
end

function T = calculate_linear_trajectory(p0, p1)
    vector = p1 - p0;
    vector_length = sqrt(vector(1)^2 + vector(2)^2 + vector(3)^2);

    i = 0;
    j = 1;
    while i <= vector_length
        T(j,:) = p0 + i * (vector / vector_length);
        i = i + 1;
        j = j + 1;
    end

    if mod(vector_length, 5) ~= 0
        T(j,:) = p1;
    end
    %plot3(T(:,1)',T(:,2)',T(:,3)','r')
end

function T = select_theta(now, theta)
    t0 = theta - now;
    t1 = t0(:,1).* t0(:,1) + t0(:,2).* t0(:,2)  + t0(:,3).* t0(:,3);
    [m,i] = min(t1);
    T = theta(i,:);
end