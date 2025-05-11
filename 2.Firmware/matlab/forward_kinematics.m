function T = forward_kinematics(i, joint_rads, d, a, alpha)
% i 是关节, 如果要末端关节就是6
    T = fk_single_transform(1,joint_rads, d, a, alpha);

    if i >1
        for j=2:1:i
            T=T*fk_single_transform(j,joint_rads, d, a, alpha);
        end
    end
    %T = util_close_zero_to_zero_T(T,4,4,5);
end

function T = fk_single_transform(i, joint_rads, d, a, alpha)

    th(1) =joint_rads(1);
    th(2) =joint_rads(2);
    th(3) =joint_rads(3);
    th(4) =joint_rads(4);
    th(5) =joint_rads(5);
    th(6) =joint_rads(6);

    T =[cos(th(i))              -sin(th(i))                 0               a(i); 
    cos(alpha(i))*sin(th(i))     cos(alpha(i))*cos(th(i))     -sin(alpha(i))   -sin(alpha(i))*d(i);
    sin(alpha(i))*sin(th(i))     sin(alpha(i))*cos(th(i))     cos(alpha(i))    cos(alpha(i))*d(i);
    0                           0                           0               1];
      
end