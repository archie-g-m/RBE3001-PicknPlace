function plot_arm_2d(q, v)

    syms q1 q2 q3 l0 l1 l2 l3;
    q1 = deg2rad(q(1));
    q2 = deg2rad(-q(2));
    q3 = deg2rad(-q(3));
    l0 = 55;
    l1 = 40;
    l2 = 100;
    l3 = 100;

    TB_0 = dht(l0,0,0,0);
    T0_1 = dht(l1, q1,0,deg2rad(90));
    T1_2 = dht(0, q2+deg2rad(90), l2, 0);
    T2_3 = dht(0, q3-deg2rad(90), l3, deg2rad(90));

    origin = [0;0;0;0];

    % f0 = origin * double(subs(TB_0));
    f0 = zeros(4,4);
    f1 = double(subs(TB_0 * T0_1));
    f2 = f1 * double(subs(T1_2));
    f3 = f2 * double(subs(T2_3));

    joint_points = zeros(4,3);
    joint_points(1,:) = f0(1:3,4);
    joint_points(2,:) = f1(1:3,4);
    joint_points(3,:) = f2(1:3,4);
    joint_points(4,:) = f3(1:3,4);
    
    %figure(1)
    plot(joint_points(:,1),joint_points(:,3),'-o','MarkerSize',10, 'LineWidth',2);
    hold on
    quiv = quiver(joint_points(4,1),joint_points(4,3), v(1), v(2), v(3));
    set(quiv,'AutoScale','on', 'AutoScaleFactor', 2);
    hold off
    xlim([-100 300]);
    ylim([-100 300]);
    grid on
    xlabel("x");
    ylabel("z");
    %hold on
    %hold off

    drawnow
    

end