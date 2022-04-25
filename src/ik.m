function q = ik(cp)
    % Takes in cp, list of cartesian points in (x,y,z)

    % Link lengths
    l0 = 55;
    l1 = 40;
    l2 = 100;
    l3 = 100;

    %% Calculate theta1
    th1 = atan2(cp(2),cp(1));

    %% Caluclate theta2
    % Find hypot
    h = cp(1)^2 + cp(2)^2 + (cp(3)-l0-l1)^2;

    th2_a = atan2((cp(3)-l0-l1), sqrt(cp(1)^2 + cp(2)^2));
    
    th2_top = l2^2 + h - l3^2;
    th2_bot = 2 * sqrt(h) * l2;
    th2_b = acos(th2_top/th2_bot);

    th2 = pi/2 -(th2_a + th2_b); % invert

    %% Calculate theta3
    th3_top = l2^2 + l3^2 - h;
    th3_bot = 2 * l2 * l3;

    th3 = -(acos(th3_top/th3_bot) - pi/2); % invert

    q = rad2deg([th1, th2, th3]);
end