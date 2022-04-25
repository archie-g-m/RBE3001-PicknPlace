function fun_traj
    clear
    clc
    clear java
    clear classes;

    vid = hex2dec('16c0');
    pid = hex2dec('0486');

    disp (vid);
    disp (pid);

    javaaddpath '../lib/SimplePacketComsJavaFat-0.6.4.jar';
    import edu.wpi.SimplePacketComs.*;
    import edu.wpi.SimplePacketComs.device.*;
    import edu.wpi.SimplePacketComs.phy.*;
    import java.util.*;
    import org.hid4java.*;
    version -java
    myHIDSimplePacketComs = HIDfactory.get();
    myHIDSimplePacketComs.setPid(pid);
    myHIDSimplePacketComs.setVid(vid);
    myHIDSimplePacketComs.connect();

    % Create a PacketProcessor object to send data to the nucleo firmware
    robot = Robot(myHIDSimplePacketComs);
    planner = Traj_Planner();

    point1 = [100 -100 12.5];
    point2 = [150 0 12.5];
    point3 = [100 100 12.5];
    
    %point1 = [128 -70 100];
    %point2 = [128 -100 130];
    %point3 = [128 -130 100];
    point4 = [128 -70 70];
    point5 = [128 -100 50];
    point6 = [128 -130 70];
    point7 = [128 -70 100];
    
    point8 = [64 110 125];
    point9 = [64 130 105];   
    point10 =[64 110 85];   
        
    points_list = [
            point1;
            point2;
            point3;
            % point4;
            % point5;
            % point6;
            % point7;
            % point8;
            % point9;
            % point10;
            % point4;
            point1];

    joint1 = ik(point1);

    vel = [0 0 0];

    interpolation_time = 3;

    resolution = 0.01;

    robot.servojp(joint1);
    pause(0.4);

    t00 = clock;
    data = []

    for point = 1:4
        disp(point)
        t = 0:resolution:interpolation_time;
        start_point = points_list(point, :);
        goal_point = points_list(point + 1, :);
        forward = robot.fk3001(goal_point);
        path_points = planner.cubic_traj(t, vel, vel, start_point, goal_point);

        % figure(point)
        % plot(t,path_points,'LineWidth',6)

        t0 = clock;

        print_val = 0;

        while etime(clock, t0) < interpolation_time
            curr_index = round(etime(clock, t0) / resolution) + 1;
            curr_pos_n_time = path_points(:, curr_index);
            curr_pos = curr_pos_n_time(2:4, :);

            ik_curpos = ik(curr_pos);
            robot.servojp(ik_curpos);

            print_val = print_val + 1;

            if (mod(print_val, 40) == 0)
                time = etime(clock, t00);
                tip_pos = robot.measured_tip();
                data_point = [time, tip_pos];
                data = [data; data_point];
            end

        end

        print_val = 0;

    end

    vel_data = diff(data);

    for i = 2:4
        vel_data(:, i) = vel_data(:, i) ./ vel_data(:, 1);
    end

    acc_data = diff(vel_data);

    for i = 2:4
        acc_data(:, i) = acc_data(:, i) ./ acc_data(:, 1);
    end

    %% 3D
    figure(1)
    set(gcf, 'Position', [100, 100, 400, 400])
    plot3(data(:, 2), data(:, 3), data(:, 4), 'LineWidth', 2)
    xlabel("x");
    ylabel("y");
    zlabel("z");
    grid on
    save_name = sprintf("ec1_traj");
    saveLoc2 = sprintf('../output/%s.png', save_name);
    saveas(gcf, saveLoc2);
    pause(0.3)

    robot.shutdown();
end
