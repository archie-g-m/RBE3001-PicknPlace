function lab3_signoff
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

    point1 = [128 -100 77];
    point2 = [64 110 105];
    point3 = [137 -13 223];

    points_li = [point1; point2; point3; point1]

    joint1 = ik(point1);
    joint2 = ik(point2);
    joint3 = ik(point3);

    vel = [0 0 0];

    joints = [joint1; joint2; joint3; joint1]

    interpolation_time = 1.5;

    resolution = 0.01;

    for traj_type = 1:3

        robot.servojp(joint1);
        pause(0.4);

        t00 = clock;
        data = []

        part_num = 4+traj_type;
        %names = {'Cubic - Joint Space', 'Cubic - Task Space', 'Quintic - Joint Space'};
        names = {'cubic_joint', 'cubic_task', 'quintic_joint'};

        part_name = cell2mat(names(traj_type));

        for point = 1:3

            if traj_type == 1% CUBIC
                t = 0:resolution:interpolation_time;
                start_point = joints(point, :);
                goal_point = joints(point + 1, :);
                forward = robot.fk3001(goal_point);
                path_points = planner.cubic_traj(t, vel, vel, start_point, goal_point);

            elseif traj_type == 2% CUBIC TASKSPACE
                t = 0:resolution:interpolation_time;
                start_point = points_li(point, :);
                goal_point = points_li(point + 1, :);
                forward = robot.fk3001(goal_point);
                path_points = planner.cubic_traj(t, vel, vel, start_point, goal_point);

            elseif traj_type == 3% QUINTIC JOINT SPACE
                t = 0:resolution:interpolation_time;
                start_point = joints(point, :);
                goal_point = joints(point + 1, :);
                forward = robot.fk3001(goal_point);
                path_points = planner.quintic_traj(t, vel, vel, start_point, goal_point);

            else % DEFAULT: CUBIC
                t = 0:resolution:interpolation_time;
                start_point = joints(point, :);
                goal_point = joints(point + 1, :);
                forward = robot.fk3001(goal_point);
                path_points = planner.cubic_traj(t, vel, vel, start_point, goal_point);
            end

            % figure(point)
            % plot(t,path_points,'LineWidth',6)

            t0 = clock;

            print_val = 0;

            while etime(clock, t0) < interpolation_time
                curr_index = round(etime(clock, t0) / resolution) + 1;
                curr_pos_n_time = path_points(:, curr_index);
                curr_pos = curr_pos_n_time(2:4, :);

                if traj_type == 2% Task space do IK _onthefly_
                    ik_curpos = ik(curr_pos);
                    robot.servojp(ik_curpos);
                else % DO NORMAL (JOINT SPACE)
                    robot.servojp(curr_pos);
                end

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
        
        %% POSITION
        figure(traj_type * 3 - 2)
        set(gcf, 'Position',  [100, 100, 400, 800])
        plot_tip_pos(data, 'Position', '(mm)');

        saveName = sprintf("p%d_%s_position", part_num, part_name);
        saveLoc2 = sprintf('../output/%s.png', saveName);
        saveas(gcf, saveLoc2);
        pause(0.3)

        %% VELOCITY
        figure(traj_type * 3 - 1)
        set(gcf, 'Position',  [100, 100, 400, 800])
        vel_data(:, 1) = data(1:end - 1, 1);
        plot_tip_pos(vel_data, 'Velocity', '(mm/s)')

        saveName = sprintf("p%d_%s_velocity", part_num, part_name);
        saveLoc2 = sprintf('../output/%s.png', saveName);
        saveas(gcf, saveLoc2);
        pause(0.3)

        %% ACCEL
        figure(traj_type * 3)
        set(gcf, 'Position',  [100, 100, 400, 800])
        acc_data(:, 1) = vel_data(1:end - 1, 1);
        plot_tip_pos(acc_data, 'Accel', '(mm/s^2)')
        
        saveName = sprintf("p%d_%s_accel", part_num, part_name);
        saveLoc2 = sprintf('../output/%s.png', saveName);
        saveas(gcf, saveLoc2);
        pause(0.3)

        %% 3D
        if (traj_type == 2) % CUBIC TASKSPACE
            figure(10)
            set(gcf, 'Position', [100, 100, 400, 400])
            plot3(data(:,2), data(:,3), data(:,4),'LineWidth',2)
            xlabel("x");
            ylabel("y");
            zlabel("z");
            grid on
            save_name = sprintf("p6_3D");
            saveLoc2 = sprintf('../output/%s.png', save_name);
            saveas(gcf, saveLoc2);
            pause(0.3)
        end

    end

    robot.shutdown();
end
