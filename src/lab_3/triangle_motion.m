function triangle_motion
    clear
    clc
    clear java
    clear classes;

    vid = hex2dec('16c0');
    pid = hex2dec('0486');

    disp (vid);
    disp (pid);

    javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
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

    %point1 = [128 -100 77];
    %point2 = [64 110 105];
    %point3 = [137 -13 223];
    
    point1 = [100 -100 12.5];
    point2 = [150 0 12.5];
    point3 = [100 100 12.5];
    
    
    joint1 = ik(point1);
    joint2 = ik(point2);
    joint3 = ik(point3);

    vel = [0 0 0];

    joints = [joint1; joint2; joint3; joint1]
    
    interpolation_time = 1.5;

    resolution = 0.01;

    robot.servojp(joint1);
    pause(0.4);
    
    t00 = clock;
    data = []

    for point = 1:3

        t = 0:resolution:interpolation_time;
        start_point = joints(point, :);
        goal_point = joints(point + 1, :);
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
            robot.servojp(curr_pos);

            print_val = print_val + 1;
            if(mod(print_val, 40) == 0)
                time = etime(clock, t00);
                tip_pos = robot.measured_tip();
                data_point = [time, tip_pos];
                data = [data ; data_point];
            end
        end

        print_val = 0;




    end
    
    robot.shutdown()

    vel_data = diff(data);

    for i = 2:4
        vel_data(:,i) = vel_data(:,i)./vel_data(:,1);
    end

    
    acc_data = diff(vel_data);
    for i = 2:4
        acc_data(:,i) = acc_data(:,i)./acc_data(:,1);
    end

    figure(1)
    plot_tip_pos(data, 'Position', '(mm)');

    figure(2)
    vel_data(:,1) = data(1:end-1,1);
    plot_tip_pos(vel_data, 'Velocity', '(mm/s)')
    
    figure(3)
    acc_data(:,1) = vel_data(1:end-1,1);
    plot_tip_pos(acc_data, 'Acceleration', '(mm/s^2)')
    

end