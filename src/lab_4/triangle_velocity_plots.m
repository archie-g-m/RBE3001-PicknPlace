function triangle_velocity_plots()

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

    point1 = [128 -100 77];
    point2 = [64 110 105];
    point3 = [137 -13 223];

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

    veld = [];

    for point = 1:3

        t = 0:resolution:interpolation_time;
        start_point = joints(point, :);
        goal_point = joints(point + 1, :);
        forward = robot.fk3001(goal_point);

        path_points = planner.cubic_traj(t, vel, vel, start_point, goal_point);
        t0 = clock;

        while etime(clock, t0) < interpolation_time
            curr_index = round(etime(clock, t0) / resolution) + 1;
            curr_pos_n_time = path_points(:, curr_index);
            curr_pos = curr_pos_n_time(2:4, :);
            robot.servojp(curr_pos);
            measured = robot.measured_js(false,true);
            q_vel = measured(2,:);
            curr_time = etime(clock, t00);
            fdk = robot.fdk3001(transpose(curr_pos), transpose(q_vel))
            veld = [veld; curr_time, transpose(fdk)];
        end

    end
    
    robot.shutdown()

    xyz_scalar_vel = [veld(:,1), sqrt(veld(:,2).^2+veld(:,3).^2+veld(:,4).^2)];
    disp(veld)
    disp(xyz_scalar_vel)

    lin_time = veld(:,1);
    xlin = veld(:,2);
    ylin = veld(:,3);
    zlin = veld(:,4);
    xq = veld(:,5);
    yq = veld(:,6);
    zq = veld(:,7);

    stime = xyz_scalar_vel(:,1);
    s = xyz_scalar_vel(:,2);

    subplot(3,1,1)
    plot(lin_time,xlin,lin_time,ylin,lin_time,zlin)
    xlabel('Time (s)')
    ylabel(['XYZ Linear Vel'])
    legend({'x','y', 'z'}, 'Location', 'best')

    subplot(3,1,2)
    plot(lin_time,xq,lin_time,yq,lin_time,zq)
    xlabel('Time (s)')
    ylabel(['XYZ Angular Vel'])
    legend({'x','y', 'z'}, 'Location', 'best')

    subplot(3,1,3)
    plot(stime,s)
    xlabel('Time (s)')
    ylabel(['Magnitude of Linear Vel'])
    legend({'x','y', 'z'}, 'Location', 'best')
    

    saveName = sprintf("p5_velocity_plots");
    saveLoc2 = sprintf('../output/%s.png', saveName);
    saveas(gcf, saveLoc2);

end
