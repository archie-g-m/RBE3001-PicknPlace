function iterative_velocity_motion(start_pos, goal_pos)

    % clear
    % clear java
    % clear classes;

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

    %constants
    threshold = 10;
    speed = 5; % mm/s
    frequency = 100;
    delay = 1 / frequency;

    %initialize dist
    distance_ = Inf;

    t00 = clock;
    t0 = clock;
    pass = 0;

    curr_q = start_pos;

    while distance_ >= threshold

        if (etime(clock, t0) > delay)
            curr_q = robot.ik_3001_numerical(curr_q, goal_pos)

            plot_arm_2d(curr_q, [0,0,0]);

            fk = robot.fk3001(deg2rad(curr_q)) * [0;0;0;1]
            tip = transpose(fk(1:3,:));
            distance_ = norm(goal_pos - tip)
        end

    end

    % Clear up memory upon termination
    robot.shutdown()
end
