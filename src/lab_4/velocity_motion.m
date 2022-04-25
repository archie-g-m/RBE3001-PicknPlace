function velocity_motion(goal_pos)

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
    speed = 30; % mm/s
    frequency = 50;
    delay = 1 / frequency;

    %initialize dist
    distance_ = Inf;

    t00 = clock;
    t0 = clock;
    pass = 0;

    while distance_ > threshold

        if (etime(clock, t0) > delay)
            t0 = clock;
            % Measure robot's current joint positions and velocities
            curr_values = robot.measured_js(true, true);
            curr_q = curr_values(1, :);
            curr_q_dot = curr_values(2, :);

            pass = pass + 1;

            % Calculate target direction
            curr_pos = robot.measured_tip();
            direction = goal_pos - curr_pos;
            distance_ = norm(direction); % calculate magnitude of error (to stop while-loop)

            target_vel = speed * (direction / distance_);

            %% Get Joint velocities (inverse differential kinematics)
            % target_joint_vel =

            q_dot = robot.idk3001(target_vel, curr_q);

            %% Set target point
            robot.servo_vel(curr_q, q_dot, delay);

            if robot.check_safety(curr_q)
                disp("I AM HERE IN ERROR")
                error("AN ERROR: APPROACHING SINGULARITY")
            end

            % if (mod(pass, 5))
            %     plot_arm(curr_q, target_vel);
            % end

        end

    end

    % Clear up memory upon termination
    robot.shutdown()
end
