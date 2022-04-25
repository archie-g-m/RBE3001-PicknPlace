function move_arm(joint_space, q)

    %%
    % RBE3001 - Laboratory 1
    %
    % Instructions
    % ------------
    % Welcome again! This MATLAB script is your starting point for Lab
    % 1 of RBE3001. The sample code below demonstrates how to establish
    % communication between this script and the Nucleo firmware, send
    % setpoint commands and receive sensor data.
    %
    % IMPORTANT - understanding the code below requires being familiar
    % with the Nucleo firmware. Read that code first.

    % Lines 15-37 perform necessary library initializations. You can skip reading
    % to line 38.

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
    pp = Robot(myHIDSimplePacketComs);

    if (joint_space == false)% If in task space
        goal = ik(q); % IK to get to joint space
    else % If in joint space
        goal = q; % Continue onward
    end

    try

        pp.interpolate_jp(goal, 500);

    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end

    pause(0.5) % Wait for interpolation
    % Clear up memory upon termination
    pp.shutdown()

end
