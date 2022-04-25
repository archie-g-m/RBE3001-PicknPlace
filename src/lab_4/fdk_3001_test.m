function fdk_3001_test()
    clear
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
    pp = Robot(myHIDSimplePacketComs);

    try
        q = [deg2rad(90), deg2rad(30), deg2rad(0)];
        q_dot = [0; -1; 0];
        pp.fdk3001(q, q_dot)

    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end

    % Clear up memory upon termination
    pp.shutdown()
end