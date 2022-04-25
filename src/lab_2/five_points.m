function five_points()
    clear
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
    myHIDSimplePacketComs=HIDfactory.get();
    myHIDSimplePacketComs.setPid(pid);
    myHIDSimplePacketComs.setVid(vid);
    myHIDSimplePacketComs.connect();

    % Create a PacketProcessor object to send data to the nucleo firmware
    pp = Robot(myHIDSimplePacketComs);

    zPoint = [0,0,0];
    aPoint = [-56,24,-45];
    bPoint = [47,82,-23];
    cPoint = [0,85,32];
    dPoint = [87, -36, -38];

    points = [zPoint;aPoint;bPoint;cPoint;dPoint];


    try


    for i = 1:5
        pp.interpolate_jp(points(i,:),1.25);
        pause(10);
        values = pp.measured_js(true, false);
        plot_arm(values(1,:))
        pp.measured_cp()
    end
    
    
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end

    % Clear up memory upon termination
    pp.shutdown()
end

