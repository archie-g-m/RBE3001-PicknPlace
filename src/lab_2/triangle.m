function triangle()
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

    zPoint = [0,0,-20];
    aPoint = [0,60,-45];
    bPoint = [0,60,35];
    cPoint = [0,0,-20];

    points = [zPoint;aPoint;bPoint;cPoint];


    try

    output = zeros(1,4);
    tip = zeros(1,3);
    
    for i = 1:4
        pp.interpolate_jp(points(i,:),1000);
%         pause(2);
        % values = pp.measured_js(true, false);
        % plot_arm(values(1,:))
        % pp.measured_cp()


       t0 = clock;
       while etime(clock, t0) < 2
        allValues = pp.measured_cp()
        tip(1) = allValues(1,4);
        tip(2) = allValues(2,4);
        tip(3) = allValues(3,4);
        output = [output;[i,tip(1,:)]];
        disp(clock);
      end
      
    end
    output(1,:) = [];
    writematrix(output, '../output/lab2-triangle.csv');
    
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end

    % Clear up memory upon termination
    pp.shutdown()
end

