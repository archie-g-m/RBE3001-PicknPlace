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
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);

int_time = 2.5

point1 = ik([128,-100,77]);
point2 = ik([64,110,105]);
point3 = ik([137,-13,223]);

points  = [point1;point2;point3];

values = [];
jpvalues = [];
tsvalues = [];
try

%   while true
%     allValues = pp.measured_js(true, false);
%     values = allValues(1,:);
% %     disp(pp.fk3001(values))
%     disp(pp.measured_cp())
%     plot_arm(values)
%   end
t00 = clock
for i = 2
    tic
    pp.interpolate_jp(points(i,:),int_time*1000);
    t0 = clock
    while etime(clock,t0)<int_time
        current_js = pp.measured_js(true, false);
        current_jp = current_js(1,:)
        current_ts = pp.measured_tip()
        time = etime(clock,t00);
        jpvalues = [jpvalues;[time, current_jp]];
        tsvalues = [tsvalues;[time, current_ts]];
        plot_arm(current_jp)
        pause(0.01)
    end
end  

disp(jpvalues)
disp(tsvalues)

% figure(1)
% sub_plotter(jpvalues, 'Joint Position', ' (Degrees)', {'q1', 'q2', 'q3'})

% figure(2)
% sub_plotter(tsvalues, 'Tip Position', ' (mm)', {'x', 'y', 'z'})
  
% figure(3)
% plot3(tsvalues(:,2), tsvalues(:,3), tsvalues(:,4))

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

