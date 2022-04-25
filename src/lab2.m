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
myHIDSimplePacketComs = HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);

% syms q1 q2 q3
% l0 = 55
% l1 = 40
% l2 = 100
% l3 = 100
%
% DH_table = [l0,0,0,0;
%             l1,q1,0,deg2rad(90);
%             0, q2+deg2rad(90), l2, 0;
%             0, q3-deg2rad(90), l3, deg2rad(90)];
%
% TB_0 = pp.dh2mat(DH_table(1,:))
% T0_1 = pp.dh2mat(DH_table(2,:))
% T1_2 = pp.dh2mat(DH_table(3,:))
% T2_3 = pp.dh2mat(DH_table(4,:))
%
% TB_3 = TB_0 * T0_1 * T1_2 * T2_3
%
% TB_3_test = pp.dh2fk(DH_table)
%
% q1 = 0;
% q2 = 0;
% q3 = 0;

%TB_3_SUBS = subs(TB_3)

%pp.interpolate_jp([45,45,45],2);
%pause(2);
%output = pp.measured_cp()

pp.goal_cp()

pp.setpoint_cp()

try


    %   while true
    %     allValues = pp.measured_js(true, false);
    %     values = allValues(1,:);
    % %     disp(pp.fk3001(values))
    %     disp(pp.measured_cp())
    %     plot_arm(values)
    %   end

    while true
        % allValues = pp.measured_js(true, true);
        % velVals = allValues(2, :);
        % jointVals = allValues(1, :);
        %     disp(pp.fk3001(values))
        %     disp(pp.measured_cp())
        disp(pp.measured_tip())
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()
