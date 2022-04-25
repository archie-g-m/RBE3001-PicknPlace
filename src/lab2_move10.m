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
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 

  origin = [0,0,0];
  target = [87, -36, -38];
  interpolateTime = 1000;

  pp.interpolate_jp(origin,0);
  pause(1);

  output = zeros(1,4);
  output2 = zeros(1,4);
  values = zeros(1,3);
  
  t0 = clock;
  pp.interpolate_jp(origin,0);
  for i = 1:10
    % Goto target
    pp.interpolate_jp(target,interpolateTime);
    pause(1.3);

    %Record values
    tip = pp.measured_cp()
%     output = [output;[i,values(1,:)]];
    output2 = [output2;tip]
    
    % Go back
    pp.interpolate_jp(origin,interpolateTime);
    pause(1.3);

    % Record values
    tip = pp.measured_cp()
%     output = [output;[i,values(1,:)]];
    output2 = [output2;tip;]

  end

%   output(1,:) = [];
%   writematrix(output, '../output/lab2-trials-p4-joint.csv');
  
  output2(1,:) = [];
  writematrix(output2, '../output/lab2-trials-p4-tip.csv');
  
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()
