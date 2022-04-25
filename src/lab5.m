%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
close all;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath '../lib/SimplePacketComsJavaFat-0.6.4.jar';
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs = HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);

%% Main Loop
try
    robot.interpolate_jp([0 0 0], 0.5)
    pause(0.5)

    robot.gripperOpen()
    pause(0.25)
    robot.gripperClose()
    pause(0.25)

    cam = Camera();
    cam.DEBUG = DEBUG_CAM;

    %% Place Poses per color
    purplePlace = [150, -50, 11];
    greenPlace = [150, 50, 11];
    pinkPlace = [75, -125, 11];
    yellowPlace = [75, 125, 11];

    colorPlaces = [purplePlace; greenPlace; pinkPlace; yellowPlace]
    
    place1 = [42 106 15];
    place2 = [82 106 15];
    place3 = [134 84 15];
    place4 = [11 80 15];

    %Z coord which will be above balls and other obstacles
    aboveZ = 50
    ballZ = 12.5
    ballX = 10

    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end

    cam.camPose = cam.getCameraPose();
    [cam.baseImg, refNewOrigin] = cam.getImage();
    cam.transform = cam.camTransform();

    % screenPoint = [900 493];

    robot.gripperOpen()
    pause(0.25)
    robot.gripperClose()
    pause(0.25)

    disp("PUT A BALL, then press any key to continue");
    pause;

    % while 1
    balls = cam.detectBalls();

    ballScreenCoords = balls(:, 1:2);
    ballColors = balls(:, 3)
    ballWorldCoords = zeros(size(ballScreenCoords, 1), 3)

    for i = 1:size(ballScreenCoords, 1)
        ballWorldCoords(i, :) = cam.screen2world(ballScreenCoords(i, :));
    end

    disp('Coordinates of balls in the world: ')
    disp(ballWorldCoords)

    intTime1 = 3%Time to go from ball loc to dropoff
    intTime2 = 1%Time to go from aboveZ to ballZ
    perpendicularOffset = 30;


    % end
    for index = 1:size(ballWorldCoords, 1)
        % for each ball in the list

        
        ballPos = ballWorldCoords(index, :);
        % theta = atan2(ballPos(2), ballPos(1));
        % dy = perpendicularOffset *sin(theta);
        % dx = perpendicularOffset * cos(theta);
        % xp = ballPos(1) + dx;
        % yp = ballPos(2) + dy;
        % ballPos(1)=xp;
        % ballPos(2)=yp;

        ballColor = ballColors(index, :)
        colorPlace = colorPlaces(uint32(ballColors(index)), :)

        poseOverBall = [ballPos(1) + ballX, ballPos(2), aboveZ]
        poseAtBall = [ballPos(1) + ballX, ballPos(2), ballZ]
        poseOverDropOff = [colorPlace(1), colorPlace(2), aboveZ]
        poseAtDropOff = [colorPlace(1), colorPlace(2), ballZ]

        %go to point over ball
        disp("goto point over ball")
        robot.cubic_cp(poseOverBall, intTime1)
        % robot.interpolate_jp(ik(poseOverBall), 3000)
        % pause(3.5);
        robot.gripperOpen();
        pause(0.125);

        %go to z of ball
        disp("goto z of ball")
        disp ("Attempting to move to: ")
        disp(poseAtBall);
        robot.cubic_cp(poseAtBall, intTime2)
        % robot.interpolate_jp(ik(poseAtBall), 1000)
        % pause(1.5);
        disp("Actually arrived at: ")
        disp(robot.measured_tip())

        %close gripper
        robot.gripperClose();

        %go back to point above ball
        disp("go back above ball")
        robot.cubic_cp(poseOverBall, intTime2)
        % robot.interpolate_jp(ik(poseOverBall), 1000)
        % pause(1.5);

        %go to point above drop off
        disp("goto above dropoff")
        robot.cubic_cp(poseOverDropOff, intTime1)
        % robot.interpolate_jp(ik(poseOverDropOff), 3000)
        % pause(3.5);
        

        %go ground z
        disp("ground z")
        robot.cubic_cp(poseAtDropOff, intTime2)
        % robot.interpolate_jp(ik(poseAtDropOff), 1000)
        % pause(1.5);
        
        %open gripper
        robot.gripperOpen()
        
        %go to point above dropoff
        disp("above dropoff")
        robot.cubic_cp(poseOverDropOff, intTime1)
        % robot.interpolate_jp(ik(poseOverDropOff), 3000)
        % pause(3.5);

    end
    
    robot.gripperClose()
    robot.interpolate_jp([0,0,0],2)
    pause(2.5)

catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
