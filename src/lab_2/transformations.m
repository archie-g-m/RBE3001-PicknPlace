function transformations

    clear
    clc

    vid = hex2dec('16c0');
    pid = hex2dec('0486');

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
    robot = Robot(myHIDSimplePacketComs);

    %% Symbolic Variables
    syms q1 q2 q3 l0 l1 l2 l3

    DH_table = [l0,0,0,0;
                l1,q1,0,deg2rad(90);
                0, q2+deg2rad(90), l2, 0;
                0, q3-deg2rad(90), l3, deg2rad(90)];

    TB_0 = robot.dh2mat(DH_table(1,:))
    T0_1 = robot.dh2mat(DH_table(2,:))
    T1_2 = robot.dh2mat(DH_table(3,:))
    T2_3 = robot.dh2mat(DH_table(4,:))

    TB_3 = robot.dh2fk(DH_table)

    %% Parameters


    l0 = 55;
    l1 = 40;
    l2 = 100;
    l3 = 100;

    %% Zero Position
    q1 = 0;
    q2 = 0;
    q3 = 0;    

    zero_position = double(subs(TB_3))

    %% Cradle Position
    q1 = deg2rad(-90);
    q2 = deg2rad(-85);
    q3 = deg2rad(-36);

    cradle_position = double(subs(TB_3))
    
    %% Pos 1
    q1 = deg2rad(45);
    q2 = deg2rad(-45);
    q3 = deg2rad(-45);

    pos_1 = double(subs(TB_3))
    
    %% Pos 2
    q1 = deg2rad(-30);
    q2 = deg2rad(-30);
    q3 = deg2rad(-60);

    pos_2 = double(subs(TB_3))


    % Clear up memory upon termination
    robot.shutdown()

end
