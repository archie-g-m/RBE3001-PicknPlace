function jacob_verify()
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

    syms q1 q2 q3
    l0 = 55;
    l1 = 40;
    l2 = 100;
    l3 = 100;


    DH_table = [l0,0,0,0;
                l1,q1,0,deg2rad(90);
                0, -q2+deg2rad(90), l2, 0;
                0, -q3-deg2rad(90), l3, deg2rad(90)];

    fk = pp.fk3001([q1,q2,q3])
    xq = fk(1:3,4)

    TB_0 = pp.dh2mat(DH_table(1,:));
    T0_1 = pp.dh2mat(DH_table(2,:));
    T1_2 = pp.dh2mat(DH_table(3,:));

    TB_1 = TB_0 * T0_1;
    TB_2 = TB_0 * T0_1 * T1_2;

    % jacob = jacobian(xq,[q1,q2,q3])

    dxdq1 = diff(xq, q1)
    dxdq2 = diff(xq, q2)
    dxdq3 = diff(xq, q3)

    Jt = [dxdq1 dxdq2 dxdq3];
    
    Jb_1 = [0;0;1];
    Jb_2 = TB_1(1:3,3);
    Jb_3 = TB_2(1:3,3);

    JB = [Jb_1 Jb_2 Jb_3]

    J = [Jt; JB]

    pp.shutdown();



end
