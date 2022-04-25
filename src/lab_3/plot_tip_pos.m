function plotPos(data, axi, unit)
    time = data(:,1);
    theta1 = data(:,2);
    theta2 = data(:,3);
    theta3 = data(:,4);

    subplot(4,1,1)
    plot(time,theta1,'LineWidth',3)
    title('x')
    xlabel('Time (s)')
    ylabel(['Tip', ' ', axi, ' ', unit])

    subplot(4,1,2)
    plot(time,theta2,'LineWidth',3)
    title('y')
    xlabel('Time (s)')
    ylabel(['Tip', ' ', axi, ' ', unit])

    subplot(4,1,3)
    plot(time,theta3,'LineWidth',3)
    title('z')
    xlabel('Time (s)')
    ylabel(['Tip', ' ', axi, ' ', unit])

    subplot(4,1,4)
    plot(time,theta1,time,theta2,time,theta3,'LineWidth',3)
    title(['Triangle', ' ', axi, ' 1.5s Interpolation'])
    xlabel('Time (s)')
    ylabel(['Tip', ' ', axi, ' ', unit])
    legend({'x','y', 'z'}, 'Location', 'best')
end