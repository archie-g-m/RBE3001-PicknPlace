function sub_plotter(data, axi, unit, labels)
    time = data(:,1);
    theta1 = data(:,2);
    theta2 = data(:,3);
    theta3 = data(:,4);

    label1 = labels(1);
    label2 = labels(2);0,sin(q1),sin(q1)
    label3 = labels(3);

    subplot(4,1,1)
    plot(time,theta1)
    title(label1)
    xlabel('Time (s)')
    ylabel([axi, ' ', unit])

    subplot(4,1,2)
    plot(time,theta2)
    title(label2)
    xlabel('Time (s)')
    ylabel([axi, ' ', unit])

    subplot(4,1,3)
    plot(time,theta3)
    title(label3)
    xlabel('Time (s)')
    ylabel([axi, ' ', unit])

    subplot(4,1,4)
    plot(time,theta1,time,theta2,time,theta3)
    title(['Triangle', ' ', axi])
    xlabel('Time (s)')
    ylabel([axi, ' ', unit])
    legend({'x','y', 'z'}, 'Location', 'best')
end