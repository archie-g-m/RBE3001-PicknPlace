%% CSV Plotter  
% Made for RBE3001 Lab1

clear
clc
%plotHist('../output/position_output.csv')
%plotPos('../output/L1_part5_point3.csv')
plotTipPos('../output/lab2-triangle.csv')
%plotThreeBase('../output/position_output_1.csv','../output/position_output_2.csv','../output/position_output_3.csv')

function plotTipPos(filename)
    data = csvread(filename);
    dx = data(:,2);
    dy = data(:,3);
    dz = data(:,4);

    subplot(2,1,1)
    plot3(dx,dy,dz)
    title('Triangle')
    xlabel('x (mm)')
    ylabel('y (mm)')
    zlabel('z (mm)')

    subplot(2,1,2)
    plot(dx,dz)
    xlabel('x (mm)')
    ylabel('z (mm)')
end

function plotPos(filename)
    data = csvread(filename);
    time = data(:,1);
    theta1 = data(:,2);
    theta2 = data(:,3);
    theta3 = data(:,4);

    subplot(4,1,1)
    plot(time,theta1)
    title('Joint 1')
    xlabel('Time (ms)')
    ylabel('Motor position (degrees)')

    subplot(4,1,2)
    plot(time,theta2)
    title('Joint 2')
    xlabel('Time (ms)')
    ylabel('Motor position (degrees)')

    subplot(4,1,3)
    plot(time,theta3)
    title('Joint 3')
    xlabel('Time (ms)')
    ylabel('Motor position (degrees)')

    subplot(4,1,4)
    plot(time,theta1,time,theta2,time,theta3)
    title('Point3: 3s Interpolation')
    xlabel('Time (ms)')
    ylabel('Motor position (degrees)')
    legend({'Joint 1','Joint 2', 'Joint 3'}, 'Location', 'northeast')
end

function plotThreeBase(filename1, filename2, filename3)
    data1 = csvread(filename1);
    data2 = csvread(filename2);
    data3 = csvread(filename3);
    time1 = data1(:,1);
    time2 = data2(:,1);
    time3 = data3(:,1);
    theta1 = data1(:,2);
    theta2 = data2(:,2);
    theta3 = data3(:,2);

    plot(time1,theta1,time2,theta2,time3,theta3, 'LineWidth', 2)
    title('Joint 1: Three Trial Composite, With 3s Interpolation')
    xlabel('Time (ms)')
    ylabel('Motor position (degrees)')
    legend({'Trial 1','Trial 2', 'Trial 3'}, 'Location', 'southeast')
end

function plotHist(filename)
    data = csvread(filename);
    time = data(:,1);
    theta1 = data(:,2);
    theta2 = data(:,3);
    theta3 = data(:,4);

    deltaTimes = diff(time)

    mean_data = mean(deltaTimes);
    median_data = median(deltaTimes);
    max_data = max(deltaTimes);
    min_data = min(deltaTimes);

    statistics = [mean_data, median_data, max_data, min_data];

    writematrix(statistics, '../output/histogram_stats.csv')
    

    histogram(deltaTimes, [0 1 2 3 4 5 6 7 8 9 10])
    xlabel('Time delay (ms)')
    ylabel('Frequency')
    title('Time delay distribution')
    xlim([0 10])
    ylim([0 800])
end
