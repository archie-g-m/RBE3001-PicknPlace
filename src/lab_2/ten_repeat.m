%% For figure 4

function ten_repeat(filename)
    data = csvread(filename);
    
    disp(length(data))
    tipsT = zeros(1,3);
    tipsB = zeros(1,3);

    % For each of 20 points (10 targets 10 home)
    for i = 1:20
       % Get the tip position

       disp(i*4 -3)
       disp(i*4 -2)
       disp(i*4 -1)
       disp(i*4)
       disp(data(i*4-3,4))
       disp(data(i*4-2,4))
       disp(data(i*4-1,4))
       
       %If case of target
       if mod(i,2)
        


       else

       end
        
        %If case of target
        if mod(i,2)
         tipsT3 = [tipsT3;someTip];
         % else case of base
        else
         tipsB3 = [tipsB3;someTip];
 
        end
    end

    tipsT1(1,:) = [];
    tipsB1(1,:) = [];
    tipsT2(1,:) = [];
    tipsB2(1,:) = [];
    tipsT3(1,:) = [];
    tipsB3(1,:) = [];

    tipsB1(:,4) = 1;
    tipsB2(:,4) = 2;
    tipsB3(:,4) = 3;

    tipsT1(:,4) = 1;
    tipsT2(:,4) = 2;
    tipsT3(:,4) = 3;
    
    colormap(lines)
    
    tipsBA = [tipsB1;tipsB2;tipsB3];
    cb = tipsBA(:,4);

    tipsTA = [tipsT1;tipsT2;tipsT3];
    ca = tipsTA(:,4);

    figure(1)
    subplot(3,1,1)
    scatter3(tipsTA(:,1),tipsTA(:,2),tipsTA(:,3),50,ca)
    xlabel("x");
    ylabel("y");
    zlabel("z");
    
    subplot(3,1,2)
    gscatter(tipsTA(:,1),tipsTA(:,2), ca)
    xlabel('x')
    ylabel('y')
    legend({'Yoni W.','Archie M.', 'Nathan S.'}, 'Location', 'best')

    subplot(3,1,3)
    gscatter(tipsTA(:,1),tipsTA(:,3), ca)
    xlabel('x')
    ylabel('z')
    legend({'Yoni W.','Archie M.', 'Nathan S.'}, 'Location', 'best')

    figure(2)
    subplot(3,1,1)
    scatter3(tipsBA(:,1),tipsBA(:,2),tipsBA(:,3),50,cb)


    xlabel("x");
    ylabel("y");
    zlabel("z");
    
    subplot(3,1,2)
    gscatter(tipsBA(:,1),tipsBA(:,2), cb)
    xlabel('x')
    ylabel('y')
    legend({'Yoni W.','Archie M.', 'Nathan S.'}, 'Location', 'best')

    subplot(3,1,3)
    gscatter(tipsBA(:,1),tipsBA(:,3), cb)
    xlabel('x')
    ylabel('z')
    legend({'Yoni W.','Archie M.', 'Nathan S.'}, 'Location', 'best')
end
