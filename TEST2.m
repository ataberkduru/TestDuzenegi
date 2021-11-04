clear all;
clc;
close all;
delete(instrfindall);
comport=serial('COM8','BaudRate',9600,'DataBits',8);
set(comport,'Parity','none');

fopen(comport);
x=1;

while(1)
    
    axis([0 100 -200 200]);
    
    x=x+1;
    if(x>100)
       f=100;
    end 
    if(x<101)
       f=x;
    end 
    y1(f)=fscanf(comport,'%d');%akým 1
    y2(f)=fscanf(comport,'%d');%akim 2
    y3(f)=fscanf(comport,'%d');%agirlik
    y4(f)=fscanf(comport,'%d');%pwm
    subplot(4,1,1);
    plot(y1,'b','linewidth',1);
    subplot(4,1,2);
    plot(y2,'b','linewidth',1);
    subplot(4,1,3);
    plot(y3,'b','linewidth',1);
    subplot(4,1,4);
    plot(y4,'b','linewidth',1);
    grid on;    
    hold on;
   
    
    drawnow;
    
    hold off;
    if(x>=100)
       loop=0;
       while(loop<99)
           loop=loop+1;
          y1(loop)=y1(loop+1);
       end

    end
end
    
    fclose(comport);
    delete(comport);