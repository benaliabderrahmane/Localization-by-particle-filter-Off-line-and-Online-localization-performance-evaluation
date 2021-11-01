function [Particles]=Particles_generator(xmin,xmax,ymin,ymax,theta_min,theta_max,N,Obstacles)
    Particles=zeros(3,N); %array (3xN) 
    P = zeros(3,1);%array (3x1)
    i = 1;
    while i<=N
        P(1,1)=(xmax-xmin)*rand+xmin; %Particle.x 
        P(2,1)=(ymax-ymin)*rand+ymin; %Particle.y
        P(3,1)=(theta_max-theta_min)*rand+theta_min; %Particle.theta
        test=isinBoxmap(P(1,1),P(2,1),Obstacles);
        if test==1
            Particles(:,i)=P;
            i=i+1;
        end
    end
    %figure(10)
    % P1=[[xmin;ymax],[xmax;ymax],[xmax;ymin],[xmin;ymin],[xmin;ymax]];
    %plot(P1(1,:),P1(2,:))
end 

