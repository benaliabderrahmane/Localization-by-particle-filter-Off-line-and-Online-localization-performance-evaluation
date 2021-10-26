function [Particles]=Particles_generator(xmin,xmax,ymin,ymax,theta_min,theta_max,N,Obstacles)
clc 
Particles=zeros(3,N); %array (3xN) 
sprintf('initial Particles')
disp(Particles)
for i=1:N
    while Particles(1,i)==0
        P(1,i)=(xmax-xmin)*rand+xmin; %Particle.x 
        P(2,i)=(ymax-ymin)*rand+ymin; %Particle.y
        P(3,i)=(theta_max-theta_min)*rand+theta_min; %Particle.theta
        test=isinBoxmap(P(1,i),P(2,i),Obstacles);
        if test==1
            Particles(1,i)=P(1,i);
            Particles(2,i)=P(2,i);
            Particles(3,i)=P(3,i);
            i=i+1;
        end
        if i==N+1
            break
        end
    end
end 
% P1=[[xmin;ymax],[xmax;ymax],[xmax;ymin],[xmin;ymin],[xmin;ymax]];
disp('------------------------------------------------------------------------------------------------------------')
disp(Particles)
