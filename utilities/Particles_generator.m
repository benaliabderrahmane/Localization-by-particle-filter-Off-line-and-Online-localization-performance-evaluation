%xmin,xmax:
%ymin,ymax:
%N: number of particles(points)
function [Particles]=Particles_generator(xmin,xmax,ymin,ymax,theta_min,theta_max,N,Obstacles)
Particles=[];
i=1;
while size(Particles,2)<N
P(1,1)=(xmax-xmin)*rand(1)+xmin; %c'est la coordonnee x de la particule candidate
P(2,1)=(ymax-ymin)*rand(1)+ymin;%c'est la coordonnee y de la particule candidate
P(3,1)=(theta_max-theta_min)*rand(1)+theta_min;%c'est l'orientation de la particule candidate
test=isinBoxmap(P(1,1),P(2,1),Obstacles);
if test==1
    Particles=[Particles,P];
    i=i+1;
end
%figure(10)

P1=[[xmin;ymax],[xmax;ymax],[xmax;ymin],[xmin;ymin],[xmin;ymax]];
%plot(P1(1,:),P1(2,:))
end



