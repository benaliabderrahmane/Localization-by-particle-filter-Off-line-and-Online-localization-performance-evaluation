function[P_new]=testInext(iNextGeneration,Particles,Obstacles)
global N;
N=0;
Particles_new=[];
IN=sort(iNextGeneration);
k=1;
cmpt=1;
for i=1:(size(IN,1)-1)
    if (IN(i+1)==IN(i))
        cmpt=cmpt+1;
    else
        Tab(k,:)=[IN(i),cmpt];
        k=k+1;
        cmpt=1;
    end
end
Tab(k,:)=[IN(i+1),cmpt];
for i=1:size(Tab,1)
T=Tab(i,1);


    xmin=Particles.x(T)-0.25;
    xmax=Particles.x(T)+0.25;
    ymin=Particles.y(T)-0.25;
    ymax=Particles.y(T)+0.25;    
theta_min=Particles.theta(T)-pi/8;
theta_max=Particles.theta(T)+pi/8;
N_new=round(coefficients(Particles)*Tab(i,2));
Particles_new=[Particles_new,Particles_generator(xmin,xmax,ymin,ymax,theta_min,theta_max,N_new,Obstacles)];
N=N+N_new;
end
P_new.x=Particles_new(1,:);
P_new.y=Particles_new(2,:);
P_new.theta=Particles_new(3,:);
end
