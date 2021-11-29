figure(10)
clc
close all
load('C:\Users\benal\Dropbox\PC\Documents\projet master\Bibliographie\Code\-Localization-by-particle-filter--Off-line-and-Online-localization-performance-evaluation\data\bat5_Obstacles_detect_redone140220.mat')
plot_Environement(Obstacles,10);%affichage de l'environnement
n = length(Data.vecteur_Robot);
Robot = Data.vecteur_Robot(:,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initialize Particles vector
flag = 1;
k=2;
while (flag == 1)
    if (Data.vecteur_particles(1,k)~=inf)
        k =k+1;
    else 
       flag = 0;
    end
end
Particles = Data.vecteur_particles(:,2:k-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robPoints=plot(Robot(1),Robot(2),'o');
particPoints=plot(Particles(1,:),Particles(2,:),'.r');
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'*k')
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'k')

%%
for i=2:n
    Robot = Data.vecteur_Robot(:,i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %update Particles vector
    flag = 1;
    m=k+1;
    k=k+1;
    while (flag == 1 && k<=length(Data.vecteur_particles))
        if (Data.vecteur_particles(1,k)~=inf)
            k =k+1;
        else 
           flag = 0;
        end
    end
    Particles = Data.vecteur_particles(:,m:k-1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(robPoints,'XData',Robot(1));
    set(robPoints,'YData',Robot(2));    
    set(particPoints,'XData',Particles(1,:));
    set(particPoints,'YData',Particles(2,:));
    pause(0.02)
    b = 1;
end