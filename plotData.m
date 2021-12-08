%% simulation part 
figure(10)
clc
close all
addpath('./affichage/');
addpath('./data/');
addpath('./likelihood/');
addpath('./redistribution (resampling)/');
addpath('./robot/');
addpath('./selection/');
addpath('./sensors/');
addpath('./utilities/');

%load('C:\Users\benal\Dropbox\PC\Documents\projet master\Bibliographie\Code\-Localization-by-particle-filter--Off-line-and-Online-localization-performance-evaluation\data\bat5_Obstacles_detect_redone140220.mat')
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
length(Particles)
a = Data.vecteur_Poids;
weights = Data.vecteur_Poids(1:length(Particles),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robPoints=plot(Robot(1),Robot(2),'ob','MarkerSize',10,'LineWidth',2);
robOr = plot([Robot(1),Robot(1)+1.5*cos(Robot(3))],[Robot(2),Robot(2)+1.5*sin(Robot(3))],'b','LineWidth',2)
%particPoints=scatter(Particles(1,:),Particles(2,:),weights/norm(weights)*200,'r');
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'*k')
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'k')

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
    weights = Data.vecteur_Poids(length(Particles)*(i-1)+1:length(Particles)*i,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(robPoints,'XData',Robot(1));
    set(robPoints,'YData',Robot(2));
    set(robOr,'XData',[Robot(1),Robot(1)+1.5*cos(Robot(3))]);
    set(robOr,'YData',[Robot(2),Robot(2)+1.5*sin(Robot(3))]);
    %set(particPoints,'SizeData',weights/norm(weights)*200);
    %set(particPoints,'XData',Particles(1,:));
    %set(particPoints,'YData',Particles(2,:));
    pause(0.1)
    b = 1;
end


%% plot error and uncertainty graphs

addpath('./data/');


AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Roulette wheel","Stochastic universel sampling"];
AllOptions.Distribution=["distance","standard Deviation"];
AllOptions.SensorsType=["laser","laser front","US","US front", "US mix"];
AllOptions.NParticles=[250 500 750 1000];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [8 16 32]; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot
for i=1:1%18
    for j=1:1%length(AllOptions.NParticles)
        for k=1:1%length(AllOptions.NR)
                for ii = 1:1%length(AllOptions.Distribution)
                    for jj=1:1%length(AllOptions.Selection)
                        for kk=1:5%length(AllOptions.SensorsType)
                            % change this before any plot
                            iii = kk-1; 
                            maxiteration = 100;%length(Data.vecteur_incertitude_x(1,:));
                            iiimax = length(AllOptions.SensorsType);
                            
                            Options.Likelihood = AllOptions.Likelihood(1);
                            Options.Selection = AllOptions.Selection(jj);
                            Options.Distribution = AllOptions.Distribution(ii);
                            Options.NParticles = AllOptions.NParticles(j);
                            Options.SensorsType = AllOptions.SensorsType(kk);
                            Options.NPP = AllOptions.NPp;
                            Options.MaxSpeed = AllOptions.MaxSpeed;
                            Options.plot = AllOptions.plot;
                            Options.NR = AllOptions.NR(k);
                            Options.StartPoint=squeeze(trajectories(i,:,1));
                            Options.EndPoint=AllOptions.EndPoint;
                            Options.PP = trajectories(i,:,:);
                            %lunch tests
                            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory2 number ",num2str(i))
                            str = regexprep(str,'[^0-9a-zA-Z]','_');
                            filename = strcat(str,".mat");
                            load(filename)
                            figure(1)
                            subplot(3,iiimax,1+iii)
                            errorbar([],Data.vecteur_erreur(1,1:maxiteration),Data.vecteur_incertitude_x(1,1:maxiteration),Data.vecteur_incertitude_x(2,1:maxiteration),'-s','MarkerSize',2,'MarkerEdgeColor','b','MarkerFaceColor','b')
                            xlabel('iterations')
                            ylabel('x(m)')
                            str = strcat("errors and uncertainties for case: ",num2str(iii+1));
                            title(str)
                            hold on 
                            subplot(3,iiimax,1+iiimax+iii)
                            errorbar([],Data.vecteur_erreur(2,1:maxiteration),Data.vecteur_incertitude_y(1,1:maxiteration),Data.vecteur_incertitude_y(2,1:maxiteration),'-s','MarkerSize',2,'MarkerEdgeColor','b','MarkerFaceColor','b')
                            xlabel('iterations')
                            ylabel('y(m)')
                            hold on 
                            subplot(3,iiimax,1+iiimax*2+iii)
                            errorbar([],Data.vecteur_erreur(3,1:maxiteration),Data.vecteur_incertitude_theta(1,1:maxiteration),Data.vecteur_incertitude_theta(2,1:maxiteration),'-s','MarkerSize',2,'MarkerEdgeColor','b','MarkerFaceColor','b')
                            xlabel('iterations')
                            ylabel('theta(rad)')
                            hold on 
                    end
                end
            end
        end
    end
end

%% plot error graphs

addpath('./data/');


AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Roulette wheel","Stochastic universel sampling"];
AllOptions.Distribution=["distance","standard Deviation"];
AllOptions.SensorsType=["laser","laser front","US","US front", "US mix"];
AllOptions.NParticles=[250 500 750 1000];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [8 16 32]; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot
for i=1:1%18
    for j=1:1%length(AllOptions.NParticles)
        for k=1:1%length(AllOptions.NR)
                for ii = 1:1%length(AllOptions.Distribution)
                    for jj=1:1%length(AllOptions.Selection)
                        for kk=1:length(AllOptions.SensorsType)
                            % for each StudyCase
                            Options.Likelihood = AllOptions.Likelihood(1);
                            Options.Selection = AllOptions.Selection(jj);
                            Options.Distribution = AllOptions.Distribution(ii);
                            Options.NParticles = AllOptions.NParticles(j);
                            Options.SensorsType = AllOptions.SensorsType(kk);
                            Options.NPP = AllOptions.NPp;
                            Options.MaxSpeed = AllOptions.MaxSpeed;
                            Options.plot = AllOptions.plot;
                            Options.NR = AllOptions.NR(k);
                            Options.StartPoint=squeeze(trajectories(i,:,1));
                            Options.EndPoint=AllOptions.EndPoint;
                            Options.PP = trajectories(i,:,:);
                            %lunch tests
                            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory2 number ",num2str(i))
                            str = regexprep(str,'[^0-9a-zA-Z]','_');
                            filename = strcat(str,".mat");
                            load(filename)
                            iii = kk-1;
                            maxiteration = 50;%length(Data.vecteur_incertitude_x(1,:));
                            iiimax = 5;
                            figure(1)
                            subplot(3,iiimax,1+iii)
                            plot(Data.vecteur_erreur(1,1:maxiteration))
                            xlabel('iterations')
                            ylabel('x(m)')
                            str = strcat("errors for case: ",num2str(iii+1));
                            title(str)
                            hold on 
                            subplot(3,iiimax,1+iiimax+iii)
                            plot(Data.vecteur_erreur(2,1:maxiteration))
                            xlabel('iterations')
                            ylabel('y(m)')
                            hold on 
                            subplot(3,iiimax,1+iiimax*2+iii)
                            plot(Data.vecteur_erreur(3,1:maxiteration))
                            xlabel('iterations')
                            ylabel('theta(rad)')
                            hold on 
                    end
                end
            end
        end
    end
end
