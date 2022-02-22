close all
clc
addpath('./affichage/');
addpath('./data/');
addpath('./likelihood/');
addpath('./redistribution (resampling)/');
addpath('./robot/');
addpath('./selection/');
addpath('./sensors/');
addpath('./utilities/');
load('bat5_Obstacles_detect_redone140220.mat');
load('trajectories.mat')
global GrandObstacle
load('GrandObstacle.mat')
%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Roulette wheel","Stochastic universel sampling"];
AllOptions.Distribution=["distance","standard Deviation"];
%Laser 360° laser1 only from -30 to 210 (240 overall), US for ultrasound 
%US front for the eight front sensors and US mix is 1-0-1-0 one us activate
%the other no...etc
AllOptions.SensorsType=["laser","laser front","US","US front", "US mix"];
AllOptions.NParticles=[100 250 500 750];
AllOptions.EndPoint=[0; 1; 0];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [8 16 32]; %number of rays
AllOptions.plot = 1; %bool 1 plot 0 do not plot

performanceIteration = zeros(4,18,2);
for i=1:18%18
    if (i == 18 || i == 17 || i == 16 || i == 15 || i == 14 || i == 9 || i == 7 || i == 5 || i == 13 || i == 10 || i == 8 || i == 6)
        continue;
    end
    
    for j=4:4%length(AllOptions.NParticles)  
        particles1=Particles_generator(26.5747,29.02,-0.269984,56,-pi,pi,floor(AllOptions.NParticles(j)/2),Obstacles);
        particles2=Particles_generator(-5,26.5747,-0.269984,3,-pi,pi,AllOptions.NParticles(j)-floor(AllOptions.NParticles(j)/2),Obstacles);
        particles=[particles1,particles2];
        for jj=2:2%length(AllOptions.Selection)
            % for each StudyCase
            here = tic;
            Options.Likelihood = AllOptions.Likelihood(1);
            Options.Selection = AllOptions.Selection(jj);
            Options.Distribution = AllOptions.Distribution(1);
            Options.NParticles = AllOptions.NParticles(j);
            Options.SensorsType = AllOptions.SensorsType(1);
            Options.NPP = AllOptions.NPp;
            Options.MaxSpeed = AllOptions.MaxSpeed;
            Options.plot = AllOptions.plot;
            Options.NR = AllOptions.NR(2);
            Options.StartPoint=squeeze(trajectories(i,:,1));
            Options.EndPoint=AllOptions.EndPoint;
            Options.PP = trajectories(i,:,:);
            Options.particles = particles;
            
            disp('start simulating with:')
            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory3 number ",num2str(i))
            str = regexprep(str,'[^0-9a-zA-Z]','_');
            %% 
            %add code to save data here
            filename = strcat("data\",str,".mat");
            if isfile(filename)
                %check if we already tested this case no need to repeat it 
                load(filename)
                %break 
            else
                %lunch tests
                Data = ParticleFilter(Options);
                save(filename,"Data")
                disp('save, end case')
            end
            
%%

for iiiii =1:length(Data.vecteur_particles(3,:))
Data.vecteur_particles(3,iiiii) =AngleWrap(Data.vecteur_particles(3,iiiii));
end

for iiiii =1:length(Data.vecteur_Robot(3,:))
Data.vecteur_Robot(3,iiiii)=AngleWrap(Data.vecteur_Robot(3,iiiii));
end

sdw = zeros(3,length(Data.vecteur_estimation));
cr2  = zeros(3,length(Data.vecteur_estimation));

PoseEstimeX     = zeros(1,length(Data.vecteur_estimation));
PoseEstimeY     = zeros(1,length(Data.vecteur_estimation));
PoseEstimeTheta = zeros(1,length(Data.vecteur_estimation));

robotX     = zeros(1,length(Data.vecteur_estimation));
robotY     = zeros(1,length(Data.vecteur_estimation));
robotTheta = zeros(1,length(Data.vecteur_estimation));

jjj = 1;
kkkk = 0;
jjjj = 1;

for kkk=1:length(Data.iteration)-1
    NN = Data.N_Particles(kkk);
    tempx = 0;
    tempy = 0;
    tempTheta = 0;
    
    dataX     = Data.vecteur_particles(1,jjj:jjj+NN);
    dataY     = Data.vecteur_particles(2,jjj:jjj+NN);
    dataTheta = Data.vecteur_particles(3,jjj:jjj+NN);
    
    index=Data.vecteur_particles(1,jjj:jjj+NN)==Inf;
    
    meanx     = sum(dataX(1,index==0))/sum(~index);
    meany     = sum(dataY(1,index==0))/sum(~index);
    meanTheta = sum(dataTheta(1,index==0))/sum(~index);
    
    for iiii=jjj:NN+jjj
        if isinf(Data.vecteur_particles(1,iiii))
            continue 
        end
        tempx = tempx + Data.vecteur_Poids(iiii)*(Data.vecteur_particles(1,iiii)-meanx)^2;
        tempy = tempy + Data.vecteur_Poids(iiii)*(Data.vecteur_particles(2,iiii)-meany)^2;
        tempTheta = tempTheta + Data.vecteur_Poids(iiii)*(Data.vecteur_particles(3,iiii)-meanTheta)^2;
    end
    kkkk = kkkk+1;
    if(kkkk==3)
    
    PoseEstimeX(jjjj) = sum(Data.vecteur_particles(1,jjj:jjj+NN)*Data.vecteur_Poids(jjj:jjj+NN))/sum(Data.vecteur_Poids(jjj:jjj+NN));
    PoseEstimeY(jjjj) = sum(Data.vecteur_particles(2,jjj:jjj+NN)*Data.vecteur_Poids(jjj:jjj+NN))/sum(Data.vecteur_Poids(jjj:jjj+NN));
    PoseEstimeTheta(jjjj) = sum(Data.vecteur_particles(3,jjj:jjj+NN)*Data.vecteur_Poids(jjj:jjj+NN))/sum(Data.vecteur_Poids(jjj:jjj+NN));
    
    sdw(1,jjjj) = sqrt(tempx/((NN-1)/NN*sum(Data.vecteur_Poids(jjj:jjj+NN)))); 
    sdw(2,jjjj) = sqrt(tempy/((NN-1)/NN*sum(Data.vecteur_Poids(jjj:jjj+NN))));
    sdw(3,jjjj) = sqrt(tempTheta/((NN-1)/NN*sum(Data.vecteur_Poids(jjj:jjj+NN))));

    cr2(1,jjjj)=max(dataX(1,index==0))-min(dataX(1,index==0));
    cr2(2,jjjj)=max(dataY(1,index==0))-min(dataY(1,index==0));
    cr2(3,jjjj)=max(dataTheta(1,index==0))-min(dataTheta(1,index==0));
    
    
    
    kkkk=0;
    jjjj = jjjj+1;
    robotX(jjjj) = Data.vecteur_Robot(1,kkk);
    robotY(jjjj) = Data.vecteur_Robot(2,kkk);
    robotTheta(jjjj) = Data.vecteur_Robot(3,kkk);
    end
    jjj = jjj+NN;
end
figure()
subplot(3,2,1)
plot(sdw(1,1:50),'LineWidth',2)
hold on 
grid on
plot( abs(-robotX(1,1:50)+Data.vecteur_estimation(1,1:50)),'LineWidth',2)
legend("weighted standard deviation for x(m)","absotute error for x(m)")
xlabel("iteration")
title(str)
subplot(3,2,3)

plot(sdw(2,1:50),'LineWidth',2)
hold on 
grid on
plot( abs(-robotY(1,1:50)+Data.vecteur_estimation(2,1:50)),'LineWidth',2)
legend("weighted standard deviation for y(m)","absotute error for y(m)")
xlabel("iteration")
title(str)

subplot(3,2,5)
plot(sdw(3,1:50),'LineWidth',2)
hold on 
grid on
plot( abs(-robotTheta(1,1:50)+Data.vecteur_estimation(3,1:50)),'LineWidth',2)
legend("weighted standard deviation for theta (radian)","absotute error for theta (radian)")
xlabel("iteration")
title(str)


subplot(3,2,2)
plot(cr2(1,1:50),'LineWidth',2)
hold on 
grid on
plot( abs(-robotX(1,1:50)+Data.vecteur_estimation(1,1:50)),'LineWidth',2)
legend("range of x(m)","absotute error for x(m)")
xlabel("iteration")
title(str)
subplot(3,2,4)

plot(cr2(2,1:50),'LineWidth',2)
hold on 
grid on
plot( (-robotY(1,1:50)+Data.vecteur_estimation(2,1:50)),'LineWidth',2)
legend("range of y (m)","absotute error for y(m)")
xlabel("iteration")
title(str)

subplot(3,2,6)
plot(cr2(3,1:50),'LineWidth',2)
hold on 
grid on
plot(-robotTheta(1,1:50)+Data.vecteur_estimation(3,1:50),'LineWidth',2)
legend("range of theta (radian)","absotute error for theta (radian)")
xlabel("iteration")
title(str)
            disp('--------------------------------------')
            
        end
    end
end