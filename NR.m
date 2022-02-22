close all
clc
%% test N particles effect save in trajectory 4 (do only 75 iterations).
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
AllOptions.NParticles=[100 200 300 400 500 600 700 800 900 1000];
AllOptions.EndPoint=[0; 1; 0];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [8 16 32 64]; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot
performanceIteration = zeros(4,18,2);
particles=Particles_generator(-5,26.5747,-0.269984,3,-pi,pi,200,Obstacles);
result = zeros(4,4); %result(i,j)
for i=5:8
    for j=1:4%length(AllOptions.NParticles)  
        particles_temp = particles(:,1:AllOptions.NParticles(2));
        for jj=2:2%length(AllOptions.Selection)
            % for each StudyCase
            here = tic;
            Options.Likelihood = AllOptions.Likelihood(1);
            Options.Selection = AllOptions.Selection(jj);
            Options.Distribution = AllOptions.Distribution(2);
            Options.NParticles = AllOptions.NParticles(2);
            Options.SensorsType = AllOptions.SensorsType(1);
            Options.NPP = AllOptions.NPp;
            Options.MaxSpeed = AllOptions.MaxSpeed;
            Options.plot = AllOptions.plot;
            Options.NR = AllOptions.NR(j);
            Options.StartPoint=squeeze(trajectories(i,:,1));
            Options.EndPoint=AllOptions.EndPoint;
            Options.PP = trajectories(i,:,:);
            Options.particles = particles_temp;
            
            disp('start simulating with:')
            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory0 number ",num2str(i))
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

result(i,j) = length(Data.iteration);
        end
    end
end




%%

% subplot(3,2,2)
% plot(cr2(1,1:50),'LineWidth',2)
% hold on 
% grid on
% plot( abs(-robotX(1,1:50)+Data.vecteur_estimation(1,1:50)),'LineWidth',2)
% legend("range of x(m)","absotute error for x(m)")
result = sort(result','descend')'; % subplot(3,2,2)
% xlabel("iteration")
% title(str)
% subplot(3,2,4)
% 
% plot(cr2(2,1:50),'LineWidth',2)
% hold on 
% grid on
% plot( (-robotY(1,1:50)+Data.vecteur_estimation(2,1:50)),'LineWidth',2)
% legend("range of y (m)","absotute error for y(m)")
% xlabel("iteration")
% title(str)
% 
% subplot(3,2,6)
% plot(cr2(3,1:50),'LineWidth',2)
% hold on 
% grid on
% plot(-robotTheta(1,1:50)+Data.vecteur_estimation(3,1:50),'LineWidth',2)
% legend("range of theta (radian)","absotute error for theta (radian)")
% xlabel("iteration")
% title(str)
%             disp('--------------------------------------')
%  
 
 
 
x = [8,16,32,64];
 for i=1:4
     figure()
     y = result(i,:);
     bar(x,y)
     xlabel("Number of rayons")
     ylabel("iteration of convergance")
     grid on
 end
 
