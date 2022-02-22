clc


%% first we need to get the initial points then we generate trajectories
xmin = 26.5747;
xmax = 29.02;
ymin = -0.269984;
ymax = 56;
trajectories = [];
points = [];
i = 1;
for x = xmin+1:0.5:xmax-1
    for y = ymin+1:ymax
        points = [points;x y];
        if y>10
            trajectories(i,:,:) = [x,x;y,y+5;-pi/2,-pi/2];
            i=i+1;
        else
            trajectories(i,:,:) = [x,x;y,y-5;pi/2,pi/2];
            i=i+1;
        end
    end
end

xmin = -5;
xmax = 26.5747;
ymin = -0.269984;
ymax = 3;

for x = xmin:0.5:xmax
    for y = ymin+1:ymax-1
        points = [points;x y];
                if x<22
            trajectories(i,:,:) = [x,x-5;y,y;pi,pi];
            i=i+1;
        else
            trajectories(i,:,:) = [x,x+5;y,y;0,0];
            i=i+1;
        end
    end
end

%% we then use particle filter
% add a condition for stop for optimisation => done in particle filter
% may be do three test and get the min 
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
global GrandObstacle
load('GrandObstacle.mat')

%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Roulette wheel","Stochastic universel sampling"];
AllOptions.Distribution=["distance","standard Deviation","WHEIGHTED STANDARD DEVIATION"];
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

particles=Particles_generator(26.5747,29.02,-0.269984,55,-pi,pi,1000,Obstacles);

result = zeros(5,10); %result(i,j)
for i=1:5
    for j=1:10%length(AllOptions.NParticles)  
        particles_temp = particles(:,1:AllOptions.NParticles(j));
        for jj=2:2%length(AllOptions.Selection)
            % for each StudyCase
            here = tic;
            Options.Likelihood = AllOptions.Likelihood(1);
            Options.Selection = AllOptions.Selection(jj);
            Options.Distribution = AllOptions.Distribution(3);
            Options.NParticles = AllOptions.NParticles(j);
            Options.SensorsType = AllOptions.SensorsType(1);
            Options.NPP = AllOptions.NPp;
            Options.MaxSpeed = AllOptions.MaxSpeed;
            Options.plot = AllOptions.plot;
            Options.NR = AllOptions.NR(3);
            Options.StartPoint=squeeze(trajectories(i,:,1));
            Options.EndPoint=AllOptions.EndPoint;
            Options.PP = trajectories(i,:,:);
            Options.particles = particles_temp;
            
            disp('start simulating with:')
            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory number ",num2str(i)," test NPP")
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

% subD(3,2,2)
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
 
 
 
x = [100,200,300,400,500,600,700,800,900,1000];
 for i=1:5
     figure()
     y = result(i,:);
     bar(x,y)
     xlabel("Number of particles")
     ylabel("iteration of convergance")
     grid on
 end
 
