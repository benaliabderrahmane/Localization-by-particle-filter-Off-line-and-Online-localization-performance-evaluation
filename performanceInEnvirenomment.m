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


%% test N particles effect save in trajectory (do only 75 iterations).
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
AllOptions.Distribution=["distance","standard Deviation"];
%Laser 360° laser1 only from -30 to 210 (240 overall), US for ultrasound 
%US front for the eight front sensors and US mix is 1-0-1-0 one us activate
%the other no...etc
AllOptions.SensorsType=["laser","laser front","US","US front", "US mix"];
AllOptions.NParticles=[100 200 300 400 500 600 700 800 900 1000];
AllOptions.EndPoint=[0; 1; 0];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [8 16 32]; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot
performanceIteration = zeros(4,18,2);


particles1=Particles_generator(26.5747,29.02,-0.269984,56,-pi,pi,300,Obstacles);
particles2=Particles_generator(-5,26.5747,-0.269984,3,-pi,pi,300,Obstacles);
particles=[particles1,particles2];

for m=1:1
    for i=1:184
        for j=6:6%length(AllOptions.NParticles)
            for jjj=5:5
                % for each StudyCase
                here = tic;
                Options.Likelihood = AllOptions.Likelihood(1);
                Options.Selection = AllOptions.Selection(2);
                Options.Distribution = AllOptions.Distribution(2);
                Options.NParticles = AllOptions.NParticles(j);
                Options.SensorsType = AllOptions.SensorsType(jjj);
                Options.NPP = AllOptions.NPp;
                Options.MaxSpeed = AllOptions.MaxSpeed;
                Options.plot = AllOptions.plot;
                Options.NR = AllOptions.NR(2);
                Options.StartPoint=squeeze(trajectories(i,:,1));
                Options.EndPoint=AllOptions.EndPoint;
                Options.PP = trajectories(i,:,:);
                Options.particles = particles;

                disp('start simulating with:')
                str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory number ",num2str(i)," test number ",num2str(m), " evualte particle filter in envirennement")
                str = regexprep(str,'[^0-9a-zA-Z]','_');
                %% 
                %add code to save data here
                filename = strcat("data\",str,".mat");
                if isfile(filename)
                    %check if we already tested this case no need to repeat it 
                    load(filename);
                    %break 
                    C(i) = length(Data.iteration);
                else
                    %lunch tests
                    Data = ParticleFilter(Options);
                    save(filename,"Data")
                    C(i) = length(Data.iteration);
                    disp('save, end case');
                end
            end
        end
    end
end
%% plot envirenoment
plot_Environement(Obstacles,1)


%% draw the map 

 scatter(points(:,1),points(:,2), 85, C, 'filled')
%scatter(points(:,1),points(:,2),85, C,'filled', 'Marker', 's')
% Apply to the plot
 cMap = interp1([0;1],[0 1 0; 1 0 0],linspace(0,1,256));
 colormap(cMap)
 colorbar

