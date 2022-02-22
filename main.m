
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
Sobst=size(Obstacles);

%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Roulette wheel","Stochastic universel sampling"];
AllOptions.Distribution=["distance","standard Deviation"];
%Laser 360° laser1 only from -30 to 210 (240 overall), US for ultrasound 
%US front for the eight front sensors and US mix is 1-0-1-0 one us activate
%the other no...etc
AllOptions.SensorsType=["laser","laser front","US","US front", "US mix"];
AllOptions.NParticles=[100 250 500 750 1000];
AllOptions.EndPoint=[0; 1; 0];
AllOptions.NPp=1;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [16 32]; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot



for i=10:18
    for j=1:length(AllOptions.NParticles)                      
        particles1=Particles_generator(26.5747,29.02,-0.269984,56,-pi,pi,floor(AllOptions.NParticles(j)/2),Obstacles);
        particles2=Particles_generator(-5,26.5747,-0.269984,3,-pi,pi,AllOptions.NParticles(j)-floor(AllOptions.NParticles(j)/2),Obstacles);
        particles=[particles1,particles2];
        for k=1:length(AllOptions.NR)
            for ii = 1:length(AllOptions.Distribution)
                for jj=1:length(AllOptions.Selection)
                   for kk=1:length(AllOptions.SensorsType)
                        % for each StudyCase
                        here = tic
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
                        Options.particles = particles;
                        %lunch tests
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
                        toc(here) 
                        disp('--------------------------------------')
                    end
                end
            end
        end
    end
end



