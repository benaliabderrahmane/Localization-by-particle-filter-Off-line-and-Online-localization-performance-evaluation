clear
close all
clc


%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1","likelihood2"];
AllOptions.Selection=["Selection1","Selection2"];
AllOptions.Distribution=["Distribution1","Distribution2"];
AllOptions.NParticles=[10];
AllOptions.StartPoint=[0;0;0];
AllOptions.SensorsType="Lidar";
AllOptions.NPp=[5];



size = max([length(AllOptions.Likelihood),length(AllOptions.Selection),length(AllOptions.Distribution)]);

for i=1:length(AllOptions.Likelihood)
    for j=1:length(AllOptions.Selection)
        for k=1:length(AllOptions.Distribution)
            % for each StudyCase
            sprintf('Particle Filter using %s %s %s %s %s %s',AllOptions.Likelihood(i),AllOptions.Selection(j),AllOptions.Distribution(k),AllOptions.NParticles,AllOptions.StartPoint,AllOptions.SensorsType,AllOptions.NPp);
            Options.Likelihood = AllOptions.Likelihood(i);
            Options.Selection = AllOptions.Selection(i);
            Options.Distribution = AllOptions.Distribution(k);
            Options.NParticles = AllOptions.NParticles;
            Options.StartPoint = AllOptions.StartPoint;
            Options.SensorsType = AllOptions.SensorsType;
            Options.NPP = AllOptions.NPp;
            
            %lunch tests
            Data = ParticleFilter(Options);
            disp('save, end case')
            %% 
            %add code to save data here
            disp('--------------------------------------')
        end
    end
end
        


