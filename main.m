
close all
clc



%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"]; % gaussienne (normal) distribution
AllOptions.Selection=["Stochastic universel sampling","Roulette wheel"];
AllOptions.Distribution=["distance","standard Deviation"];
AllOptions.SensorsType=["laser","laser front","US"]; %Laser 360° laser1 only from -30 to 210 (240 overall) LAdar for US only.
AllOptions.NParticles=[50 100 250 400];
AllOptions.EndPoint=[0; 1; 0];
AllOptions.NPp=20;
AllOptions.MaxSpeed=0.4;
AllOptions.NR = [16 32 48 64]; %number of rays
AllOptions.plot = 1; %bool 1 plot 0 do not plot


size = max([length(AllOptions.Likelihood),length(AllOptions.Selection),length(AllOptions.Distribution)]);

for i=1:30
    for j=1:length(AllOptions.NParticles)
        for k=1:length(AllOptions.NR)
                for ii = 1:length(AllOptions.Distribution)
                    for jj=1:length(AllOptions.Selection)
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
                            disp('start simulating with:')
                            str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " trajectory number ",num2str(i))
                            str = regexprep(str,'[^0-9a-zA-Z]','_');
                            Data = ParticleFilter(Options);

                            %% 
                            %add code to save data here
                            filename = strcat("data\",str,".mat");
                            save(filename,"Data")
                            %save(str1,"Data");
                            %save(str2,"Options");
                            disp('save, end case')
                            disp('--------------------------------------')
                    end
                end
            end
        end
    end
end
        


