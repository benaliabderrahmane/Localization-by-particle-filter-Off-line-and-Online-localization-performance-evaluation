clear
close all
clc


%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"];
AllOptions.Selection=["Selection1","Selection2"];
AllOptions.Distribution=["distance","standard Deviation"];
AllOptions.SensorsType=["laser","laser front","US"]; %Laser 360° laser1 only from -30 to 210 (240 overall) LAdar for US only.
AllOptions.NParticles=[5];
AllOptions.StartPoint=[27.5;50;-pi/2];
AllOptions.EndPoint=[27;5;0];
AllOptions.NPp=5;
AllOptions.MaxSpeed=0.6;
AllOptions.NR = 16; %number of rays
AllOptions.plot = 0; %bool 1 plot 0 do not plot


size = max([length(AllOptions.Likelihood),length(AllOptions.Selection),length(AllOptions.Distribution)]);

for i=1:1%length(AllOptions.Likelihood)
    for j=1:1%length(AllOptions.Selection)
        for k=2:length(AllOptions.Distribution)
            for ii = 1:length(AllOptions.NParticles)
                % for each StudyCase
                sprintf('Particle Filter using %s %s %s %s %s %s',AllOptions.Likelihood(i),AllOptions.Selection(j),AllOptions.Distribution(k),AllOptions.NParticles,AllOptions.StartPoint,AllOptions.SensorsType,AllOptions.NPp);
                Options.Likelihood = AllOptions.Likelihood(i);
                Options.Selection = AllOptions.Selection(j);
                Options.Distribution = AllOptions.Distribution(k);
                Options.NParticles = AllOptions.NParticles(ii);
                Options.StartPoint = AllOptions.StartPoint;
                Options.SensorsType = AllOptions.SensorsType(1);
                Options.NPP = AllOptions.NPp;
                Options.MaxSpeed = AllOptions.MaxSpeed;
                Options.plot = AllOptions.plot;
                Options.NR = AllOptions.NR;
                Options.StartPoint=AllOptions.StartPoint;
                Options.EndPoint=AllOptions.EndPoint;
                %lunch tests
                Data = ParticleFilter(Options);
                disp('save, end case')
                %% 
                %add code to save data here
                t = datetime('now');
                tt = datestr(t);
                filename=regexprep(tt,'[^0-9a-zA-Z]','_');
                str1 = strcat("data\",filename,"_data",".mat");
                str2 = strcat("data\",filename,"_options",".mat");
                % we need to use this instead to keep track of options
                % to be tested and verified later
                %str = strcat(Options.Likelihood,Options.Selection,num2str(Options.NParticles),num2str(Options.StartPoint),".mat")
                %save(str,"Data")
                save(str1,"Data");
                save(str2,"Options");
                disp('--------------------------------------')
            end
        end
    end
end
        


