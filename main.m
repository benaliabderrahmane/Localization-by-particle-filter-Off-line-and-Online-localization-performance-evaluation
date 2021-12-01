clear
close all
clc


%% loop over all combinations of (Likelihood, Selection, Distribution) using same (NParticles, PtDepart, NCapteur) : 

AllOptions.Likelihood=["likelihood1"];
AllOptions.Selection=["Selection1","Selection2"];
AllOptions.Distribution=["distance","standard Deviation"];
AllOptions.SensorsType=["laser","laser front","US"]; %Laser 360° laser1 only from -30 to 210 (240 overall) LAdar for US only.
AllOptions.NParticles=[5];
AllOptions.StartPoint=[27.5;15;-pi/2];
AllOptions.EndPoint=[27;5;0];
AllOptions.NPp=2;
AllOptions.MaxSpeed=2.6;
AllOptions.NR = 16; %number of rays
AllOptions.plot = 1; %bool 1 plot 0 do not plot


size = max([length(AllOptions.Likelihood),length(AllOptions.Selection),length(AllOptions.Distribution)]);

for i=1:1%length(AllOptions.Likelihood)
    for j=1:1%length(AllOptions.Selection)
        for k=2:length(AllOptions.Distribution)
            for ii = 1:length(AllOptions.NParticles)
                % for each StudyCase
                Options.Likelihood = AllOptions.Likelihood(i);
                Options.Selection = AllOptions.Selection(j);
                Options.Distribution = AllOptions.Distribution(k);
                Options.NParticles = AllOptions.NParticles(ii);
                Options.SensorsType = AllOptions.SensorsType(2);
                Options.NPP = AllOptions.NPp;
                Options.MaxSpeed = AllOptions.MaxSpeed;
                Options.plot = AllOptions.plot;
                Options.NR = AllOptions.NR;
                Options.StartPoint=AllOptions.StartPoint;
                Options.EndPoint=AllOptions.EndPoint;
                %lunch tests
                disp('start simulating with:')
                str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR))
                str = regexprep(str,'[^0-9a-zA-Z]','_');
                sprintf(str);
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
        


