clc


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
global GrandObstacle
load('GrandObstacle.mat')



%generate trajectory

trajectory = [27.5,27.5;30,10;-pi/2,-pi/2];

for m=1:1
    %generate particles 
    particles1=Particles_generator(26.5747,29.02,-0.269984,56,-pi,pi,300,Obstacles);
    particles2=Particles_generator(-5,26.5747,-0.269984,3,-pi,pi,300,Obstacles);
    particles=[particles1,particles2];   
    Options.Likelihood = "likelihood1";
    Options.Selection = "Stochastic universel sampling";
    Options.Distribution = "WHEIGHTED STANDARD DEVIATION";
    Options.NParticles = 600;
    Options.SensorsType = "laser";
    Options.NPP = 0;
    Options.MaxSpeed = 0.4;
    Options.plot = 1;   %% bool to plot or no during the simulation                        
    Options.NR = 16;
    Options.StartPoint=squeeze(trajectory(:,1));
    Options.EndPoint=squeeze(trajectory(:,end));
    Options.PP = trajectory(:,:);
    Options.particles = particles;

    disp('start simulating with:')
    str = strcat(Options.Likelihood," ",Options.Selection," ",Options.Distribution," ",num2str(Options.NParticles)," ",Options.SensorsType," ",num2str(Options.NPP)," ",num2str(Options.MaxSpeed)," ",num2str(Options.NR), " final simulation "," test number ",num2str(m))
    str = regexprep(str,'[^0-9a-zA-Z]','_');
    %% 
    %add code to save data here
    filename = strcat("data\",str,".mat");
     if isfile(filename)
        %check if we already tested this case no need to repeat it 
        load(filename);
        %break 
     else
        %lunch tests
        Data = ParticleFilter(Options);
        save(filename,"Data")
        disp('save, end case');
     end
end

figure(1)
subplot(3,1,1)
plot(abs(Data.vecteur_Robot(1,:)-Data.vecteur_estimation(1,:)),"linewidth",2)
hold on 
flagPlot = true;
flagPlot2 = true;
for i = 1:length(Data.vecteur_Robot(1,:))
    if (Data.vecteurFlagConvergence(i))
        if (flagPlot)
            text(i,5,"convergence",'Color','blue','FontSize',14)
            xline(i)
            flagPlot = false;
        end
    else 
        flagPlot = true;
    end       
    if (~Data.vecteurFlagRedistribution(i))
        if (flagPlot2)
            text(i,10,"redistribution",'Color','red','FontSize',14)
            xline(i)
            flagPlot2 = false;
        end
    else 
        flagPlot2 = true;
    end     
end
grid on

xlabel("iterations")
ylabel("x absolute error(m)")

subplot(3,1,2)
plot(abs(Data.vecteur_Robot(2,:)-Data.vecteur_estimation(2,:)),"linewidth",2)
hold on 
grid on
hold on 
flagPlot = true;
flagPlot2 = true;
for i = 1:length(Data.vecteur_Robot(1,:))
    if (Data.vecteurFlagConvergence(i))
        if (flagPlot)
            text(i,5,"convergence",'Color','blue','FontSize',14)
            xline(i)
            flagPlot = false;
        end
    else 
        flagPlot = true;
    end       
    if (~Data.vecteurFlagRedistribution(i))
        if (flagPlot2)
            text(i,15,"redistribution",'Color','red','FontSize',14)
            xline(i)
            flagPlot2 = false;
        end
    else 
        flagPlot2 = true;
    end     
end

xlabel("iterations")
ylabel("y absolute error(m)")


subplot(3,1,3)
plot(abs(Data.vecteur_Robot(3,:)-Data.vecteur_estimation(3,:)),"linewidth",2)
hold on 
grid on
hold on 
flagPlot = true;
flagPlot2 = true;
for i = 1:length(Data.vecteur_Robot(1,:))
    if (Data.vecteurFlagConvergence(i))
        if (flagPlot)
            text(i,0.5,"convergence",'Color','blue','FontSize',14)
            xline(i)
            flagPlot = false;
        end
    else 
        flagPlot = true;
    end       
    if (~Data.vecteurFlagRedistribution(i))
        if (flagPlot2)
            text(i,0.75,"redistribution",'Color','red','FontSize',14)
            xline(i)
            flagPlot2 = false;
        end
    else 
        flagPlot2 = true;
    end     
end
xlabel("iterations")
ylabel("theta absolute error(rad)")



%% simulation part 
figure(10)

%load('C:\Users\benal\Dropbox\PC\Documents\projet master\Bibliographie\Code\-Localization-by-particle-filter--Off-line-and-Online-localization-performance-evaluation\data\bat5_Obstacles_detect_redone140220.mat')
plot_Environement(Obstacles,10);%affichage de l'environnement
hold on
robPoints=plot(Data.vecteur_Robot(1,1),Data.vecteur_Robot(2,1),'o');
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'*k')
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'k')
particlePoints = plot(Data.vecteur_particles(1,1:600),Data.vecteur_particles(2,1:600),'.r');
legend("good detection","intermidate detection","no detection")
k = 600;

%% Initialize video
myVideo = VideoWriter('simulationVideoForParticles'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
set(gcf, 'Position', get(0, 'Screensize'));
%% Plot in a loop and grab frames
for i=2:length(Data.vecteur_Robot)
    
    pause(0.01)
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    set(robPoints,'XData',Data.vecteur_Robot(1,i));
    set(robPoints,'YData',Data.vecteur_Robot(2,i));
    set(particlePoints,'XData',Data.vecteur_particles(1,k+1:k+Data.N_Particles(i)));
    set(particlePoints,'YData',Data.vecteur_particles(2,k+1:k+Data.N_Particles(i)));
    k = k+Data.N_Particles(i);
    title("particle simulation during a mission")
end

close(myVideo)

figure(11)
%% Initialize video
myVideo2 = VideoWriter('simulationVideoForRobot'); %open video file
myVideo2.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo2)
set(gcf, 'Position', get(0, 'Screensize'));

plot_Environement(Obstacles,11);%affichage de l'environnement
xlim([10 25])
ylim([-5 10])
grid on
hold on
robPoints=plot(Data.vecteur_Robot(1,1),Data.vecteur_Robot(2,1),'o','MarkerSize',12,'color','r');
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'*k')
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'k')
estRobPoints = plot(0,0,'o','MarkerSize',12,'color','b');
legend("","","","actual robot","","","estimated robot")
k = 600;

%% Plot in a loop and grab frames
for i=2:length(Data.vecteur_Robot)
    pause(0.01)
    frame = getframe(gcf); %get frame
    writeVideo(myVideo2, frame);
    set(robPoints,'XData',Data.vecteur_Robot(1,i));
    set(robPoints,'YData',Data.vecteur_Robot(2,i));
    if (Data.vecteurFlagConvergence(i))
    set(estRobPoints,'XData',Data.vecteur_estimation(1,i));
    set(estRobPoints,'YData',Data.vecteur_estimation(2,i));
    end
    title("estimated robot simulation during a mission")
end
close(myVideo2)
