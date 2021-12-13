addpath('./affichage/');
addpath('./data/');
addpath('./likelihood/');
addpath('./redistribution (resampling)/');
addpath('./robot/');
addpath('./selection/');
addpath('./sensors/');
addpath('./utilities/');
    
%% initialization of parameters 
%load map data
load('bat5_Obstacles_detect_redone140220.mat');
plot_Environement(Obstacles,10);

%% save trajectories
NPP = 1; 
startX = [7 12 15 22 27.5 19 16 9 27];
startY = [0.5 0.2 0.3 1.2 1.3 1.8 1.7 0.7 0.8];
endX = [12 15 20 28 18 14 11 3 15];
endY = [1.2 0.6 1.9 0.8 1.7 1.2 0.4 0.8 0.9];

trajectories = zeros(2*length(startX),3,NPP+2);
for j = 1:2
    for i = 1:length(startX)
        scatter(startX(i),startY(i))
        Start = [startX(i); startY(i); pi];
        End = [endX(i); endY(i); pi];
        PP = trajectoryGenerator(NPP, Obstacles, Start, End, 10, 1);
        trajectories(i+(j-1)*length(startX),:,:) = PP;
        pause(1)
    end
end
filename = ('trajectories.mat')
save(filename, 'trajectories');