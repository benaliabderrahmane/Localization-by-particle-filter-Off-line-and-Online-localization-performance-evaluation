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
NPP = 20; 
start_x = [27.5; 28.0; 27.2; 28; 27.3;
    27.5; 27.2; 27.2; 27.4; 28];
start_y = [50; 45; 40; 35; 30;
    1.5; 5; 4; 10; 8];
trajectories = zeros(30,3,22);
for j = 1:3
    for i = 1:length(start_x)
        Start = [start_x(i); start_y(i); -pi/2];
        End = [0; 1; 0];
        PP = trajectoryGenerator(NPP, Obstacles, Start, End, 10, 1);
        trajectories(i+(j-1)*length(start_x),:,:) = PP;
        filename = ('trajectories.mat')
        save(filename, 'trajectories');
    end
end