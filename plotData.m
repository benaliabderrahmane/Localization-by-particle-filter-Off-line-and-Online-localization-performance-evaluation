figure(10)
clc
close all
addpath('./affichage/');
addpath('./data/');
addpath('./likelihood/');
addpath('./redistribution (resampling)/');
addpath('./robot/');
addpath('./selection/');
addpath('./sensors/');
addpath('./utilities/');
    
%load('C:\Users\benal\Dropbox\PC\Documents\projet master\Bibliographie\Code\-Localization-by-particle-filter--Off-line-and-Online-localization-performance-evaluation\data\bat5_Obstacles_detect_redone140220.mat')
plot_Environement(Obstacles,10);%affichage de l'environnement
n = length(Data.vecteur_Robot);
Robot = Data.vecteur_Robot(:,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initialize Particles vector
flag = 1;
k=2;
while (flag == 1)
    if (Data.vecteur_particles(1,k)~=inf)
        k =k+1;
    else 
       flag = 0;
    end
end
Particles = Data.vecteur_particles(:,2:k-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robPoints=plot(Robot(1),Robot(2),'o');
particPoints=plot(Particles(1,:),Particles(2,:),'.r');
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'*k')
plot(Data.desired_trajectory(1,:),Data.desired_trajectory(2,:),'k')

%%
for i=2:n
    Robot = Data.vecteur_Robot(:,i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %update Particles vector
    flag = 1;
    m=k+1;
    k=k+1;
    while (flag == 1 && k<=length(Data.vecteur_particles))
        if (Data.vecteur_particles(1,k)~=inf)
            k =k+1;
        else 
           flag = 0;
        end
    end
    Particles = Data.vecteur_particles(:,m:k-1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(robPoints,'XData',Robot(1));
    set(robPoints,'YData',Robot(2));    
    set(particPoints,'XData',Particles(1,:));
    set(particPoints,'YData',Particles(2,:));
    pause(0.01)
    b = 1;
end

% clear mean
% %% load data
% load('likelihood1_Roulette_wheel_distance_50_laser_20_0_4_16_trajectory_number_1.mat');
% data1 = Data;
% load('likelihood1_Roulette_wheel_distance_50_laser_20_0_4_32_trajectory_number_1.mat');
% data2 = Data;
% load('likelihood1_Roulette_wheel_distance_50_laser_front_20_0_4_16_trajectory_number_1.mat');
% data3 = Data;
% load('likelihood1_Roulette_wheel_distance_50_laser_front_20_0_4_32_trajectory_number_1.mat');
% data4 = Data;
% load('likelihood1_Roulette_wheel_distance_50_US_20_0_4_16_trajectory_number_1.mat');
% data5 = Data;
% load('likelihood1_Roulette_wheel_distance_50_US_20_0_4_32_trajectory_number_1.mat');
% data6 = Data;
% load('likelihood1_Roulette_wheel_standard_Deviation_50_laser_20_0_4_16_trajectory_number_1.mat');
% data7 = Data;
% load('likelihood1_Roulette_wheel_standard_Deviation_50_laser_front_20_0_4_16_trajectory_number_1.mat');
% data8 = Data;
% load('likelihood1_Roulette_wheel_standard_Deviation_50_US_20_0_4_16_trajectory_number_1.mat');
% data9 = Data;
% load('likelihood1_Stochastic_universel_sampling_distance_50_laser_20_0_4_16_trajectory_number_1.mat');
% data10 = Data;
% load('likelihood1_Stochastic_universel_sampling_distance_50_laser_front_20_0_4_16_trajectory_number_1.mat');
% data11 = Data;
% load('likelihood1_Stochastic_universel_sampling_distance_50_US_20_0_4_16_trajectory_number_1.mat');
% data12 = Data;
% load('likelihood1_Stochastic_universel_sampling_standard_Deviation_50_laser_20_0_4_16_trajectory_number_1.mat');
% data13 = Data;
% load('likelihood1_Stochastic_universel_sampling_standard_Deviation_50_laser_front_20_0_4_16_trajectory_number_1.mat');
% data14 = Data;
% load('likelihood1_Stochastic_universel_sampling_standard_Deviation_50_US_20_0_4_16_trajectory_number_1.mat');
% data15 = Data;
% 
% %% plot
% mx1 = [ ];my1 = [ ];mt1 = [ ];
% mx3 = [ ];my3 = [ ];mt3 = [ ];
% mx5 = [ ];my5 = [ ];mt5 = [ ];
% mx10 = [ ];my10 = [ ];mt10 = [ ];
% j = 1;
% for i = 0:50:(length(data1.vecteur_particles)-4*8128)  
%     X1 = data1.vecteur_particles(1,i+1:i+50);
%     Y1 = data1.vecteur_particles(2,i+1:i+50);
%     T1 = data1.vecteur_particles(3,i+1:i+50);
%     X3 = data3.vecteur_particles(1,i+1:i+50);
%     Y3 = data3.vecteur_particles(2,i+1:i+50);
%     T3 = data3.vecteur_particles(3,i+1:i+50);
%     X5 = data5.vecteur_particles(1,i+1:i+50);
%     Y5 = data5.vecteur_particles(2,i+1:i+50);
%     T5 = data5.vecteur_particles(3,i+1:i+50);
%     X10 = data10.vecteur_particles(1,i+1:i+50);
%     Y10 = data10.vecteur_particles(2,i+1:i+50);
%     T10 = data10.vecteur_particles(3,i+1:i+50);
%     %convert inf to NaN
%     X1(find(X1==Inf))=NaN; Y1(find(Y1==Inf))=NaN; T1(find(T1==Inf))=NaN;
%     X3(find(X3==Inf))=NaN; Y3(find(Y3==Inf))=NaN; T3(find(T3==Inf))=NaN;
%     X5(find(X5==Inf))=NaN; Y5(find(Y5==Inf))=NaN; T5(find(T5==Inf))=NaN;
%     X10(find(X10==Inf))=NaN; Y10(find(Y10==Inf))=NaN; T10(find(T10==Inf))=NaN;
%     %compute mean ignoring NaN
%     mx1(j) = nanmean(X1); my1(j) = nanmean(Y1); mt1(j) = nanmean(T1);
%     mx3(j) = nanmean(X3); my3(j) = nanmean(Y3); mt3(j) = nanmean(T3);
%     mx5(j) = nanmean(X5); my5(j) = nanmean(Y5); mt5(j) = nanmean(T5);
%     mx10(j) = nanmean(X10); my10(j) = nanmean(Y10); mt10(j) = nanmean(T10);
%     j = j+1;
% end
% 
% robotx1 = [27.5 data1.vecteur_Robot(1,:)];roboty1 = [50    data1.vecteur_Robot(2,:)];robott1 = [-pi/2 data1.vecteur_Robot(3,:)];
% robotx3 = [27.5 data3.vecteur_Robot(1,:)];roboty3 = [50    data3.vecteur_Robot(2,:)];robott3 = [-pi/2 data3.vecteur_Robot(3,:)];
% robotx5 = [27.5 data5.vecteur_Robot(1,:)];roboty5 = [50    data5.vecteur_Robot(2,:)];robott5 = [-pi/2 data5.vecteur_Robot(3,:)];
% robotx10 = [27.5 data10.vecteur_Robot(1,:)];roboty10 = [50    data10.vecteur_Robot(2,:)];robott10 = [-pi/2 data10.vecteur_Robot(3,:)];
% figure(1)
% subplot(3,3,1)
% plot(abs(mx1(1:100)-robotx1(1:100)))
% subplot(3,3,4)
% plot(abs(my1(1:100)-roboty1(1:100)))
% subplot(3,3,7)
% plot(abs(mt1(1:100)-robott1(1:100)))
% subplot(3,3,2)
% plot(abs(mx3(1:100)-robotx3(1:100)))
% subplot(3,3,5)
% plot(abs(my3(1:100)-roboty3(1:100)))
% subplot(3,3,8)
% plot(abs(mt3(1:100)-robott3(1:100)))
% subplot(3,3,3)
% plot(abs(mx10(1:100)-robotx10(1:100)))
% subplot(3,3,6)
% plot(abs(my10(1:100)-roboty10(1:100)))
% subplot(3,3,9)
% plot(abs(mt10(1:100)-robott10(1:100)))
% 
% 
% figure(2)
% subplot(3,2,1)
% plot(abs(mx1(1:100)-robotx1(1:100)))
% subplot(3,2,3)
% plot(abs(my1(1:100)-roboty1(1:100)))
% subplot(3,2,5)
% plot(abs(mt1(1:100)-robott1(1:100)))
% subplot(3,2,2)
% plot(abs(mx10(1:100)-robotx10(1:100)))
% subplot(3,2,4)
% plot(abs(my10(1:100)-roboty10(1:100)))
% subplot(3,2,6)
% plot(abs(mt10(1:100)-robott10(1:100)))

% iteration_convergence = 25;
% figure(11)
% subplot(311); plot(iteration(iteration_convergence:end),pose_estime.x,'r');
% hold on 
% %y1 = (vecteur_incertitude.x.min+vecteur_incertitude.x.min)/2;
% %err1 = vecteur_incertitude.x.max-vecteur_incertitude.x.min;
% %errorbar(iteration(iteration_convergence:end),y1,err1,'|y')
% legend('x estime','incertitude')
% xlabel('iteration');
% subplot(312); plot(iteration(iteration_convergence:end),pose_estime.y,'r')
% %hold on 
% %errorbar(iteration(iteration_convergence:end), (vecteur_incertitude.y.min+vecteur_incertitude.y.min)/2, vecteur_incertitude.y.max-vecteur_incertitude.y.min,'|y')
% legend('y estime','incertitude')
% xlabel('iteration');
% 
% 
% subplot(313); plot(iteration(iteration_convergence:end),pose_estime.theta,'r')
% %hold on 
% %errorbar(iteration(iteration_convergence:end),(vecteur_incertitude.theta.min+vecteur_incertitude.theta.min)/2,vecteur_incertitude.theta.max-vecteur_incertitude.theta.min,'|y')
% legend('theta estime','incertitude')
% xlabel('iteration');
% title('les estimqtions et les incertitudes en fonction d iteration')
% %%
% figure(22)
% 
% subplot(211); plot(iteration,N_Particles)
% xlabel('iteration'); ylabel('Nombre de particules')
% title('Evolution de nombre de particules et de temps de calculs avec les iterations')
% subplot(212); plot(iteration,t_iteration)
% xlabel('iteration'); ylabel('Temps a chaque iteration')
% % 
% figure(33)
% 
% subplot(311); plot(iteration(iteration_convergence:end), pose_estime.x,'r');
% hold on 
% plot(iteration,vecteur_Robot.x,'b');
% legend('x estime','x Robot')
% xlabel('iteration');
% 
% 
% subplot(312); plot(iteration(iteration_convergence:end),pose_estime.y,'r')
% hold on 
% plot(iteration, vecteur_Robot.y,'b');
% legend('y estime','y Robot')
% xlabel('iteration');
% 
% 
% subplot(313); plot(iteration(iteration_convergence:end),pose_estime.theta,'r')
% hold on 
% plot(iteration, vecteur_Robot.theta,'b');
% 
% legend('theta estime','Theta Robot')
% xlabel('iteration');
% title('')
% 
% 
% 
% 
% 
% 
% save('data.mat','vecteur_estimation','vecteur_Robot','vecteur_erreur','t_iteration','N_Particles','iteration','vecteur_incertitude')


     