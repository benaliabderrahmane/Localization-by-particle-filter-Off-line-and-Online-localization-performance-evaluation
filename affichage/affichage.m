load('data.mat');
%% 500 particules 

iteration_convergence = 25;
figure(11)
subplot(311); plot(iteration(iteration_convergence:end),pose_estime.x,'r');
hold on 
%y1 = (vecteur_incertitude.x.min+vecteur_incertitude.x.min)/2;
%err1 = vecteur_incertitude.x.max-vecteur_incertitude.x.min;
%errorbar(iteration(iteration_convergence:end),y1,err1,'|y')
legend('x estime','incertitude')
xlabel('iteration');
subplot(312); plot(iteration(iteration_convergence:end),pose_estime.y,'r')
%hold on 
%errorbar(iteration(iteration_convergence:end), (vecteur_incertitude.y.min+vecteur_incertitude.y.min)/2, vecteur_incertitude.y.max-vecteur_incertitude.y.min,'|y')
legend('y estime','incertitude')
xlabel('iteration');


subplot(313); plot(iteration(iteration_convergence:end),pose_estime.theta,'r')
%hold on 
%errorbar(iteration(iteration_convergence:end),(vecteur_incertitude.theta.min+vecteur_incertitude.theta.min)/2,vecteur_incertitude.theta.max-vecteur_incertitude.theta.min,'|y')
legend('theta estime','incertitude')
xlabel('iteration');
title('les estimqtions et les incertitudes en fonction d iteration')
%%
figure(22)

subplot(211); plot(iteration,N_Particles)
xlabel('iteration'); ylabel('Nombre de particules')
title('Evolution de nombre de particules et de temps de calculs avec les iterations')
subplot(212); plot(iteration,t_iteration)
xlabel('iteration'); ylabel('Temps a chaque iteration')
% 
figure(33)

subplot(311); plot(iteration(iteration_convergence:end), pose_estime.x,'r');
hold on 
plot(iteration,vecteur_Robot.x,'b');
legend('x estime','x Robot')
xlabel('iteration');


subplot(312); plot(iteration(iteration_convergence:end),pose_estime.y,'r')
hold on 
plot(iteration, vecteur_Robot.y,'b');
legend('y estime','y Robot')
xlabel('iteration');


subplot(313); plot(iteration(iteration_convergence:end),pose_estime.theta,'r')
hold on 
plot(iteration, vecteur_Robot.theta,'b');

legend('theta estime','Theta Robot')
xlabel('iteration');
title('')






save('data.mat','vecteur_estimation','vecteur_Robot','vecteur_erreur','t_iteration','N_Particles','iteration','vecteur_incertitude')
