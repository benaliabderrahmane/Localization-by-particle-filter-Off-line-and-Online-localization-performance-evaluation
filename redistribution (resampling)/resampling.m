function [particles,vecteurTconvergence,vecteurItConvergence,flag,Indice_,N] = resampling(Robot,PoseEstime,Nmax,TimeDebut,iteration,flag,vecteurTconvergence,vecteurItConvergence)
%RESAMPLING is the redistribution function 
%   INPUT:
%       
%   OUTPUT:
%       paritcles: new particles after redistribution
%       vecteurTconvergence and vecteurItConvergence are used to track time
%           number of iterations necessary for convergance.
%       flag to check if it's the first iteration of convergence
%       N: number of particles (in case we change the number of particles when we redistribute)

    if flag==0 %si on est dans la premiere iteration de convergence
        T_convergence=toc(TimeDebut);
        vecteurTconvergence=[vecteurTconvergence,T_convergence];
        iteration_convergence = iteration; 
        vecteurItConvergence=[vecteurItConvergence,iteration_convergence];
        Particles_old.x=PoseEstime.x;
        Particles_old.y=PoseEstime.y;
        robot_old.x=Robot.x;
        robot_old.y=Robot.y;
        flag=1;
    else
        % calcule de la distance parcourue pour les particules :
        Distance_Particles=norm([PoseEstime.x,PoseEstime.y]-[Particles_old.x,Particles_old.y]);

        % calcule de la distance parcourue pour le robot :
        Distance_robot=norm([Robot.x,Robot.y]-[robot_old.x,robot_old.y]);

        % s'il y a une difference de  2.5 m, on effectue la redistribution : 
        if(abs(Distance_Particles-Distance_robot)>2.5)
            N=Nmax;
            particles1=Particles_generator(26.5747,29.02,-0.269984,55,-pi,pi,N/2,Obstacles);
            particles2=Particles_generator(-5,26.5747,-0.269984,11.53,-pi,pi,N/2,Obstacles);
            particles=[particles1,particles2];
            Particles.x=particles(1,:);
            Particles.y=particles(2,:);
            Particles.theta=particles(3,:);
            Indice_ = 0;
            flag=0;
        end  
    end
end

