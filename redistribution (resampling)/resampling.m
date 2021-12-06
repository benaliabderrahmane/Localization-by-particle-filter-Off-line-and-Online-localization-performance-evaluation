function [Particles,OldParticles,OldRobot,vecteurTconvergence,vecteurItConvergence] = resampling(Particles,Choice,Obstacles,OldRobot,OldParticles,Nmax,TimeDebut,iteration,vecteurTconvergence,vecteurItConvergence,SdX,SdY,SdTheta)
%RESAMPLING is the redistribution function 
%   INPUT:
%       
%   OUTPUT:
%       paritcles: new particles after redistribution
%       vecteurTconvergence and vecteurItConvergence are used to track time
%           number of iterations necessary for convergance.
%       FlagRedistribution to check if it's the first iteration of convergence
%       N: number of particles (in case we change the number of particles when we redistribute)
    global Indice_
    global FlagRedistribution
    %global Particles
    global Robot
    global N
    global PoseEstime
    
    if FlagRedistribution==0 %si on est dans la premiere iteration de convergence
        T_convergence=toc(TimeDebut);
        vecteurTconvergence=[vecteurTconvergence,T_convergence];
        iteration_convergence = iteration; 
        vecteurItConvergence=[vecteurItConvergence,iteration_convergence];
        OldParticles.x=PoseEstime.x;
        OldParticles.y=PoseEstime.y;
        OldRobot.x=Robot.x;
        OldRobot.y=Robot.y;
        FlagRedistribution=1;
    else
        Choice = upper(Choice);
        switch Choice 
        case "DISTANCE"
            % calcule de la distance parcourue pour les particules :
            Distance_Particles=norm([PoseEstime.x,PoseEstime.y]-[OldParticles.x,OldParticles.y]);

            % calcule de la distance parcourue pour le robot :
            Distance_robot=norm([Robot.x,Robot.y]-[OldRobot.x,OldRobot.y]);

            % s'il y a une difference de  2.5 m, on effectue la redistribution : 
            if(abs(Distance_Particles-Distance_robot)>1)
                N=Nmax;
                particles1=Particles_generator(26.5747,29.02,-0.269984,55,-pi,pi,floor(N/2),Obstacles);
                particles2=Particles_generator(-5,26.5747,-0.269984,11.53,-pi,pi,N-floor(N/2),Obstacles);
                particles=[particles1,particles2];
                OldParticles.x = [];
                OldParticles.y = [];
                OldRobot.x = [];
                OldRobot.y = [];
                Particles.x=particles(1,:);
                Particles.y=particles(2,:);
                Particles.theta=particles(3,:);
                Indice_ = 0;
                FlagRedistribution=0;
            end
        case "STANDARD DEVIATION"
            if (SdX<0.5 && SdY<0.5 && SdTheta<0.2)
                N=Nmax;
                particles1=Particles_generator(26.5747,29.02,-0.269984,55,-pi,pi,floor(N/2),Obstacles);
                particles2=Particles_generator(-5,26.5747,-0.269984,11.53,-pi,pi,N-floor(N/2),Obstacles);
                particles=[particles1,particles2];
                Particles.x=particles(1,:);
                Particles.y=particles(2,:);
                Particles.theta=particles(3,:);
                Indice_ = 0;
                FlagRedistribution=0;
            end 
        otherwise
            error("redistribution function is not defined!! add it in redistribution (resampling)/resampling.m")
        end
    end
end


