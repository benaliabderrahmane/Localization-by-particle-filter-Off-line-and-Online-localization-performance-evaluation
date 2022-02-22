function Data = ParticleFilter(Options)

    close all
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
    

    %start tracking time
    T_Debut=tic;
    

    global Indice_ % to be used in redistribution
    global FlagRedistribution % to be used in redistribution (may be used the same way as Indice_ check later!!!)
    global N ;% Nombre de particules
    global idx_seg ;% indice du segment courant
    global test_orientation; % pour tester si on doit faire une translation(==0) ou rotation(==1)
    global Robot;
    global Particles; %position des particules
    global PoseEstime;



    
    N=Options.NParticles;  
    idx_seg=1;
    test_orientation=0;
    %maxIteration = 10000;
    testMesure=0; % indice utilisee pour effectuer une mesure pour plusieurs iterations
    Portee=4; % portee des capteurs
    fin_trajectoire=0; % test pour verifier que le robot a termine sa trajectoire 
    ObstaclesMobiles=[];
    vecteurTemps_mesure = [];
    t_iteration = [];%NaN(1,maxIteration); % vecteur contient le temps de chaque iteration  
    N_Particles=[];%NaN(1,maxIteration);  % vecteur contient le nombre des particules de chaque iteration
    iteration=[];%NaN(1,maxIteration);
    vecteur_Robot=[];%NaN(3,maxIteration);
    vecteur_estimation=[];%NaN(3,maxIteration);
    vecteur_estimation2=[];%NaN(3,maxIteration);
    vecteur_erreur=[];%NaN(3,maxIteration);
    vecteur_particles=[];%NaN(3,maxIteration*N);
    vecteur_Poids = [];%NaN(maxIteration*N,3);
    vecteur_incertitude_x=[];%NaN(2,maxIteration);
    vecteur_incertitude_y=[];%NaN(2,maxIteration);
    vecteur_incertitude_theta=[];%NaN(2,maxIteration);
    vecteur_Tconvergence=[];
    vecteur_sdw=[];
    vecteurFlagConvergence=[];
    iteration_convergence =0;
    T_convergence = 0;
    vecteurFlagRedistribution = [];
    indiceControle=0; 
    FlagRedistribution = 0 ; 
    Indice_ = 1 ;
    Indice2_ = 0;




%% generation des trajectoire : calcule les points de passages du robot et la vitesse correspond a  chaque segment de la trajectoire   
    Start=Options.StartPoint; % point de dépat du robot 
    Robot.x=Start(1);
    Robot.y=Start(2);
    Robot.theta=0.25;
    
    

%% generation des particules dans l'environement  


    PP = squeeze(Options.PP);
    v = Options.MaxSpeed*ones(length(PP),1);
    clear Particles;
    Particles.x=Options.particles(1,:);
    Particles.y=Options.particles(2,:);
    Particles.theta=Options.particles(3,:);


    % la definition du nombre de capteurs et de leurs angles :
    theta=linspace(-pi,pi,Options.NR);  


 %% initialisation d'affichage : 
    if Options.plot
        figure(10)
        plot_Environement(Obstacles,10);%affichage de l'environnement
        robPoints = plot(Robot.x,Robot.y,'o');
        particPoints=plot(Particles.x,Particles.y,'.r');
        plot(PP(1,:),PP(2,:),'*k')
        hold on
        plot(PP(1,:),PP(2,:),'k')
    end
    


indiceControle=1; % indice pour effectuer ou non le controle
%% boucle du filtrage:
    i=0;
    
    while(fin_trajectoire == 0)
    %%while(fin_trajectoire == 0)%&& i~=75) use this to stop PF after 75
    %%iterations
    
        temps_debut_iteration=tic;
        testMesure=testMesure+1;
        i=i+1
        % initialisation des poids :
        Poids = ones(N,1)/N;
        
        if (indiceControle==1)

            % controle du robot :
            [Robot,u,dt] = controle(Robot,PP,v,0);
            % affichage du robot :
            if Options.plot
                set(robPoints,'XData',Robot.x);
                set(robPoints,'YData',Robot.y);
            end

            % controle des particules :
            for j=1:N
                if (idx_seg == (Options.NPP+2))
                    break
                end
                P.x=Particles.x(j);
                P.y=Particles.y(j);
                P.theta=Particles.theta(j);
                P=Run_Robot(P,u,dt,0);
                Particles(1).x(j)=P.x;
                Particles(1).y(j)=P.y;
                Particles(1).theta(j)=P.theta;
            end

            % affichage des particules :
            if Options.plot
                hold on 
                set(particPoints,'XData',Particles.x);
                set(particPoints,'YData',Particles.y);
            end
        else 
            disp("here no control")
        end


%% measurement step
        % prendre les mesures du robot et des particules :
        debutMesure=tic; % variable pour calculer le temps du mesure 

            % mesures du robot :
            rho_rob=Mesure_act(Options.SensorsType,Portee,Robot.x,Robot.y,Robot.theta,theta,Obstacles,ObstaclesMobiles,1,1);
            for k=1:N
                % mesures des particules :
                rho_particles=Mesure_act(Options.SensorsType,Portee,Particles.x(k),Particles.y(k),Particles.theta(k),theta,Obstacles,ObstaclesMobiles,0,1);
                
                %likelihood step
                Poids(k)=likelihood(Options.Likelihood,rho_rob,rho_particles);
            end
            Temps_mesure=toc(debutMesure); % temps pour chaque mesure 
            vecteurTemps_mesure = [vecteurTemps_mesure Temps_mesure];
%% selection step
            iNextGeneration = selection(Options.Selection,Poids,N);

%% check for convergence
            % calcule de l'ecart-type des particules :
            SdX=sqrt(var(Particles.x(iNextGeneration)));
            SdY=sqrt(var(Particles.y(iNextGeneration)));
            SdTheta=sqrt(var(Particles.theta(iNextGeneration)));

                % la psition estimee du robot = la moyenne des particules :
                %PoseEstime.x=mean(Particles.x(iNextGeneration));
                %PoseEstime.y=mean(Particles.y(iNextGeneration));
                %PoseEstime.theta=mean(Particles.theta(iNextGeneration));
                % la psition estimee du robot = la somme particules*poids

                PoseEstime.x = sum(Particles.x*Poids)/sum(Poids);
                PoseEstime.y = sum(Particles.y*Poids)/sum(Poids);
                PoseEstime.theta = AngleWrap(sum(Particles.theta*Poids)/sum(Poids));
                [~,index] = max(Poids);
                PoseEstime2.x = sum(Particles.x(index));
                PoseEstime2.y = sum(Particles.y(index));
                PoseEstime2.theta = AngleWrap(sum(Particles.theta(index)));
                %Validation criteria:
                SdwX=sqrt(sum(Poids'.*(Particles.x-mean(Particles.x)).^2)/((N-1)/N*sum(Poids)));
                SdwY=sqrt(sum(Poids'.*(Particles.y-mean(Particles.y)).^2)/((N-1)/N*sum(Poids)));
                SdwTheta=sqrt(sum(Poids'.*(Particles.theta-mean(Particles.theta)).^2)/((N-1)/N*sum(Poids)));
                %% if we converge we stop the simulations (only for testing)
                if (SdwX<2 && SdwY<2)
                   % si l'ecart-type soit inferieur Ã  certin seulle <=> les particules
                        % converge :
                    vecteurFlagConvergence = [vecteurFlagConvergence 1];
                    
                    %% the 2 next lines to stop PF after convergence 
%                     fin_trajectoire = 1;
%                     indiceControle = 0;
              else
                vecteurFlagConvergence = [vecteurFlagConvergence 0];  
                end
                %%
            
                % information sur l'erreur 
                Erreur.x=Robot.x-PoseEstime.x;
                Erreur.y=Robot.y-PoseEstime.y;
                Erreur.theta=AngleWrap(Robot.theta-PoseEstime.theta);
                vecteur_erreur=[vecteur_erreur,[Erreur.x;Erreur.y;Erreur.theta]];


                % information sur l'incertitude
                incertitude_x(1) = min(Robot.x-Particles.x(iNextGeneration));
                incertitude_x(2) = max(Robot.x-Particles.x(iNextGeneration));
                incertitude_y(1) = min(Robot.y-Particles.y(iNextGeneration));
                incertitude_y(2) = max(Robot.y-Particles.y(iNextGeneration));
                incertitude_theta(1) = min(AngleWrap(Robot.theta-Particles.theta(iNextGeneration)));
                incertitude_theta(2) = max(AngleWrap(Robot.theta-Particles.theta(iNextGeneration)));

                vecteur_incertitude_x = [vecteur_incertitude_x, incertitude_x'];
                vecteur_incertitude_y = [vecteur_incertitude_y, incertitude_y'];
                vecteur_incertitude_theta = [vecteur_incertitude_theta, incertitude_theta'];

                vecteur_estimation=[vecteur_estimation,[PoseEstime.x;PoseEstime.y;PoseEstime.theta]];
                vecteur_estimation2=[vecteur_estimation2,[PoseEstime2.x;PoseEstime2.y;PoseEstime2.theta]];
                
                vecteur_sdw=[vecteur_sdw,[SdwX;SdwY;SdwTheta]];


            
                
                % redistribution
                % si les particules convergent vers une position autre que la position du robot :
                % On redistribue les particules sur toute la carte 
                if ~exist('OldParticles','var')
                    OldParticles.x = [];
                    OldParticles.y = [];
                end
                if ~exist('OldRobot','var')
                    OldRobot.x = [];
                    OldRobot.y = [];
                end
                
                [Particles,OldParticles,OldRobot,vecteur_Tconvergence,Poids,Indice2_] = resampling(Particles,Options.Distribution,Obstacles,OldRobot,OldParticles,Options.NParticles,T_Debut,i,vecteur_Tconvergence,SdX,SdY,SdTheta,SdwX,SdwY,SdwTheta,Poids,Indice2_);
                 
  
            if  Indice_ == 1
                Particles=testInext(iNextGeneration,Particles,Obstacles);
            else 
                Indice_ = 1;
            end
            
            %% save data 


                
        indiceControle=1;
        drawnow;
        
        % si on arrive au dernier segment on finit le controle  
        if (idx_seg == (Options.NPP+2))
            fin_trajectoire = 1;
            indiceControle = 0;
        end
        
        temps_iteration=toc(temps_debut_iteration); % temps pour chaque iteration 
        vecteur_Poids = [vecteur_Poids;Poids];
        t_iteration = [t_iteration , temps_iteration];
        N_Particles = [N_Particles,N];
        iteration = [iteration , i]; 
        vecteur_Robot=[vecteur_Robot,[Robot.x;Robot.y;Robot.theta]];
        vecteur_particles=[vecteur_particles,[Particles.x;Particles.y;Particles.theta]];
        vecteurFlagRedistribution = [vecteurFlagRedistribution FlagRedistribution];
    end
    
    
    
    T_fin=toc(T_Debut); % temps du programme  
    %%logs
    Data.vecteurFlagConvergence = vecteurFlagConvergence;
    Data.desired_trajectory = PP;
    Data.vecteur_incertitude_x = vecteur_incertitude_x;
    Data.vecteur_incertitude_y = vecteur_incertitude_y;
    Data.vecteur_incertitude_theta = vecteur_incertitude_theta;
    Data.N_Particles = N_Particles;
    Data.t_iteration = t_iteration;
    Data.vecteur_Robot = vecteur_Robot;
    Data.vecteur_estimation = vecteur_estimation;
    Data.vecteur_estimation2 = vecteur_estimation2;
    Data.vecteur_sdw = vecteur_sdw;
    Data.iteration = iteration;
    Data.T_convergence = T_convergence;
    Data.iteration_convergence = iteration_convergence;
    Data.vecteur_erreur = vecteur_erreur;
    Data.T_fin = T_fin;
    Data.vecteur_particles = vecteur_particles;
    Data.vecteur_Poids=vecteur_Poids;
    Data.vecteurTemps_mesure = vecteurTemps_mesure;
    Data.vecteurFlagRedistribution = vecteurFlagRedistribution;
  end



