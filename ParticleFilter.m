function ParticleFilter()

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

    
    global N ;% Nombre de particules
    global idx_seg ;% indice du segment courant
    global test_orientation; % pour tester si on doit faire une translation(==0) ou rotation(==1)
    global Robot;
    global Particles; %position des particules
    global pose_estime;



    Nmax=10;
    N=Nmax;  
    idx_seg=1;
    test_orientation=0;
    Robot.x=27.5;
    Robot.y=53;
    Robot.theta=-pi/2;

    N_PP=3; % nombre de point de passage
    test_mesure=0; % indice utilisee pour effectuer une mesure pour plusieurs iterations
    Portee=4; % portee des capteurs
    fin_trajectoire=0; % test pour verifier que le robot a termine sa trajectoire 
    ObstaclesMobiles=[];

    t_iteration =[]; % vecteur contient le temps de chaque iteration  
    N_Particles=[];  % vecteur contient le nombre des particules de chaque iteration
    iteration=[];
    vecteur_Robot=[];
    vecteur_estimation=[];
    vecteur_erreur=[];
    vecteur_particles=[];

    vecteur_incertitude_x=[];
    vecteur_incertitude_y=[];
    vecteur_incertitude_theta=[];
    vecteur_Tconvergence=[];
    vecteur_It_convergence=[];
    iteration_convergence =0;
    T_convergence = 0;

    indice_controle=0; % indice pour effectuer ou non le controle
    FlagRedistribution = 0 ; 
    Indice_ = 1 ;





%% generation des trajectoire : calcule les points de passages du robot et la vitesse correspond a  chaque segment de la trajectoire   

    [PP,v] = trajectory_generator(N_PP,Obstacles,10);
%     PP=[27.5000000000000,27.2033898732974,27.4721696236095,27.7489383369825,27.6752137586831,27.4764283019429,28.0636474780856,27.4185388602376,28.4203991460876,27.0112195666672,27.5620342609094,27.5000000000000,25.8037922466977,23.5753521996448,20.4411248600483,17.0655134940915,15.0895883044919,11.8792429638910,10.3656653357397,5.86374377586899,4.02232296127434,1.85700612138352,0;53,49.7078469245938,47.9278001232452,38.1570425584487,34.6003872218339,30.9832740293548,23.6865946109579,19.1039582063765,12.6857209440967,12.1157472361984,4.37283996378318,1.15000000000000,0.476222049378097,1.88322196830137,2.60801742786320,2.01033699354604,1.69220006403437,1.59250030623232,0.852370684719879,1.81769272353208,1.06495249855317,2.18626599228018,1.15000000000000];
%     v=[0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.90,0.900,0.900];




%% generation des particules dans l'environement  

    %generetion des particles autour du robot :
    %particles=Particles_generator(26.5747,29.02,50,54,-pi,pi,N,Obstacles); 


    %generation des particules dans toute la map :
    particles1=Particles_generator(26.5747,29.02,-0.269984,56,-pi,pi,N/2,Obstacles);
    particles2=Particles_generator(-5,26.5747,-0.269984,11.53,-pi,pi,N/2,Obstacles);
    particles=[particles1,particles2];
    clear Particles
    Particles.x=particles(1,:);
    Particles.y=particles(2,:);
    Particles.theta=particles(3,:);


    % la definition du nombre de capteurs et de leurs angles :
    theta=linspace(-pi,pi,16);
    theta(end)=[];

 %% initialisation d'affichage : 
    figure(10)
    plot_Environement(Obstacles,10);%affichage de l'environnement
    robPoints = plot(Robot.x,Robot.y,'o');
    particPoints=plot(Particles.x,Particles.y,'.r');



    
%% boucle du filtrage:
    i=0;
    while(fin_trajectoire == 0)
        temps_debut_iteration=tic;
        test_mesure=test_mesure+1;
        i=i+1;

        % initialisation des poids :
        Poids = ones(N,1)/N;

        if (indice_controle==1)

            % controle du robot :
            Robot = controle(Robot,PP,v,0);
            % affichage du robot :
            set(robPoints,'XData',Robot.x);
            set(robPoints,'YData',Robot.y);

            % controle des particules :
            for j=1:N
                P.x=Particles.x(j);
                P.y=Particles.y(j);
                P.theta=Particles.theta(j);
                P=controle(P,PP,v,0);
                Particles.x(j)=P.x;
                Particles.y(j)=P.y;
                Particles.theta(j)=P.theta;
            end

            % affichage des particules :
            hold on 
            set(particPoints,'XData',Particles.x);
            set(particPoints,'YData',Particles.y);

        end


%% measurement step
        % prendre les mesures du robot et des particules :
        Debut_mesure=tic; % variable pour calculer le temps du mesure 

        if(test_mesure==3) 
            % mesures du robot :
            rho_rob=Mesure_act(Portee,Robot.x,Robot.y,Robot.theta,theta,Obstacles,ObstaclesMobiles,1,1);
            rho_particles=[];
            for k=1:N
                % mesures des particules :
                rho_particles=Mesure_act(Portee,Particles.x(k),Particles.y(k),Particles.theta(k),theta,Obstacles,ObstaclesMobiles,0,1);
                
                %likelihood step
                Poids(k)=likelihood(rho_rob,rho_particles);
            end
            Temps_mesure=toc(Debut_mesure); % temps pour chaque mesure 
%% selection step
            iNextGeneration = selection(Poids,N);

%% check for convergance
            % calcule de l'ecart-type des particules :
            Et_x=sqrt(var(Particles.x(iNextGeneration)));
            Et_y=sqrt(var(Particles.y(iNextGeneration)));
            Et_theta=sqrt(var(Particles.theta(iNextGeneration)));

            % si l'ecart-type soit inferieur Ã  certin seulle <=> les particules
            % converge :
            if(Et_x<7 && Et_y<7 && Et_theta<2)
                % la psition estimee du robot = la moyenne des particules :
                pose_estime.x=mean(Particles.x(iNextGeneration));
                pose_estime.y=mean(Particles.y(iNextGeneration));
                pose_estime.theta=mean(Particles.theta(iNextGeneration));

                % information sur l'erreur 
                Erreur.x=Robot.x-pose_estime.x;
                Erreur.y=Robot.y-pose_estime.y;
                Erreur.theta=Robot.theta-pose_estime.theta;
                vecteur_erreur=[vecteur_erreur,[Erreur.x;Erreur.y;Erreur.theta]];


                % information sur l'incertitude
                incertitude_x(1) = min(Particles.x(iNextGeneration));
                incertitude_x(2) = max(Particles.x(iNextGeneration));
                incertitude_y(1) = min(Particles.y(iNextGeneration));
                incertitude_y(2) = max(Particles.y(iNextGeneration));
                incertitude_theta(1) = min(Particles.theta(iNextGeneration));
                incertitude_theta(2) = max(Particles.theta(iNextGeneration));

                vecteur_incertitude_x = [vecteur_incertitude_x, incertitude_x'];
                vecteur_incertitude_y = [vecteur_incertitude_y, incertitude_y'];
                vecteur_incertitude_theta = [vecteur_incertitude_theta, incertitude_theta'];

                vecteur_estimation=[vecteur_estimation,[pose_estime.x;pose_estime.y;pose_estime.theta]];



%% redistribution
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
                [Particles,OldParticles,OldRobot,vecteur_Tconvergence,vecteur_It_convergence,FlagRedistribution,Indice_,N] = resampling(Obstacles,Robot,OldRobot,pose_estime,Particles,OldParticles,Nmax,T_Debut,i,FlagRedistribution,vecteur_Tconvergence,vecteur_It_convergence,Indice_,N);
            end

            if  Indice_ ~= 0 
                Particles=testInext(iNextGeneration,Particles,Obstacles);
            else 
                Indice_ = 1;
            end
            test_mesure=0;
        end

        indice_controle=1;
        drawnow;
        % si on arrive au dernier segment on finit le controle  
        
        if (idx_seg== (N_PP+2))
            fin_trajectoire= 1 ;
        end
        temps_iteration=toc(temps_debut_iteration); % temps pour chaque iteration 

%% save data 
        t_iteration = [t_iteration , temps_iteration];
        N_Particles = [N_Particles,N];
        iteration = [iteration , i]; 

        vecteur_Robot=[vecteur_Robot,[Robot.x;Robot.y;Robot.theta]];
        vecteur_particles=[vecteur_particles,[inf;inf;inf],[Particles.x;Particles.y;Particles.theta]];

    end
    T_fin=toc(T_Debut); % temps du programme 


