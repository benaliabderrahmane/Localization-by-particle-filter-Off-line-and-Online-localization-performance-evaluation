function [rho,ximp,yimp,GrandObstacle]=Mesure_act(Choice,Portee,xROB,yROB,thetaROB,theta,Obstacles,ObstaclesMobiles,Bruitage,CoefTexture_Wall)

    Choice = upper(Choice);
    switch Choice
        case "LASER"
            theta=round(theta*(1024/(2*pi)))*(2*pi/1024);
            ObstaclesComplet=[Obstacles ObstaclesMobiles];
            if isempty(ObstaclesComplet)
                rho=inf*ones(length(theta),1);
                ximp=inf*ones(1,length(theta));
                yimp=inf*ones(1,length(theta));
                GrandObstacle=[];
            else
   
                [rho,ximp,yimp,GrandObstacle]=USPATCH_act(Portee,theta,ObstaclesComplet,xROB,yROB,thetaROB,Bruitage,CoefTexture_Wall);  
            end
            rho=min(rho,Portee);
            
        case "LASER FRONT"
            % we only use the sensor on the front but we keep the same
            % number of rayes 
            theta=linspace(-120*pi/180,120*pi/180,length(theta));
            ObstaclesComplet=[Obstacles ObstaclesMobiles];
            if isempty(ObstaclesComplet)
                rho=inf*ones(length(theta),1);
                ximp=inf*ones(1,length(theta));
                yimp=inf*ones(1,length(theta));
                GrandObstacle=[];
            else
   
                [rho,ximp,yimp,GrandObstacle]=USPATCH_act(Portee,theta,ObstaclesComplet,xROB,yROB,thetaROB,Bruitage,CoefTexture_Wall);  
            end
            rho=min(rho,Portee);
            
        case "US"
            theta=round(theta*(1024/(2*pi)))*(2*pi/1024);
            ObstaclesComplet=[Obstacles ObstaclesMobiles];
            if isempty(ObstaclesComplet)
                rho = inf*ones(length(theta),1);
                ximp=inf*ones(1,length(theta));
                yimp=inf*ones(1,length(theta));
                GrandObstacle=[];
            else
                RoboPose = [xROB, yROB, theta];
                Active = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
                Noise = Bruitage;
                Mesurement_segmentation =  5;  %Nombre de rayon tirer par chaque US pour simuler la dispertion du capteur.
                [Result,ximp,yimp ] = Mesures_US( RoboPose, Obstacles, Active, Noise, Mesurement_segmentation, CoefTexture_Wall );
                rho = Result;
            end
            rho=min(rho,Portee);
            
        case "US FRONT"
            theta=round(theta*(1024/(2*pi)))*(2*pi/1024);
            ObstaclesComplet=[Obstacles ObstaclesMobiles];
            if isempty(ObstaclesComplet)
                rho = inf*ones(length(theta),1);
                ximp=inf*ones(1,length(theta));
                yimp=inf*ones(1,length(theta));
                GrandObstacle=[];
            else
                RoboPose = [xROB, yROB, theta];
                Active = [1 1 1 1 1 1 1 1 0 0 0 0 0 0 0];
                Noise = Bruitage;
                Mesurement_segmentation =  5;  %Nombre de rayon tirer par chaque US pour simuler la dispertion du capteur.
                [Result,ximp,yimp ] = Mesures_US( RoboPose, Obstacles, Active, Noise, Mesurement_segmentation, CoefTexture_Wall );
                rho = Result;
            end
            rho=min(rho,Portee);
            
        case "US MIX"
            theta=round(theta*(1024/(2*pi)))*(2*pi/1024);
            ObstaclesComplet=[Obstacles ObstaclesMobiles];
            if isempty(ObstaclesComplet)
                rho = inf*ones(length(theta),1);
                ximp=inf*ones(1,length(theta));
                yimp=inf*ones(1,length(theta));
                GrandObstacle=[];
            else
                RoboPose = [xROB, yROB, theta];
                Active = [1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0];
                Noise = Bruitage;
                Mesurement_segmentation =  5;  %Nombre de rayon tirer par chaque US pour simuler la dispertion du capteur.
                [Result,ximp,yimp ] = Mesures_US( RoboPose, Obstacles, Active, Noise, Mesurement_segmentation, CoefTexture_Wall );
                rho = Result;
            end
            rho=min(rho,Portee);
            
        otherwise
            error("sensor type is not supported!! add it in sensors(resampling)/Mesure_act.m")
    end
end