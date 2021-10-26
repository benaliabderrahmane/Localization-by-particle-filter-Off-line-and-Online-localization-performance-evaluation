function [rho,ximp,yimp,GrandObstacle]=Mesure_act(Choice,Portee,xROB,yROB,thetaROB,theta,Obstacles,ObstaclesMobiles,Bruitage,CoefTexture_Wall)

    Choice = upper(Choice);
    switch Choice 
        case "LIDAR"
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
        otherwise
            error("sensor type is not supported!! add it in sensors(resampling)/Mesure_act.m")