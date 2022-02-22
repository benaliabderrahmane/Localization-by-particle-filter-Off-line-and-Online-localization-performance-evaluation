function [Robot_fin,u,dt] = controle(Robot,PP,v,bruit)
global idx_seg; global test_orientation; 

dot_theta = 0.5; % vitesse de rotation 

delta_y = PP(2, idx_seg+1) - PP(2, idx_seg); 
delta_x = PP(1, idx_seg+1) - PP(1, idx_seg);  
distance = norm(PP(1:2,idx_seg+1)-PP(1:2,idx_seg)); % calcule du distance entre deux points de passages 
theta_new = atan2((delta_y/distance),(delta_x/distance)); % calcule la nouvelle angle d'orientation 
if ((theta_new -Robot.theta) ~= 0)
    test_orientation = 1 ;
end

if (test_orientation == 1)
        u = 0; % mettre la vitesse linéaire  a 0
        theta_orientation = AngleWrap(theta_new-Robot.theta); % calcule l'angle de rotation restant pour que le robot atteigne l'orientation désirée 
                                                              %   la fonction AngleWrap : pour bornée l'angle entre -pi et pi avec
        %% calcule la vitesse de rotation dt                                                   
        if(sign(theta_orientation) == -1)  % si theta_orientation est négative, le robot tourne dans la sens négatif 
            dt=max(-dot_theta,theta_orientation*10); %  pour éviter toute erreur d'orientation 

        else 
            dt=min(dot_theta,theta_orientation*10);
        end
        %% calcule la nouvelle pose du robot 
        Robot_fin = Run_Robot( Robot, u, dt, bruit);
        Robot_fin.theta=AngleWrap(Robot_fin.theta);
        if( abs(theta_new-Robot_fin.theta)<(10^(-2))) % si Robot.theta = l'orientation désirée on passe à la translation
            test_orientation = 0;
        end

else
    dt = 0; % mettre la vitesse angulaire  a 0
        
        
        %% calcule la distance entre le robot et le prochain point de passage
        dist= sqrt((Robot.x-PP(1, idx_seg+1))^2+(Robot.y-PP(2,idx_seg+1))^2);
        u =min( v(idx_seg),dist*10); % calcul u 
        u = 0.4;
        %end
        %% calcule la neuvelle pose du robot
      
        Robot_fin = Run_Robot( Robot, u, dt, bruit);
        dist_Rob_finSEG=sqrt((Robot_fin.x-PP(1, idx_seg+1))^2+(Robot_fin.y-PP(2,idx_seg+1))^2); % calcule de neuvelle distance entre le robot et PP 
                                                                                                % pour faire le test 
        
        if(dist_Rob_finSEG<(10^(-8)))
            test_orientation = 1; % si Robot rejoindre la fin de segement on passe à la rotation
            idx_seg=idx_seg+1; % pour le prouchaine calcule on va utiliser des neuveau point de passage 
        end
end
end