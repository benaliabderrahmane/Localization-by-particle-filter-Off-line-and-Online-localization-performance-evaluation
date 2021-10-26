function [PP, v ] = trajectory_generator(N,Obstacles,Fig)
% la fonction trajectory_generator va générer le trajectoire du robot ent
% passant par des de points de passages.
% Nous divisons la carte en deux grands rectangles et dans chaque rectangle,
% nous allons générer la moitié (N/2) du point de passage. 

% entrees : - N         : nombre de point de passage 
%           - Obstacles : la map 
%         - - Fig       : figure pour afficher les points 

% sorties : - PP : cordonnees des points de passages 
%           - v : vitesse pour chaque segment de trajectoire  



debut=[27.5;53;0]; % point de dépat du robot 
fin=[0;1.15;0]; % point de la fin 
milieu =[27.5;1.15;0]; % point de milieu 


x1 =27.0; x2= 28.5;  % points fixes qui définit les abscisses x_max et x_min pour tout le rectangle créé dans la première partie de la map. 
y1= 0 ; y2= 1;% points fixes qui définit les ordonnées y_max et y_min pour tous les rectangle crée dans la deuxième partie 

PP = debut; % l'initialisation du vecteur PP qui contiendra les coordonnées des points de passage 

N1= floor(N/2); % N1: nombre de point de passage pour la première partie de la map (prend la valeur la plus proche inférieure à N/2) 
dy= (53 -2.30002)/N1; % dy : la langeur des regtangles quand va cree pour la premier partie 

N2= ceil(N/2); % N2 nombre de point de passage pour la première partie (prend la valeur la plus proche superieur à N/2)
dx= (26.5747-0)/N2; % dx : la langeur des regtangles quand va cree pour la deuxième partie

k=1; % indice des element du vecteur vitesse v

% création d'un vecteur GrandObstacle qui contiendra les coordonnées des segments d'obstacles (la map) 
Sobst=size(Obstacles);
GrandObstacle=[];
for i=1:Sobst(2)
    GrandObstacle=[GrandObstacle;Obstacles(i).Pos_vertex(:,1)  Obstacles(i).Pos_vertex(:,2)];
    GrandObstacle=[GrandObstacle; NaN NaN];
end




for i= 0:N1-1
    test_intersection = 1; % flag pour indiquer l'intersection de segment crée avec la map :
                          % 1 intersction 
                          % 0 pas d'intersction 
    % x_min, x_max, y_min et y_max sont les cordonnées des sommets du réctangle crée 
    x_min = x1;
    x_max = x2;
    y_min = 53-(i+1)*dy;
    y_max = 53-i*dy;
    
    P=[];
    
%     a=[x1 ;  53-i*dy]; 
%     b=[x2 ;  53-i*dy];
%     c=[x2 ;  53-(i+1)*dy];
%     d=[x1 ;  53-(i+1)*dy];
    
    
    while (test_intersection == 1)
        P=Particles_generator(x_min,x_max,y_min ,y_max,-pi/2,pi/2, 1,Obstacles); % créeation d'un point de passage dans le réctange 
        x= polyxpoly([PP(1,end) P(1)],[PP(2,end), P(2)],GrandObstacle(:,1),GrandObstacle(:,2)); % la vérification d'intersection 
        
        if(isempty(x))  % si y a d'intersection 
            distance = ( PP(1:2,end)- P(1:2));
            if( distance > (y_max-y_min)/2)
                v(k) = 0.45;
            else 
                v(k) = 0.3;
            end
            PP=[PP,P];  % ajout de ce point dans le vecteur PP
            test_intersection = 0; % mettre le flag à 0 pour sortir de while ( et creé d'autre point) 
            k=k+1;
        end
        
    end
    

end
v(k) = 0.3;
k=k+1;

PP=[PP,milieu]; % ajout du point de milieu
v(k) = 0.3;
k=k+1;
for i= 0:N2-1
    test_intersection = 1;
    x_min = x1-(i+1)*dx;
    x_max = x1-i*dx ;
    y_min = y2;
    y_max = y1;
    P=[];
    
%     a=[x1-i*dx       ;  y1]; 
%     b=[x1-i*dx       ;  y2];
%     c=[x1-(i+1)*dx   ;  y2];
%     d=[x1-(i+1)*dx   ;  y1];
    
     while (test_intersection == 1) 
        P=Particles_generator(x_min,x_max,y_min ,y_max,-pi/2,pi/2, 1,Obstacles);
        x= polyxpoly([PP(1,end) P(1)],[PP(2,end), P(2)],GrandObstacle(:,1),GrandObstacle(:,2));
        if(isempty(x))
            distance = ( PP(1:2,end)- P(1:2));
            if( distance > (x_max-x_min)/2)
                v(k) = 0.45;
            else 
                v(k) = 0.3;
            end
            PP=[PP,P];
            test_intersection = 0;
            k=k+1;
        end

     end
end

PP=[PP,fin]; % ajout du pint de fin 

% affichage 
PP = PP(1:2,:);
figure(Fig)
plot(PP(1,:),PP(2,:),'*k')
plot(PP(1,:),PP(2,:),'k')

end



