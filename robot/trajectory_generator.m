function [PP, v ] = trajectory_generator(N,Obstacles,Fig,Start,End)
% la fonction trajectory_generator va générer le trajectoire du robot ent
% passant par des de points de passages.
% Nous divisons la carte en deux grands rectangles et dans chaque rectangle,
% nous allons générer la moitié (N/2) du point de passage. 

% entrees : - N         : nombre de point de passage 
%           - Obstacles : la map 
%         - - Fig       : figure pour afficher les points 

% sorties : - PP : cordonnees des points de passages 
%           - v : vitesse pour chaque segment de trajectoire  



x1 =27.0; x2= 28.5;  % points fixes qui définit les abscisses x_max et x_min pour tout le rectangle créé dans la première partie de la map. 
y1= 0 ; y2= 1;% points fixes qui définit les ordonnées y_max et y_min pour tous les rectangle crée dans la deuxième partie 

PP = zeros(3,N+2);
PP(:,1) = Start;
PP(:,end) = End;

v = zeros(N+2,1);

% création d'un vecteur GrandObstacle qui contiendra les coordonnées des segments d'obstacles (la map) 
Sobst=size(Obstacles);
GrandObstacle=zeros(Sobst(2)*3,2);

for i=1:Sobst(2)
    kk = (i-1)*3;
    GrandObstacle(kk+1:kk+2,:)=[Obstacles(i).Pos_vertex(:,1)  Obstacles(i).Pos_vertex(:,2)];
    GrandObstacle(kk+3,:)=[NaN NaN];
end

k=1; % indice des element du vecteur vitesse v

if Start(1)>=26 && End(1)>= 26
    %trajectory in first part of the map
    section = 1;
elseif Start(2)<5.5 && End(2)< 5.5
    %trajectory in second part of the map
    section = 2; 
else
    %trajectory in both part of the map
    section = 12;
end

if section == 1
    %trajectory in first part of the map
    
    
elseif section == 2
    %trajectory in second part of the map
    
    
else 
    %trajectory in both part of the map
    
    



for i= 0:N
    test_intersection = 1; % flag pour indiquer l'intersection de segment crée avec la map :
                          % 1 intersction 
                          % 0 pas d'intersction 

    x_min = x1;
    x_max = x2;
    y_min = 53-(i+1)*dy;
    y_max = 53-i*dy;
    

%     a=[x1 ;  53-i*dy]; 
%     b=[x2 ;  53-i*dy];
%     c=[x2 ;  53-(i+1)*dy];
%     d=[x1 ;  53-(i+1)*dy];
    
    
    while (test_intersection == 1)
        P=Particles_generator(x_min,x_max,y_min ,y_max,-pi/2,pi/2, 1,Obstacles); % créeation d'un point de passage dans le réctange 
        x= polyxpoly([PP(1,i+1) P(1)],[PP(2,i+1), P(2)],GrandObstacle(:,1),GrandObstacle(:,2)); % la vérification d'intersection 
        if(isempty(x))  % si y a d'intersection 
            distance = ( PP(1:2,end)- P(1:2));
            if( distance > (y_max-y_min)/2)
                v(k) = 0.45;
            else 
                v(k) = 0.3;
            end
            PP(:,i+2)=P;  % ajout de ce point dans le vecteur PP
            test_intersection = 0; % mettre le flag à 0 pour sortir de while ( et creé d'autre point) 
            k=k+1;
        end
    end
   

end
v(k) = 0.3;
k=k+1;

PP(:,N1+2)=milieu; % ajout du point de milieu

for i= 0:N2-1
    test_intersection = 1;
    x_min = x1-(i+1)*dx;
    x_max = x1-i*dx ;
    y_min = y2;
    y_max = y1;

    
%     a=[x1-i*dx       ;  y1]; 
%     b=[x1-i*dx       ;  y2];
%     c=[x1-(i+1)*dx   ;  y2];
%     d=[x1-(i+1)*dx   ;  y1];
    
     while (test_intersection == 1) 
        P=Particles_generator(x_min,x_max,y_min ,y_max,-pi/2,pi/2, 1,Obstacles);
        x= polyxpoly([PP(1,i+N1+1) P(1)],[PP(2,i+N1+1), P(2)],GrandObstacle(:,1),GrandObstacle(:,2));
        if(isempty(x))
            distance = ( PP(1:2,end)- P(1:2));
            if( distance > (x_max-x_min)/2)
                v(k) = 0.45;
            else 
                v(k) = 0.3;
            end
            PP(:,i+N1+3)=P;
            test_intersection = 0;
            k=k+1;
        end

     end
end

PP(:,N2+N1+3)=P; % ajout du pint de fin 

% affichage 
figure(Fig)
plot(PP(1,:),PP(2,:),'*k')
hold on
plot(PP(1,:),PP(2,:),'k')

end



