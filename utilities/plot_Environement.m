function [  ] = plot_Environement( Obstacles, Fig )
%PLOT_ENVIRONEMENT affiche les murs de l'enfironnement dans Fig
%  Obstacles : Les obstalces de l'environnement.
%  Fig : La figure courante obtenu tel que Fig=figure()

figure(Fig)

%Generation des murs
T=1;
nan=0/0;
OKWall=[];
OKColor=[0 0.8 0];
GreyWall=[];
GreyColor=[0 0 0.8];
NodetectWall=[];
NodetecColor='r';
for i=1:length(Obstacles)
    if Obstacles(i).Dist_Detect>100000 %Mur ok
        tmp=T*[Obstacles(i).Pos_vertex'; 1 1];
        OKWall=[OKWall;nan nan; tmp(1:2,:)' ;nan nan];
    elseif Obstacles(i).Dist_Detect==0 % Segment non detecté
        tmp=T*[Obstacles(i).Pos_vertex'; 1 1];
        NodetectWall=[NodetectWall;nan nan;tmp(1:2,:)';nan nan];
    else %Detection intermédiaire : gris
        tmp=T*[Obstacles(i).Pos_vertex'; 1 1];
        GreyWall=[GreyWall;nan nan;tmp(1:2,:)';nan nan];
    end
end

hold on
plot(OKWall(:,1),OKWall(:,2),'Color',OKColor);
hold on
plot(GreyWall(:,1),GreyWall(:,2),'Color',GreyColor);
hold on
plot(NodetectWall(:,1),NodetectWall(:,2),'Color',NodetecColor);

end

