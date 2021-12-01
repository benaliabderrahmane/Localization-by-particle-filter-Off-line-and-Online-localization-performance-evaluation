function [ Result,ximp,yimp ] = Mesures_US( RoboPose, Obstacles, Active, Noise, Mesurement_segmentation, CoefTexture_Wall )
%MESURES_US Simule l'estimationd es us fonctiond e pos ro et env
%   Detailed explanation goes here

%Robpos=[x,y,theta] %Position du robot
%Obstacles : LEs ostacles de la map.
%Active=[0/1 0/1.....] size 16 %Vecteur de booléen pour dir quels US sont
%actifs ou non
%Noise : 1 => Ajout du bruit autre=> pas de bruit.
%Mesurement_segmentation : Nombre de rayon tirer par chaque US pour simuler
%la dispertion du capteur.
%%%%%
%Result : Mesures
%ximp : X impact utilisé (exact)
%yimp : Y impact utilisé (exact)

ObstaclesComplet=Obstacles;

Et=[-5 +5]; % Amplitude d'ouverture du capteur US
Perror=0.05; % Pourcentage de bruit possile sur la mesure (1=100%) Temporaire

Et=Et.*(pi/180);
astep=(max(Et)-min(Et))/(Mesurement_segmentation+1);
angles=[];
for i=1:Mesurement_segmentation
    angles=[angles min(Et)+i*astep];
end

%Ini pos Capteurs
PosSensors=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
            90 50 30 10 -10 -30 -50 -90 -90 -130 -150 -170 170 150 130 90];
%Conv to rad
PosSensors(3,:)=PosSensors(3,:).*(pi/180);
PosSensors(1,1:8)=PosSensors(1,1:8)+0.08;
PosSensors(1,9:end)=PosSensors(1,9:end)-0.16;


PosSensors(:,Active(:)==0)=[];


for i=1:length(PosSensors(1,:))
    Tpose=[cos(PosSensors(3,i)) -sin(PosSensors(3,i)) PosSensors(1,i);
    sin(PosSensors(3,i)) cos(PosSensors(3,i)) PosSensors(2,i);
    0 0 1];
    tmp=Tpose*[0.1; 0;1];
    PosSensors(1:2,i)=tmp(1:2);
end


Tpose=[cos(RoboPose(3)) -sin(cos(RoboPose(3))) RoboPose(1);
    sin(RoboPose(3)) cos(RoboPose(3)) RoboPose(2);
    0 0 1];

tmp=Tpose*[PosSensors(1:2,:);ones(1,length(PosSensors(1,:)))];


%Init errors perfs
Mesures=[];
Pmax=4;
for i=1:length(PosSensors(1,:))
    [DIST,ximp,yimp,GrandObstacle]=USPATCH_act(Pmax,angles,Obstacles,tmp(1,i),tmp(2,i),PosSensors(3,i),0,CoefTexture_Wall);
    %[DIST,ximp,yimp,GrandObstacle]=USPATCH(Portee,angles,Obstacles,x,y,theta,Bruitage,CoefTexture_Wall)
    [D,I]=min(DIST);
    Mesures=[Mesures; ximp(I) yimp(I) D];
end

%Simple noise handelling.
if Noise==1
    Imax=Mesures(:,3)==Pmax;
    Mesures(:,3)=Mesures(:,3)+(Mesures(:,3).*(rand(length(Mesures(:,1)),1)*Perror)-Perror/2);
    Mesures(Mesures(:,3)>Pmax,3)=Pmax;
end


Result=Mesures(:,3);
ximp=Mesures(:,1);
yimp=Mesures(:,2);

end

